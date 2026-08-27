--[[
  DEPTH-TRIGGERED EXPLORE-THEN-COVER SURVEY -- MARK 4
  ArduPilot Rover, skid-steer catamaran.
  A require()-able MODULE driven by survey_dispatcher.lua. Belongs in
  scripts/modules/, NOT in scripts/.

  Replaces mk3's reactive hub-anchored grid (a KNOWN, ARTIFICIAL boundary --
  a 100m circle around the hub, unrelated to the real shore) with a design
  that finds the REAL shore first and plans coverage against it. Five
  phases:

    SEEK    Drive a straight line on the activation heading until the first
            confirmed-shallow or sustained depth-loss gives shore contact.
            This is what lets the survey start from anywhere in the lake --
            mk3 could only ever cover a fixed-radius disk around wherever it
            happened to activate; SEEK finds the actual water body first.

    TRACE   Follow the shoreline by holding TRACE_DEPTH_M with proportional
            heading control, shore consistently on the boat's right. Records
            a decimated polygon (a vertex only every >VERTEX_MIN_DIST_M of
            travel or >VERTEX_MIN_HEADING_DEG of heading change -- a smooth
            curve would otherwise want a vertex every tick, which a 44KB
            heap cannot hold). Closes when the boat returns within
            CLOSE_RADIUS_M of the trace start after MIN_PERIMETER_M.

    PLAN    One-shot geometry on the closed polygon: principal axis via PCA
            (sets the survey line bearing), line spacing chosen so total
            line length (estimated as polygon Area / spacing -- exact for
            parallel scan lines regardless of concavity) fits the mission
            time budget at FORWARD_SPEED_MPS, then a scanline fill produces
            an ordered list of spans -- each an absolute-GPS start/end pair.
            A concave lake correctly yields two (or more) spans on one scan
            line. Reports span count and ETA over the GCS. Refuses to start
            (enter_hold) if the real scanned length does not fit the budget
            even at the widest allowed spacing -- this is a planning
            failure, not a mission to attempt and hope.

    COVER   Drive the spans serpentine, span by span (each span is itself an
            APPROACH-then-RUN pair of hub-anchored... no: SPAN-anchored,
            precomputed-in-PLAN legs, same two-leg mechanic as mk3's
            strips). Depth here is a SAFETY LIMIT ONLY -- the boundary is
            the traced polygon, already known, so a shallow reading during
            COVER means "something is off from what TRACE measured" (real
            depth is noisier than a single traced pass, or the polygon
            approximation cut a corner) and the correct response is to
            abort *that span* and move to the next one. It must never end
            the survey the way it did in mk3 -- there the depth threshold
            *was* the boundary; here it is a trip-wire.

    RETURN  Home, LOITER (or HOLD per DONE_STATION_KEEP), done.

  FAILURE MODES, STATED EXPLICITLY (not silently absorbed):

    - A closed trace loop that is too small, or does not enclose the SEEK
      point, is a mid-lake shoal or island -- not the shore. Detected with a
      point-in-polygon test against the original seek point. Response: push
      outward -- resume SEEK past the shoal along the SAME activation
      bearing, from wherever the shoal-trace closed, and try again. Capped
      at SHOAL_RETRY_LIMIT before giving up (enter_hold).

    - A trace that exceeds TRACE_MAX_DISTANCE_M (or fills MAX_VERTICES)
      without closing has failed to find a tractable shoreline. Falls back
      to mk3's fixed hub-anchored grid, but the range cap is the MAXIMUM
      distance from hub actually reached during the failed trace -- a
      *measured* bound, not an arbitrary constant, so the fallback still
      reflects real water this survey has already confirmed exists.

  MISSION_TIMEOUT_S and FORWARD_SPEED_MPS are what PLAN's spacing/ETA math
  is computed against. If either is a placeholder rather than a real
  measured number, PLAN's output is not meaningful -- it will confidently
  compute a spacing and an ETA from wrong inputs and never tell you they
  were wrong. Measure the boat's actual sustained cruise speed on water
  before trusting this module's ETA.

  Inherited VERBATIM from mk3 (sound, orthogonal to the phase/geometry
  redesign -- do not "improve" these without a specific reason):
    F1  Stops are mode changes, never a possibly-rejected zero-speed command.
    F2  ahrs:get_location() fetched once per tick, nil-checked -- a nil is a
        frozen tick, not a Lua error that would take the dispatcher with it.
    F6  Depth debounce: shallow must persist DEPTH_CONFIRM_MS AND across
        DEPTH_MIN_SAMPLES distinct sonar samples, because the NMEA backend
        holds a stale value for up to 3s.
    F7  Every set_target_location() return value is checked; a persistent
        rejection (fence, bad location) ends the leg rather than spinning a
        rejected command indefinitely.
    F8  resume() re-sends the CURRENT target -- it must NOT rebuild the leg
        (would reset the progress baseline from wherever the boat drifted to
        while the operator had manual control).

  MEMORY: script heap on an F4 (256KB total SRAM, ~44KB stock scripting
  heap) is the binding constraint, same as mk3's own header notes on this
  project's hardware. Polygon vertices and scan spans are stored as plain
  number arrays (east/north metres relative to the hub), sized ONCE at
  init() to MAX_VERTICES / MAX_SPANS and never grown -- no per-tick table
  allocation anywhere in TRACE's hot loop.

  API NOTE: Location_ud:offset_bearing() mutates in place and returns
  nothing. Always call it on a :copy(). Location_ud:get_distance_NE(other)
  returns the (north, east) metre vector FROM self TO other.

  REQUIREMENTS: SCR_ENABLE=1; RNGFND1 configured and wet; SCR_USER1=4;
  armed and in GUIDED; WP_PIVOT_ANGLE / WP_PIVOT_RATE set for clean
  skid-steer pivots (the real parameter names are WP_PIVOT_*, not
  PIVOT_TURN_*).

  Tested: untested. TEST IN SITL (against sim/lake_feeder_complex.py)
  BEFORE bench, BEFORE water. This is a materially more complex algorithm
  than mk2/mk3 and has not yet been run against anything.
--]]

-- ===================== TUNABLES =====================
-- These two are what PLAN's spacing/ETA computation is actually budgeted
-- against -- see header note. Get them right before trusting anything PLAN
-- prints.
local FORWARD_SPEED_MPS          = 1.0    -- MUST be a measured sustained cruise speed
local MISSION_TIMEOUT_S          = 3600   -- MUST be a real time budget, not a placeholder

local TRACE_DEPTH_M              = 2.5    -- shoreline-following target depth
local TRACE_SPEED_MPS            = 0.6    -- slower than cruise -- control fidelity over speed
local TRACE_STEER_GAIN_DEG_PER_M = 8.0    -- proportional gain, depth error (m) -> heading (deg)
local TRACE_STEER_MAX_DELTA_DEG  = 6.0    -- clamp per-tick heading change (noisy bottom, don't oscillate)
local TRACE_LOOKAHEAD_M          = 8.0    -- distance to the continuously-retargeted trace point
local TRACE_LOST_SIGNAL_STEER_DEG = 3.0   -- per-tick bias AWAY from shore while signal is out during TRACE

local VERTEX_MIN_DIST_M          = 5.0    -- decimation: new vertex only past this much travel
local VERTEX_MIN_HEADING_DEG     = 10.0   -- ...or this much heading change, whichever comes first
local MAX_VERTICES               = 48     -- preallocated once; hitting this is a TRACE failure (fallback grid)
local MAX_SPANS                  = 64     -- preallocated once; PLAN caps scanline output here

local CLOSE_RADIUS_M             = 8.0    -- loop is "closed" within this of the trace start
local MIN_PERIMETER_M            = 30.0   -- ...but only after travelling at least this far first
local SEEK_MAX_DISTANCE_M        = 300.0  -- give up looking for ANY shore past this (enter_hold)
local TRACE_MAX_DISTANCE_M       = 800.0  -- give up closing the loop past this (fallback grid)
local SHOAL_RETRY_LIMIT          = 3      -- how many mid-lake obstacles to push past before giving up

local MIN_LINE_SPACING_M         = 4.0    -- PLAN will not space lines tighter than this
local MAX_LINE_SPACING_M         = 40.0   -- ...or wider than this, regardless of budget pressure
local RETURN_BUDGET_FRACTION     = 0.10   -- reserve this fraction of MISSION_TIMEOUT_S for the trip home

local DEPTH_THRESHOLD_M          = 1.0    -- SEEK/fallback-grid: the boundary. COVER: a safety trip-wire only.
local DEPTH_CONFIRM_MS           = 1200   -- shallow must hold this long before it counts (F6)
local DEPTH_MIN_SAMPLES          = 2      -- ...across at least this many sonar updates (F6)
local DEPTH_SAMPLE_GAP_MS        = 500    -- an unchanged value this old still counts as a new sample
local DEPTH_STALE_TIMEOUT_S      = 3.0    -- no valid depth this long -> treat as a shore/safety signal
local DEPTH_FIRST_FIX_TIMEOUT_S  = 10.0   -- refuse to survey at all if the sonar never reads since activation
local COOLDOWN_DISTANCE_M        = 3.0    -- ignore depth for this much progress at the start of a COVER leg

local NAV_STALL_TIMEOUT_S        = 15.0   -- pivot-aware (F4)
local NAV_STALL_PROGRESS_M       = 0.2
local TURNING_YAW_DEG_PER_TICK   = 0.5

local ABSOLUTE_MAX_RANGE_M       = 300.0  -- outer defensive catch-all, independent of phase/geometry;
                                           -- should never fire in normal operation

-- Fallback grid only (mk3 pattern, entered when TRACE fails to close):
local GRID_OFFSET_DISTANCE_M     = 10.0
local GRID_STREAK_LIMIT          = 3
local GRID_MIN_LEG_LENGTH_M      = 8.0
local GRID_RANGE_MARGIN_M        = 3.0
local DONE_STATION_KEEP          = true   -- finish in LOITER (hold position) rather than HOLD (drift)
local DEBUG                      = true

-- ============ FIRMWARE CONSTANTS (verified in source) ============
local RNGFND_ORIENT_DOWN = 25
local RNGFND_STATUS_GOOD = 4    -- AP_RangeFinder.h: Status::Good
local MODE_HOLD          = 4    -- Rover/mode.h
local MODE_LOITER        = 5
local MODE_GUIDED        = 15

-- ===================== PHASES =====================
local PHASE_SEEK    = 1
local PHASE_TRACE   = 2
local PHASE_PLAN    = 3   -- never checked in update()'s dispatcher -- run_plan() runs synchronously
                          -- as a one-shot call from TRACE's closure detection, not its own tick
local PHASE_COVER   = 4
local PHASE_RETURN  = 5
local PHASE_GRID    = 6   -- fallback: mk3-style hub-anchored grid, measured radius
local PHASE_DONE    = 7
local PHASE_HALTED  = 8   -- terminal: only a SCR_USER1 toggle restarts

local SUBLEG_APPROACH = 1
local SUBLEG_RUN      = 2

-- ===================== MUTABLE STATE =====================
-- All (re)assigned in init(). Re-entering GUIDED does NOT call init() -- it
-- resumes the mission via resume(). To genuinely restart, set SCR_USER1 0
-- then 4.
local phase
local hub, base_bearing_deg
local mission_start_ms, failsafe_triggered, mission_done_announced
local last_yaw_rad
local wp_radius_m

-- SEEK
local seek_from_e, seek_from_n     -- hub-relative (north,east)... see local_ne()
local shoal_retry_count

-- TRACE
local trace_heading_deg
local trace_start_e, trace_start_n
local trace_perimeter_m
local trace_max_dist_m
local last_vertex_e, last_vertex_n, last_vertex_heading_deg
local prev_trace_e, prev_trace_n   -- previous tick's position, for perimeter accumulation
local vertex_count
local vertex_e, vertex_n           -- preallocated MAX_VERTICES arrays

-- PLAN output / COVER state
local span_count
local span_start_e, span_start_n, span_end_e, span_end_n   -- preallocated MAX_SPANS arrays
local cover_index, cover_sub_leg
local line_bearing_deg
local grid_radius_m                -- set only when falling back to PHASE_GRID

-- Generic current-leg state (SEEK's approach, COVER's spans, RETURN, GRID all use this)
local leg_start_loc, leg_unit_n, leg_unit_e, leg_length_m, current_target
local need_new_leg

-- Depth / debounce
local shallow_since_ms, shallow_samples
local last_depth_value, last_depth_sample_ms
local last_valid_depth_ms, have_valid_depth, depth_lost_warned

-- Nav stall watchdog
local nav_stall_last_dist, nav_stall_last_progress_ms, nav_stall_retried

-- Fallback grid (mirrors mk3)
local grid_side, grid_strip_k, grid_leg_dir, grid_sub_leg, grid_streak

-- ===================== SMALL HELPERS =====================
local function wrap_deg(d)
  d = d % 360
  if d < 0 then d = d + 360 end
  return d
end

local function wrap_pi(r)
  return (r + math.pi) % (2 * math.pi) - math.pi
end

local function heading_diff_deg(a, b)
  local d = (a - b) % 360
  if d > 180 then d = d - 360 end
  return d
end

-- Hub-relative (north, east) metres for any Location -- the single frame
-- every stored coordinate in this file uses (vertices, spans, seek/trace
-- anchors). Keeping everything in one consistent frame relative to a FIXED
-- point (the hub, captured once) is what makes the polygon geometry in
-- PLAN correct without repeated GPS round-trips.
local function local_ne(loc)
  local v = hub:get_distance_NE(loc)
  return v:x(), v:y()
end

local function loc_from_ne(n, e)
  local l = hub:copy()
  l:offset_bearing(0, n)     -- north
  l:offset_bearing(90, e)    -- east
  return l
end

-- F12-style margin, verbatim from mk3: a target inside WP_RADIUS is
-- "arrived" natively and the vehicle holds station there, so demanding the
-- exact full commanded distance is a test with zero margin.
local function arrive_m(len_m)
  return math.max(len_m - wp_radius_m * 1.25, len_m * 0.4)
end

-- ===================== FAILSAFE (verbatim F1) =====================
local function enter_hold(reason)
  if not failsafe_triggered then
    gcs:send_text(3, "SURVEY: HOLD - " .. reason)   -- 3 = ERROR
    failsafe_triggered = true
  end
  vehicle:set_mode(MODE_HOLD)
  phase = PHASE_HALTED
end

-- F7, verbatim.
local function command_target(loc, what)
  if not vehicle:set_target_location(loc) then
    enter_hold("nav target rejected (" .. what .. ") - outside fence, or bad location")
    return false
  end
  if not vehicle:set_desired_speed(FORWARD_SPEED_MPS) then
    gcs:send_text(4, "SURVEY: set_desired_speed rejected (" .. what .. ")")
  end
  return true
end

-- ===================== NAV STALL WATCHDOG (verbatim pattern) =====================
local function nav_stall_reset(now_ms)
  nav_stall_last_dist = nil
  nav_stall_last_progress_ms = now_ms
  nav_stall_retried = false
end

local function nav_stall_check(now_ms, dist, turning, resend_fn)
  if turning then
    nav_stall_last_progress_ms = now_ms
    nav_stall_last_dist = dist
    return false
  end
  if nav_stall_last_dist == nil or math.abs(dist - nav_stall_last_dist) > NAV_STALL_PROGRESS_M then
    nav_stall_last_dist = dist
    nav_stall_last_progress_ms = now_ms
    return false
  end
  if (now_ms - nav_stall_last_progress_ms) > (NAV_STALL_TIMEOUT_S * 1000) then
    if not nav_stall_retried then
      nav_stall_retried = true
      nav_stall_last_progress_ms = now_ms
      gcs:send_text(4, "SURVEY: nav target not progressing, re-sent it")
      resend_fn()
      return false
    end
    return true
  end
  return false
end

-- ===================== DEPTH (verbatim F6) =====================
local function read_depth(now_ms)
  if not rangefinder:has_data_orient(RNGFND_ORIENT_DOWN) then return nil, false end
  if rangefinder:status_orient(RNGFND_ORIENT_DOWN) ~= RNGFND_STATUS_GOOD then return nil, false end
  local d = rangefinder:distance_orient(RNGFND_ORIENT_DOWN)
  local fresh = (last_depth_value == nil)
             or (d ~= last_depth_value)
             or (last_depth_sample_ms == nil)
             or ((now_ms - last_depth_sample_ms) >= DEPTH_SAMPLE_GAP_MS)
  if fresh then
    last_depth_value = d
    last_depth_sample_ms = now_ms
  end
  return d, fresh
end

-- ===================== GENERIC LEG (SEEK / COVER spans / RETURN / GRID) =====================
local function begin_leg(here, target, what)
  leg_start_loc = here:copy()
  local v = leg_start_loc:get_distance_NE(target)
  leg_length_m = math.sqrt(v:x() * v:x() + v:y() * v:y())
  if leg_length_m < 0.01 then
    leg_unit_n, leg_unit_e = 0.0, 0.0
  else
    leg_unit_n, leg_unit_e = v:x() / leg_length_m, v:y() / leg_length_m
  end
  current_target = target:copy()
  local ok = command_target(current_target, what)
  nav_stall_reset(millis():toint())
  return ok
end

local function leg_progress_m(here)
  local v = leg_start_loc:get_distance_NE(here)
  return v:x() * leg_unit_n + v:y() * leg_unit_e
end

-- ===================== PREFLIGHT SANITY =====================
local function preflight_check()
  local min_m = (param:get("RNGFND1_MIN_CM") or 0) / 100.0
  if min_m > 0 and DEPTH_THRESHOLD_M < min_m * 1.5 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN trigger band %.1f-%.1fm is very narrow; raise DEPTH_THRESHOLD_M or lower RNGFND1_MIN_CM",
      min_m, DEPTH_THRESHOLD_M))
  end
  if TRACE_DEPTH_M < min_m * 1.5 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN TRACE_DEPTH_M %.1fm is close to RNGFND1_MIN_CM (%.1fm) - trace may see out-of-range instead of shallow",
      TRACE_DEPTH_M, min_m))
  end
  local fence_on = (param:get("FENCE_ENABLE") or 0) > 0
  local fence_r  = param:get("FENCE_RADIUS") or 0
  if fence_on and fence_r > 0 and SEEK_MAX_DISTANCE_M >= fence_r * 0.9 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN SEEK_MAX_DISTANCE_M %.0fm vs FENCE_RADIUS %.0fm - seek targets may be rejected",
      SEEK_MAX_DISTANCE_M, fence_r))
  end
end

-- ===================== TRACE HELPERS =====================
-- Point-in-polygon (ray casting), testing the point (0,0) in hub-relative
-- (north,east) space -- i.e. testing whether the closed trace encloses the
-- HUB. If the survey didn't start exactly at the seek point (it always
-- does here -- hub IS the seek origin) this would need its own test point;
-- keeping it hub-relative keeps the geometry uniform with everything else.
local function polygon_encloses_hub()
  local inside = false
  local j = vertex_count
  for i = 1, vertex_count do
    local ni, ei = vertex_n[i], vertex_e[i]
    local nj, ej = vertex_n[j], vertex_e[j]
    if ((ei > 0) ~= (ej > 0)) and
       (0 < (nj - ni) * (0 - ei) / (ej - ei) + ni) then
      inside = not inside
    end
    j = i
  end
  return inside
end

-- Shoelace formula, rotation-invariant -- used for PLAN's spacing estimate
-- (total scan length ~= Area / spacing) regardless of how the polygon is
-- oriented.
local function polygon_area()
  local a = 0.0
  local j = vertex_count
  for i = 1, vertex_count do
    a = a + (vertex_e[j] * vertex_n[i] - vertex_e[i] * vertex_n[j])
    j = i
  end
  return math.abs(a) * 0.5
end

-- ===================== LIFECYCLE =====================
local function reset_common(now_ms)
  failsafe_triggered = false
  mission_done_announced = false
  mission_start_ms = now_ms
  last_yaw_rad = nil
  need_new_leg = true
  leg_start_loc, current_target = nil, nil
  leg_unit_n, leg_unit_e, leg_length_m = 0.0, 0.0, 0.0

  shallow_since_ms, shallow_samples = nil, 0
  last_depth_value, last_depth_sample_ms = nil, nil
  last_valid_depth_ms = now_ms
  have_valid_depth = false
  depth_lost_warned = false

  wp_radius_m = param:get("WP_RADIUS") or 2.0
  if wp_radius_m <= 0 then wp_radius_m = 2.0 end

  nav_stall_reset(now_ms)
end

local function init(now_ms)
  phase = PHASE_SEEK
  hub, base_bearing_deg = nil, nil
  seek_from_e, seek_from_n = nil, nil
  shoal_retry_count = 0

  trace_heading_deg = nil
  trace_start_e, trace_start_n = nil, nil
  trace_perimeter_m = 0.0
  trace_max_dist_m = 0.0
  last_vertex_e, last_vertex_n, last_vertex_heading_deg = nil, nil, nil
  prev_trace_e, prev_trace_n = nil, nil
  vertex_count = 0
  vertex_e, vertex_n = {}, {}
  for i = 1, MAX_VERTICES do vertex_e[i], vertex_n[i] = 0.0, 0.0 end

  span_count = 0
  span_start_e, span_start_n = {}, {}
  span_end_e, span_end_n = {}, {}
  for i = 1, MAX_SPANS do
    span_start_e[i], span_start_n[i] = 0.0, 0.0
    span_end_e[i], span_end_n[i] = 0.0, 0.0
  end
  cover_index, cover_sub_leg = 1, SUBLEG_APPROACH
  line_bearing_deg = nil
  grid_radius_m = nil

  grid_side, grid_strip_k, grid_leg_dir, grid_sub_leg, grid_streak = 1, 0, 1, SUBLEG_APPROACH, 0

  reset_common(now_ms)
  preflight_check()
end

-- F8, verbatim pattern: re-send whatever target is in flight, never rebuild
-- the leg. TRACE has no fixed target (continuously retargeted) so it just
-- needs its stall clock reset; every other phase behaves exactly like mk3.
local function resume(now_ms)
  nav_stall_reset(now_ms)
  if phase == PHASE_TRACE then
    -- next tick's retarget picks up from wherever the vehicle actually is
  elseif current_target ~= nil then
    command_target(current_target, "resume")
  else
    need_new_leg = true
  end
  if DEBUG then gcs:send_text(6, "SURVEY: re-entered GUIDED, target re-sent") end
end

-- F1, verbatim.
local function stop(now_ms)
  if arming:is_armed() and vehicle:get_mode() == MODE_GUIDED then
    local here = ahrs:get_location()
    if here == nil or not vehicle:set_target_location(here) then
      vehicle:set_mode(MODE_HOLD)
    end
  end
  phase = PHASE_HALTED
end

-- ===================== PLAN (one-shot) =====================
-- Runs entirely inside a single tick: PCA for the line bearing, area-based
-- spacing choice against the mission budget, scanline fill, serpentine
-- ordering. Vertex/span counts are small (<= MAX_VERTICES / MAX_SPANS) so
-- this is cheap even done in one shot on an F4.
local function run_plan(now_ms)
  -- Principal axis via 2x2 PCA on the hub-relative vertices.
  local mean_e, mean_n = 0.0, 0.0
  for i = 1, vertex_count do
    mean_e = mean_e + vertex_e[i]
    mean_n = mean_n + vertex_n[i]
  end
  mean_e = mean_e / vertex_count
  mean_n = mean_n / vertex_count

  local sxx, syy, sxy = 0.0, 0.0, 0.0
  for i = 1, vertex_count do
    local de, dn = vertex_e[i] - mean_e, vertex_n[i] - mean_n
    sxx = sxx + de * de
    syy = syy + dn * dn
    sxy = sxy + de * dn
  end
  sxx, syy, sxy = sxx / vertex_count, syy / vertex_count, sxy / vertex_count

  local theta = 0.5 * math.atan(2 * sxy, sxx - syy)   -- math angle from east, CCW
  line_bearing_deg = wrap_deg(90 - math.deg(theta))
  local perp_bearing_deg = wrap_deg(line_bearing_deg + 90)
  local lb_rad, pb_rad = math.rad(line_bearing_deg), math.rad(perp_bearing_deg)

  -- Rotate vertices into (u = along-line, p = across-line) using the same
  -- north*cos + east*sin decomposition mk2/mk3 already used for bearings.
  local vertex_u, vertex_p = {}, {}
  local p_min, p_max = nil, nil
  for i = 1, vertex_count do
    local n, e = vertex_n[i], vertex_e[i]
    vertex_u[i] = n * math.cos(lb_rad) + e * math.sin(lb_rad)
    vertex_p[i] = n * math.cos(pb_rad) + e * math.sin(pb_rad)
    if p_min == nil or vertex_p[i] < p_min then p_min = vertex_p[i] end
    if p_max == nil or vertex_p[i] > p_max then p_max = vertex_p[i] end
  end

  local area = polygon_area()

  local elapsed_s = (now_ms - mission_start_ms) / 1000.0
  local remaining_s = MISSION_TIMEOUT_S - elapsed_s
  local cover_budget_s = remaining_s - (RETURN_BUDGET_FRACTION * MISSION_TIMEOUT_S)
  if cover_budget_s <= 0 then
    enter_hold(string.format(
      "no time budget left for PLAN (%.0fs elapsed of %.0fs mission)", elapsed_s, MISSION_TIMEOUT_S))
    return
  end

  local spacing_m = area / (cover_budget_s * FORWARD_SPEED_MPS)
  if spacing_m < MIN_LINE_SPACING_M then spacing_m = MIN_LINE_SPACING_M end
  if spacing_m > MAX_LINE_SPACING_M then spacing_m = MAX_LINE_SPACING_M end

  -- Scanline fill at the chosen spacing. xs[] is a small scratch buffer,
  -- reused every line -- not grown per-scanline.
  span_count = 0
  local xs = {}
  local p = p_min + spacing_m * 0.5
  while p < p_max and span_count < MAX_SPANS do
    local xcount = 0
    local j = vertex_count
    for i = 1, vertex_count do
      local pi, pj = vertex_p[i], vertex_p[j]
      if (pi > p) ~= (pj > p) then
        local t = (p - pi) / (pj - pi)
        xcount = xcount + 1
        xs[xcount] = vertex_u[i] + t * (vertex_u[j] - vertex_u[i])
      end
      j = i
    end
    -- insertion sort -- xcount is always small (a handful of edge crossings)
    for a = 2, xcount do
      local key = xs[a]
      local b = a - 1
      while b >= 1 and xs[b] > key do
        xs[b + 1] = xs[b]
        b = b - 1
      end
      xs[b + 1] = key
    end
    -- Reverse alternate lines (and the span order within them) for a
    -- serpentine traversal instead of a full transit back to one side
    -- after every single line.
    local reverse_line = (math.floor((p - (p_min + spacing_m * 0.5)) / spacing_m + 0.5) % 2) == 1
    local line_first_idx = span_count + 1
    local a = 1
    while a + 1 <= xcount and span_count < MAX_SPANS do
      span_count = span_count + 1
      local u0, u1 = xs[a], xs[a + 1]
      if reverse_line then u0, u1 = u1, u0 end
      -- Convert (u, p) back to hub-relative (north, east).
      span_start_n[span_count] = u0 * math.cos(lb_rad) + p * math.cos(pb_rad)
      span_start_e[span_count] = u0 * math.sin(lb_rad) + p * math.sin(pb_rad)
      span_end_n[span_count]   = u1 * math.cos(lb_rad) + p * math.cos(pb_rad)
      span_end_e[span_count]   = u1 * math.sin(lb_rad) + p * math.sin(pb_rad)
      a = a + 2
    end
    if reverse_line then
      -- Also reverse the ORDER of spans within this line (each span's own
      -- u0/u1 is already flipped above) for concave lakes with multiple
      -- spans per line, so consecutive spans in the final list stay
      -- geometrically adjacent instead of jumping across the whole line.
      local lo, hi = line_first_idx, span_count
      while lo < hi do
        span_start_n[lo], span_start_n[hi] = span_start_n[hi], span_start_n[lo]
        span_start_e[lo], span_start_e[hi] = span_start_e[hi], span_start_e[lo]
        span_end_n[lo], span_end_n[hi]     = span_end_n[hi], span_end_n[lo]
        span_end_e[lo], span_end_e[hi]     = span_end_e[hi], span_end_e[lo]
        lo, hi = lo + 1, hi - 1
      end
    end
    p = p + spacing_m
  end

  -- Real total length from the actual scan, not the area estimate -- this
  -- is the number PLAN refuses to start on if it doesn't fit.
  local total_len = 0.0
  for i = 1, span_count do
    local dn, de = span_end_n[i] - span_start_n[i], span_end_e[i] - span_start_e[i]
    total_len = total_len + math.sqrt(dn * dn + de * de)
  end
  local eta_s = total_len / FORWARD_SPEED_MPS

  gcs:send_text(6, string.format(
    "SURVEY: PLAN area=%.0fm2 spacing=%.1fm spans=%d length=%.0fm ETA=%.0fs budget=%.0fs",
    area, spacing_m, span_count, total_len, eta_s, cover_budget_s))

  if span_count == 0 then
    enter_hold("PLAN produced zero spans - polygon too small or degenerate")
    return
  end
  if eta_s > cover_budget_s then
    enter_hold(string.format(
      "PLAN does not fit budget: ETA %.0fs > %.0fs available even at %.1fm spacing",
      eta_s, cover_budget_s, spacing_m))
    return
  end

  cover_index, cover_sub_leg = 1, SUBLEG_APPROACH
  need_new_leg = true
  phase = PHASE_COVER
end

-- ===================== FALLBACK GRID (mk3 pattern, measured radius) =====================
local function grid_strip_endpoints(the_side, k, dir)
  local d_k = the_side * k * GRID_OFFSET_DISTANCE_M
  local arg = grid_radius_m * grid_radius_m - d_k * d_k
  if arg <= 0 then return nil, nil end
  local half_len = math.sqrt(arg)
  local p_k = hub:copy()
  p_k:offset_bearing(wrap_deg(base_bearing_deg + 90), d_k)
  local entry = p_k:copy()
  entry:offset_bearing(base_bearing_deg, -dir * half_len)
  local far = p_k:copy()
  far:offset_bearing(base_bearing_deg, dir * half_len)
  return entry, far
end

local function grid_advance_side_or_finish()
  grid_streak = 0
  if grid_side == 1 then
    grid_side = -1
    grid_strip_k = 1
    grid_sub_leg = SUBLEG_APPROACH
  else
    phase = PHASE_RETURN
  end
  need_new_leg = true
end

-- ===================== MAIN TICK =====================
local last_halt_hint_ms

local function update(now_ms)
  if phase == PHASE_HALTED then
    vehicle:set_mode(MODE_HOLD)
    if last_halt_hint_ms == nil or (now_ms - last_halt_hint_ms) > 5000 then
      gcs:send_text(4, "SURVEY: halted - set SCR_USER1 to 0 then 4 to restart the survey")
      last_halt_hint_ms = now_ms
    end
    return
  end
  if phase == PHASE_DONE then
    return
  end

  -- F2, verbatim.
  local here = ahrs:get_location()
  if here == nil then
    enter_hold("no position estimate (GPS/EKF)")
    return
  end

  if hub == nil then
    hub = here:copy()
    base_bearing_deg = wrap_deg(math.deg(ahrs:get_yaw_rad()))
    seek_from_e, seek_from_n = 0.0, 0.0   -- hub-relative; SEEK starts at the hub
    mission_start_ms = now_ms
    need_new_leg = true
    gcs:send_text(6, string.format("SURVEY: activated, seek bearing %.0f", base_bearing_deg))
  end

  local yaw = ahrs:get_yaw_rad()
  local turning = false
  if last_yaw_rad ~= nil then
    turning = math.abs(wrap_pi(yaw - last_yaw_rad)) > math.rad(TURNING_YAW_DEG_PER_TICK)
  end
  last_yaw_rad = yaw

  local depth, depth_fresh = read_depth(now_ms)
  if depth ~= nil then
    last_valid_depth_ms = now_ms
    have_valid_depth = true
    depth_lost_warned = false
  end

  if (not have_valid_depth)
     and ((now_ms - mission_start_ms) > (DEPTH_FIRST_FIX_TIMEOUT_S * 1000)) then
    enter_hold("no valid depth reading since activation - check the sonar")
    return
  end

  -- Rate-limits the GCS message only. Whether signal loss ENDS a leg is
  -- decided fresh per-phase below, cooldown-gated where relevant -- see
  -- mk3's own fix history for why a global one-shot latch here silently
  -- disarms detection for the rest of a continuous loss episode.
  local depth_signal_lost = have_valid_depth
    and ((now_ms - last_valid_depth_ms) > (DEPTH_STALE_TIMEOUT_S * 1000))
  if depth_signal_lost and not depth_lost_warned then
    depth_lost_warned = true
    gcs:send_text(4, "SURVEY: depth signal lost")
  end

  if ((now_ms - mission_start_ms) > (MISSION_TIMEOUT_S * 1000))
     and phase ~= PHASE_RETURN and phase ~= PHASE_DONE then
    phase = PHASE_RETURN
    need_new_leg = true
  end

  local hub_dist = hub:get_distance(here)
  if hub_dist > ABSOLUTE_MAX_RANGE_M then
    enter_hold(string.format("exceeded absolute range catch-all (%.0fm > %.0fm) - geometry fault", hub_dist, ABSOLUTE_MAX_RANGE_M))
    return
  end

  -- ================= PHASE_SEEK =================
  if phase == PHASE_SEEK then
    if need_new_leg then
      local from = loc_from_ne(seek_from_n, seek_from_e)
      local target = from:copy()
      target:offset_bearing(base_bearing_deg, SEEK_MAX_DISTANCE_M)
      if not begin_leg(here, target, "seek") then return end
      need_new_leg = false
      shallow_since_ms, shallow_samples = nil, 0
      if DEBUG then gcs:send_text(6, "SURVEY: seek, bearing " .. string.format("%.0f", base_bearing_deg)) end
    end

    local prog = leg_progress_m(here)
    local dist = leg_start_loc:get_distance(here)
    local in_cooldown = prog < COOLDOWN_DISTANCE_M

    if (not in_cooldown) and depth ~= nil and depth < DEPTH_THRESHOLD_M then
      if shallow_since_ms == nil then shallow_since_ms, shallow_samples = now_ms, 0 end
      if depth_fresh then shallow_samples = shallow_samples + 1 end
    else
      shallow_since_ms, shallow_samples = nil, 0
    end
    local shallow_confirmed = (shallow_since_ms ~= nil)
      and ((now_ms - shallow_since_ms) >= DEPTH_CONFIRM_MS)
      and (shallow_samples >= DEPTH_MIN_SAMPLES)
    local lost_ends = (not in_cooldown) and depth_signal_lost
    local arrived = (leg_length_m < 0.01) or (prog >= arrive_m(leg_length_m))
    local stalled = nav_stall_check(now_ms, dist, turning,
      function() command_target(current_target, "seek retry") end)

    if arrived then
      enter_hold(string.format("no shore found within SEEK_MAX_DISTANCE_M (%.0fm)", SEEK_MAX_DISTANCE_M))
      return
    end
    if stalled then
      enter_hold("seek stalled after retry - GPS nav not responding")
      return
    end
    if shallow_confirmed or lost_ends then
      -- First shore contact. Start TRACE from here, turning left 90 deg
      -- from the seek heading -- shore ends up on the boat's right.
      local n, e = local_ne(here)
      trace_start_n, trace_start_e = n, e
      trace_heading_deg = wrap_deg(base_bearing_deg - 90)
      trace_perimeter_m = 0.0
      prev_trace_n, prev_trace_e = n, e
      vertex_count = 1
      vertex_n[1], vertex_e[1] = n, e
      last_vertex_n, last_vertex_e = n, e
      last_vertex_heading_deg = trace_heading_deg
      trace_max_dist_m = hub:get_distance(here)
      nav_stall_reset(now_ms)
      phase = PHASE_TRACE
      gcs:send_text(6, "SURVEY: shore contact, tracing")
    end
    return
  end

  -- ================= PHASE_TRACE =================
  if phase == PHASE_TRACE then
    local n, e = local_ne(here)

    if depth ~= nil then
      local err = depth - TRACE_DEPTH_M
      local delta = TRACE_STEER_GAIN_DEG_PER_M * err
      if delta > TRACE_STEER_MAX_DELTA_DEG then delta = TRACE_STEER_MAX_DELTA_DEG end
      if delta < -TRACE_STEER_MAX_DELTA_DEG then delta = -TRACE_STEER_MAX_DELTA_DEG end
      trace_heading_deg = wrap_deg(trace_heading_deg + delta)
    elseif depth_signal_lost then
      -- No reading: could mean drifted into open water (exceeded
      -- RNGFND1_MAX) OR got too close to dry land (exceeded RNGFND1_MIN) --
      -- ambiguous which, and shoreline-hugging at a shallow TRACE_DEPTH_M
      -- makes the "too close to shore" case the more likely one in
      -- practice. Bias AWAY from shore (the safe direction under
      -- uncertainty) rather than assume it's safe to lean in further.
      trace_heading_deg = wrap_deg(trace_heading_deg - TRACE_LOST_SIGNAL_STEER_DEG)
    end

    local target = here:copy()
    target:offset_bearing(trace_heading_deg, TRACE_LOOKAHEAD_M)
    if not command_target(target, "trace") then return end
    vehicle:set_desired_speed(TRACE_SPEED_MPS)

    local step_n, step_e = n - prev_trace_n, e - prev_trace_e
    trace_perimeter_m = trace_perimeter_m + math.sqrt(step_n * step_n + step_e * step_e)
    prev_trace_n, prev_trace_e = n, e
    if hub_dist > trace_max_dist_m then trace_max_dist_m = hub_dist end

    local stalled = nav_stall_check(now_ms, trace_perimeter_m, turning,
      function() end)
    if stalled then
      enter_hold("trace stalled - not making progress along the shore")
      return
    end

    local dn, de = n - last_vertex_n, e - last_vertex_e
    local vertex_dist = math.sqrt(dn * dn + de * de)
    local vertex_hdg_change = math.abs(heading_diff_deg(trace_heading_deg, last_vertex_heading_deg))
    if vertex_dist > VERTEX_MIN_DIST_M or vertex_hdg_change > VERTEX_MIN_HEADING_DEG then
      if vertex_count >= MAX_VERTICES then
        gcs:send_text(4, "SURVEY: trace exceeded MAX_VERTICES, falling back to grid")
        grid_radius_m = trace_max_dist_m
        phase = PHASE_GRID
        need_new_leg = true
        return
      end
      vertex_count = vertex_count + 1
      vertex_n[vertex_count], vertex_e[vertex_count] = n, e
      last_vertex_n, last_vertex_e, last_vertex_heading_deg = n, e, trace_heading_deg
    end

    if trace_perimeter_m > TRACE_MAX_DISTANCE_M then
      gcs:send_text(4, "SURVEY: trace exceeded TRACE_MAX_DISTANCE_M, falling back to grid")
      grid_radius_m = trace_max_dist_m
      phase = PHASE_GRID
      need_new_leg = true
      return
    end

    if trace_perimeter_m > MIN_PERIMETER_M then
      local dsn, dse = n - trace_start_n, e - trace_start_e
      local dist_to_start = math.sqrt(dsn * dsn + dse * dse)
      if dist_to_start < CLOSE_RADIUS_M then
        -- Loop closed. Is it the real shore, or a mid-lake shoal?
        if polygon_encloses_hub() then
          gcs:send_text(6, string.format("SURVEY: trace closed, %d vertices, planning", vertex_count))
          run_plan(now_ms)
        else
          shoal_retry_count = shoal_retry_count + 1
          if shoal_retry_count > SHOAL_RETRY_LIMIT then
            enter_hold("too many mid-lake obstacles - can't find real shore")
            return
          end
          gcs:send_text(4, string.format(
            "SURVEY: closed loop did not enclose the seek point (shoal %d/%d) - pushing outward",
            shoal_retry_count, SHOAL_RETRY_LIMIT))
          seek_from_n, seek_from_e = n, e
          phase = PHASE_SEEK
          need_new_leg = true
        end
        return
      end
    end
    return
  end

  -- ================= PHASE_COVER =================
  if phase == PHASE_COVER then
    if need_new_leg then
      if cover_index > span_count then
        phase = PHASE_RETURN
        need_new_leg = true
        return
      end
      local target
      if cover_sub_leg == SUBLEG_APPROACH then
        target = loc_from_ne(span_start_n[cover_index], span_start_e[cover_index])
      else
        target = loc_from_ne(span_end_n[cover_index], span_end_e[cover_index])
      end
      if not begin_leg(here, target, cover_sub_leg == SUBLEG_APPROACH and "cover approach" or "cover run") then return end
      need_new_leg = false
      shallow_since_ms, shallow_samples = nil, 0
      if DEBUG then
        gcs:send_text(6, string.format("SURVEY: cover span %d/%d %s", cover_index, span_count,
          cover_sub_leg == SUBLEG_APPROACH and "approach" or "run"))
      end
    end

    local prog = leg_progress_m(here)
    local dist = leg_start_loc:get_distance(here)
    local in_cooldown = prog < COOLDOWN_DISTANCE_M

    if (not in_cooldown) and depth ~= nil and depth < DEPTH_THRESHOLD_M then
      if shallow_since_ms == nil then shallow_since_ms, shallow_samples = now_ms, 0 end
      if depth_fresh then shallow_samples = shallow_samples + 1 end
    else
      shallow_since_ms, shallow_samples = nil, 0
    end
    local shallow_confirmed = (shallow_since_ms ~= nil)
      and ((now_ms - shallow_since_ms) >= DEPTH_CONFIRM_MS)
      and (shallow_samples >= DEPTH_MIN_SAMPLES)
    local arrived = (leg_length_m < 0.01) or (prog >= arrive_m(leg_length_m))
    local stalled = nav_stall_check(now_ms, dist, turning,
      function() command_target(current_target, "cover retry") end)

    if shallow_confirmed then
      -- SAFETY LIMIT ONLY: the boundary is the traced polygon, not depth.
      -- Abort this span and move to the next one -- never end the survey.
      gcs:send_text(4, string.format(
        "SURVEY: shallow %.1fm mid-span %d, skipping to next span", depth or -1, cover_index))
      cover_index = cover_index + 1
      cover_sub_leg = SUBLEG_APPROACH
      need_new_leg = true
      return
    end

    if arrived or stalled then
      if cover_sub_leg == SUBLEG_APPROACH then
        cover_sub_leg = SUBLEG_RUN
      else
        cover_index = cover_index + 1
        cover_sub_leg = SUBLEG_APPROACH
      end
      need_new_leg = true
    end
    return
  end

  -- ================= PHASE_GRID (fallback) =================
  if phase == PHASE_GRID then
    if need_new_leg then
      local entry, far = grid_strip_endpoints(grid_side, grid_strip_k, grid_leg_dir)
      if entry == nil then
        grid_advance_side_or_finish()
        return
      end
      local target = (grid_sub_leg == SUBLEG_APPROACH) and entry or far
      if not begin_leg(here, target, grid_sub_leg == SUBLEG_APPROACH and "grid approach" or "grid run") then return end
      need_new_leg = false
      shallow_since_ms, shallow_samples = nil, 0
      if DEBUG then
        gcs:send_text(6, string.format("SURVEY: grid side %d strip %d %s (fallback, r=%.0fm)",
          grid_side, grid_strip_k, grid_sub_leg == SUBLEG_APPROACH and "approach" or "run", grid_radius_m))
      end
    end

    local prog = leg_progress_m(here)
    local dist = leg_start_loc:get_distance(here)
    local in_cooldown = prog < COOLDOWN_DISTANCE_M

    if (not in_cooldown) and depth ~= nil and depth < DEPTH_THRESHOLD_M then
      if shallow_since_ms == nil then shallow_since_ms, shallow_samples = now_ms, 0 end
      if depth_fresh then shallow_samples = shallow_samples + 1 end
    else
      shallow_since_ms, shallow_samples = nil, 0
    end
    local shallow_confirmed = (shallow_since_ms ~= nil)
      and ((now_ms - shallow_since_ms) >= DEPTH_CONFIRM_MS)
      and (shallow_samples >= DEPTH_MIN_SAMPLES)
    local lost_ends = (not in_cooldown) and depth_signal_lost
    local arrived = (leg_length_m < 0.01) or (prog >= arrive_m(leg_length_m))
    local out_of_range = hub_dist > (grid_radius_m + GRID_RANGE_MARGIN_M)
    local stalled = nav_stall_check(now_ms, dist, turning,
      function() command_target(current_target, "grid retry") end)

    if out_of_range then
      grid_advance_side_or_finish()
      return
    end
    if shallow_confirmed or lost_ends or arrived or stalled then
      if grid_sub_leg == SUBLEG_RUN then
        if prog < GRID_MIN_LEG_LENGTH_M then grid_streak = grid_streak + 1 else grid_streak = 0 end
      end
      if grid_streak >= GRID_STREAK_LIMIT then
        grid_advance_side_or_finish()
        return
      end
      if grid_sub_leg == SUBLEG_APPROACH then
        grid_sub_leg = SUBLEG_RUN
      else
        grid_sub_leg = SUBLEG_APPROACH
        grid_strip_k = grid_strip_k + 1
        grid_leg_dir = -grid_leg_dir
      end
      need_new_leg = true
    end
    return
  end

  -- ================= PHASE_RETURN =================
  if phase == PHASE_RETURN then
    if need_new_leg then
      if not begin_leg(here, hub, "return") then return end
      need_new_leg = false
      if DEBUG then gcs:send_text(6, "SURVEY: returning to hub") end
    end
    local prog = leg_progress_m(here)
    local dist = leg_start_loc:get_distance(here)
    local arrived = (leg_length_m < 0.01) or (prog >= arrive_m(leg_length_m))
    if arrived then
      gcs:send_text(6, "SURVEY: back at hub, survey complete")
      mission_done_announced = true
      phase = PHASE_DONE
      if DONE_STATION_KEEP then
        vehicle:set_mode(MODE_LOITER)
      else
        vehicle:set_mode(MODE_HOLD)
      end
    elseif nav_stall_check(now_ms, dist, turning,
             function() command_target(hub, "return retry") end) then
      enter_hold("return-to-hub stalled after retry - GPS nav not responding")
    end
    return
  end
end

gcs:send_text(6, "SURVEY: depth_survey_mk4.lua loaded")

return {
  NAME   = "mk4 explore-then-cover",
  init   = init,
  update = update,
  stop   = stop,
  resume = resume,
}
