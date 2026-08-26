--[[
  DEPTH-TRIGGERED GRID SURVEY -- MARK 3  (hub-anchored, 100m-capped, no drift)
  ArduPilot Rover, skid-steer catamaran.
  A require()-able MODULE driven by survey_dispatcher.lua. Belongs in
  scripts/modules/, NOT in scripts/.

  This REPLACES an earlier reactive two-phase mk3 entirely (not a patch of it).
  That design let each forward leg run however far depth happened to allow,
  never returning to a shared baseline between offsets -- over several legs
  the boat's position could drift arbitrarily far along the forward axis
  (one SITL run: +52m net drift over three legs that should have roughly
  cancelled), producing lopsided coverage that didn't reflect the real lake
  shape and revisited water instead of sweeping it once. Discarded outright,
  not tuned.

  ---------------------------------------------------------------------------
  DESIGN: fixed, hub-anchored grid of parallel survey strips, bounded by a
  known MAX_RANGE_M-radius circle centred on the hub. The true shoreline is
  still unknown and still discovered reactively (downward sonar, exactly as
  before) -- but the OUTER boundary is now known in advance (it's the range
  cap itself), which is what makes "cover almost all of the lake, bounded to
  100m, without revisiting" tractable without a pre-loaded lake boundary.

  Every strip k (perpendicular offset d_k = side * k * OFFSET_DISTANCE_M from
  the hub) has a chord half-length L_k = sqrt(MAX_RANGE_M^2 - d_k^2) inside
  the range circle. A strip is two legs, BOTH anchored to the hub, NEVER to
  wherever the boat currently is:
    APPROACH  drive to the strip's near end (entry)
    RUN       drive to the strip's far end (far) -- the actual survey pass,
              cut short by a confirmed shallow reading (real shore found)
  Anchoring both legs to the hub (via P_k = hub:offset_bearing(...)) means
  every commanded target is provably inside the range circle by construction
  -- not a reactive clip after the fact. It also means strip 0 and the
  side-switch need no special-case code: strip 0's APPROACH/RUN covers its
  own centreline in both directions like any other strip, and switching from
  side +1 to side -1 is just the next strip's APPROACH, which naturally
  becomes the one-time crossing back through the hub area.

  A side ends when it runs out of range budget (d_k >= MAX_RANGE_M, chord
  length collapses to zero) or STREAK_LIMIT consecutive RUN legs came back
  short (real shore found across the whole probed width) -- either way,
  advance_side_or_finish() hands off to the other side, or to final return.
  Nothing here is open-ended or reactive-length the way the discarded design
  was: every leg's target is fixed geometry computed from the hub before it
  is ever sent.

  Needs zero sin/cos/atan2 at runtime. offset_bearing() does its own trig in
  C++; strip geometry and leg progress are pure get_distance_NE()+dot-product
  arithmetic. The only transcendental call anywhere is one math.sqrt() per
  strip (not per tick).

  Inherited unchanged from the previous design (sound, orthogonal to the
  coverage-pattern rewrite):
    F1  Stops are mode changes, never a possibly-rejected zero-speed command.
    F2  ahrs:get_location() fetched once per tick, nil-checked -- a nil is a
        frozen tick, not a Lua error that would take the dispatcher with it.
    F6  Depth debounce: shallow must persist DEPTH_CONFIRM_MS AND across
        DEPTH_MIN_SAMPLES distinct sonar samples, because the NMEA backend
        holds a stale value for up to 3s.
    F7  Every set_target_location() return value is checked; a persistent
        rejection (fence, bad location) ends the leg rather than spinning a
        rejected command indefinitely.
    F8  resume() re-sends the CURRENT leg's target -- it must NOT rebuild the
        leg (that would reset the progress baseline from wherever the boat
        drifted to while the operator had manual control, corrupting arrival
        detection for the leg already in flight).

  New this file:
    - GPS dropout mid-leg freezes the nav-stall clock rather than letting a
      transient outage read as "target not being pursued."
    - Sonar hardware fault (no data, ever): debounce never confirms shallow,
      every strip runs its full geometric length -- degrades to "cover the
      full range-capped disk regardless of true shoreline." Safe and bounded,
      just no longer shoreline-adaptive. Expected degraded behaviour, not a
      bug.

  API NOTE: Location_ud:offset_bearing() mutates in place and returns nothing.
  Always call it on a :copy(). Location_ud:get_distance_NE(other) returns the
  (north, east) metre vector FROM self TO other.

  REQUIREMENTS: SCR_ENABLE=1; RNGFND1 configured and wet; SCR_USER1=3; armed
  and in GUIDED; WP_PIVOT_ANGLE / WP_PIVOT_RATE set for clean skid-steer
  pivots (the real parameter names are WP_PIVOT_*, not PIVOT_TURN_*).
  TEST IN SITL, THEN RESTRAINED ON THE BENCH, THEN WATER.
--]]

-- ===================== TUNABLES =====================
local MAX_RANGE_M                 = 100.0 -- hard cap: never further than this from the hub
local OFFSET_DISTANCE_M           = 10.0  -- strip spacing. Raised from earlier 4.0m default --
                                           -- at 4m spacing a 100m-radius grid is ~25 strips a
                                           -- side (hours). Tune against real boat speed/battery.
local DEPTH_THRESHOLD_M           = 1.0   -- turn when the bottom comes up to this
local DEPTH_CONFIRM_MS            = 1200  -- shallow must hold this long before it counts (F6)
local DEPTH_MIN_SAMPLES           = 2     -- ...across at least this many sonar updates (F6)
local DEPTH_SAMPLE_GAP_MS         = 500   -- an unchanged value this old still counts as a new sample
local DEPTH_STALE_TIMEOUT_S       = 3.0   -- no valid depth this long -> treat as reaching real shore
local DEPTH_FIRST_FIX_TIMEOUT_S   = 10.0  -- refuse to survey at all if the sonar has never read since activation
local STREAK_LIMIT                = 3     -- consecutive short RUN legs before ending a side. Each
                                           -- strip here spans nearly the whole disk radius, a far
                                           -- stronger probe than a reactive leg ever was, so this
                                           -- doesn't need to be conservative to avoid false positives.
local COOLDOWN_DISTANCE_M         = 3.0   -- ignore depth for this much progress at the start of
                                           -- every leg -- a new leg often begins exactly where the
                                           -- last one just triggered shallow, and without this the
                                           -- debounce re-confirms almost instantly on the same spot.
local MIN_LEG_LENGTH_M            = 8.0   -- a RUN leg shorter than this counts toward the streak
local RANGE_MARGIN_M              = 3.0   -- defensive pad on the out-of-range safety check
local FORWARD_SPEED_MPS           = 1.0
local NAV_STALL_TIMEOUT_S         = 15.0  -- pivot-aware (F4)
local NAV_STALL_PROGRESS_M        = 0.2
local TURNING_YAW_DEG_PER_TICK    = 0.5   -- yaw change per tick that still counts as "pivoting" (F4)
local MISSION_TIMEOUT_S           = 3600  -- outer absolute safety net. Raised from 1800s -- a full
                                           -- 100m-radius grid at 10m spacing is materially bigger
                                           -- than the old reactive design ever attempted (order of
                                           -- ~20 strips). May still exceed this on a real boat --
                                           -- expected given the range requirement, tune against
                                           -- measured battery endurance (F11).
local DONE_STATION_KEEP           = true  -- finish in LOITER (hold the hub) rather than HOLD (drift)
local DEBUG                       = true

-- ============ FIRMWARE CONSTANTS (verified in source) ============
local RNGFND_ORIENT_DOWN = 25
local RNGFND_STATUS_GOOD = 4    -- AP_RangeFinder.h: Status::Good
local MODE_HOLD          = 4    -- Rover/mode.h
local MODE_LOITER        = 5
local MODE_GUIDED        = 15

-- ===================== PHASES / SUB-LEGS =====================
local PHASE_SURVEY = 1
local PHASE_RETURN = 2
local PHASE_DONE   = 3
local PHASE_HALTED = 4   -- terminal: only a SCR_USER1 toggle restarts

local SUBLEG_APPROACH = 1
local SUBLEG_RUN      = 2

-- ===================== MUTABLE STATE =====================
-- All (re)assigned in init(), which the dispatcher calls every time the
-- module is activated, so switching away and back never resumes stale
-- progress. Re-entering GUIDED does NOT call init() -- it resumes the
-- mission in progress via resume(). To genuinely restart, set SCR_USER1 0
-- then 3.
local phase
local hub, hub_captured, forward_bearing_deg, offset_bearing_deg
local side, strip_k, leg_dir, sub_leg
local streak
local wp_radius_m

-- Current leg (both APPROACH/RUN legs and the final RETURN-to-hub leg use
-- these same fields).
local leg_start_loc, leg_unit_n, leg_unit_e, leg_length_m, current_target
local need_new_leg

local shallow_since_ms, shallow_samples
local last_depth_value, last_depth_sample_ms
local last_valid_depth_ms, have_valid_depth, depth_lost_warned
local failsafe_triggered, mission_done_announced, mission_start_ms
local last_yaw_rad
local nav_stall_last_dist, nav_stall_last_progress_ms, nav_stall_retried

-- ===================== SMALL HELPERS =====================
local function wrap_deg(d)
  d = d % 360
  if d < 0 then d = d + 360 end
  return d
end

local function wrap_pi(r)
  return (r + math.pi) % (2 * math.pi) - math.pi
end

-- F12-style margin: a target inside WP_RADIUS is "arrived" natively and the
-- vehicle holds station there, so demanding the exact full commanded
-- distance is a test with zero margin. Generalises the old offset_arrive_m()
-- to any leg length (short lateral APPROACH hops and long RUN passes alike).
local function arrive_m(len_m)
  return math.max(len_m - wp_radius_m * 1.25, len_m * 0.4)
end

-- ===================== FAILSAFE =====================
-- F1: the only thing that actually stops this boat is a mode change.
local function enter_hold(reason)
  if not failsafe_triggered then
    gcs:send_text(3, "SURVEY: HOLD - " .. reason)   -- 3 = ERROR
    failsafe_triggered = true
  end
  vehicle:set_mode(MODE_HOLD)
  phase = PHASE_HALTED
end

-- F7: a rejected target is a failure, not something to drive on through.
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

-- ===================== NAV STALL WATCHDOG =====================
-- Direction-agnostic: watches the leg's progress metric for any change at
-- all, re-sends once, and only then reports a stall.
-- F4: a pivot moves no distance, so any tick where yaw is still slewing
-- resets the clock -- otherwise a slow 180deg pivot reads as a dead nav
-- controller. Frozen (not reset, not advanced) during a GPS dropout so a
-- transient outage can't masquerade as "target not being pursued."
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

-- ===================== DEPTH =====================
-- Returns depth_m (or nil) and whether this counts as a NEW sensor sample.
-- The NMEA backend holds the last parsed value for up to 3s, so "the value
-- changed" is the only cheap proxy for "a new sentence arrived"; the
-- DEPTH_SAMPLE_GAP_MS clause keeps that from stalling over a flat bottom
-- where two genuine consecutive samples can be bit-identical.
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

-- ===================== LEG GEOMETRY =====================
-- Begin a leg toward `target`, anchoring the progress baseline to the
-- vehicle's position RIGHT NOW. Every target this module ever commands is
-- computed from `hub`, never from current position -- begin_leg() only uses
-- current position to establish the progress vector for arrival detection.
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

-- Signed progress along the current leg's own bearing (dot product in the
-- local North/East frame) -- projection, not a scalar radius, because a
-- boat has no brakes and could otherwise satisfy a short step by coasting.
local function leg_progress_m(here)
  local v = leg_start_loc:get_distance_NE(here)
  return v:x() * leg_unit_n + v:y() * leg_unit_e
end

-- Strip k's two endpoints on `side`, anchored entirely to `hub`. Returns
-- nil, nil if this strip is entirely outside the range circle (side
-- exhausted on range budget, not on real shore).
local function strip_endpoints(the_side, k, dir)
  local d_k = the_side * k * OFFSET_DISTANCE_M
  local arg = MAX_RANGE_M * MAX_RANGE_M - d_k * d_k
  if arg <= 0 then return nil, nil end
  local half_len = math.sqrt(arg)
  local p_k = hub:copy()
  p_k:offset_bearing(offset_bearing_deg, d_k)
  local entry = p_k:copy()
  entry:offset_bearing(forward_bearing_deg, -dir * half_len)
  local far = p_k:copy()
  far:offset_bearing(forward_bearing_deg, dir * half_len)
  return entry, far
end

-- Side `side` is done (range budget exhausted, or STREAK_LIMIT short strips
-- in a row -- real shore found across the whole probed width). Hand off to
-- the other side, or to final return if both are done.
local function advance_side_or_finish(now_ms)
  streak = 0
  if side == 1 then
    side = -1
    strip_k = 1   -- k=0 already covered by side 1's strip 0
    sub_leg = SUBLEG_APPROACH
    need_new_leg = true
  else
    phase = PHASE_RETURN
    need_new_leg = true
  end
end

-- ===================== PREFLIGHT SANITY =====================
local function preflight_check()
  local min_m = (param:get("RNGFND1_MIN_CM") or 0) / 100.0
  if min_m > 0 and DEPTH_THRESHOLD_M < min_m * 1.5 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN trigger band %.1f-%.1fm is very narrow; raise DEPTH_THRESHOLD_M or lower RNGFND1_MIN_CM",
      min_m, DEPTH_THRESHOLD_M))
  end
  if OFFSET_DISTANCE_M < wp_radius_m * 2.0 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN OFFSET_DISTANCE_M %.1fm vs WP_RADIUS %.1fm - the step sits inside the arrival radius; widen the spacing or lower WP_RADIUS",
      OFFSET_DISTANCE_M, wp_radius_m))
  end
  if OFFSET_DISTANCE_M >= MAX_RANGE_M / 3 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN OFFSET_DISTANCE_M %.1fm is wide relative to MAX_RANGE_M %.1fm - very few strips will fit",
      OFFSET_DISTANCE_M, MAX_RANGE_M))
  end
  local fence_on = (param:get("FENCE_ENABLE") or 0) > 0
  local fence_r  = param:get("FENCE_RADIUS") or 0
  if fence_on and fence_r > 0 and MAX_RANGE_M >= fence_r * 0.9 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN MAX_RANGE_M %.0fm vs FENCE_RADIUS %.0fm - strip targets may be rejected",
      MAX_RANGE_M, fence_r))
  end
end

-- ===================== LIFECYCLE =====================
local function init(now_ms)
  phase = PHASE_SURVEY

  hub, hub_captured = nil, false
  forward_bearing_deg, offset_bearing_deg = nil, nil

  side, strip_k, leg_dir, sub_leg = 1, 0, 1, SUBLEG_APPROACH
  streak = 0

  leg_start_loc, current_target = nil, nil
  leg_unit_n, leg_unit_e, leg_length_m = 0.0, 0.0, 0.0
  need_new_leg = true

  shallow_since_ms, shallow_samples = nil, 0
  last_depth_value, last_depth_sample_ms = nil, nil

  last_valid_depth_ms = now_ms          -- NOT 0 (see F3 note in earlier design history)
  have_valid_depth = false              -- watchdog stays disarmed until a real reading
  depth_lost_warned = false

  failsafe_triggered = false
  mission_done_announced = false
  mission_start_ms = now_ms
  last_yaw_rad = nil

  wp_radius_m = param:get("WP_RADIUS") or 2.0
  if wp_radius_m <= 0 then wp_radius_m = 2.0 end

  nav_stall_reset(now_ms)
  preflight_check()
end

-- F8: called when the vehicle re-enters armed+GUIDED. ModeGuided::_enter()
-- discards whatever target was set, so re-send it. Deliberately re-sends the
-- SAME target rather than rebuilding the leg -- rebuilding would reset
-- leg_start_loc/leg_unit_*/leg_length_m from wherever the boat drifted to
-- while the operator had manual control, corrupting arrival detection for
-- the leg already in flight.
local function resume(now_ms)
  nav_stall_reset(now_ms)
  if current_target ~= nil then
    command_target(current_target, "resume")
  else
    need_new_leg = true
  end
  if DEBUG then gcs:send_text(6, "SURVEY: re-entered GUIDED, target re-sent") end
end

-- F1: stop() must not silently do nothing. Called on a deliberate SCR_USER1
-- switch, so parking the boat where it is (it holds station on an arrived
-- waypoint) is the least surprising safe-neutral; HOLD is the fallback.
local function stop(now_ms)
  if arming:is_armed() and vehicle:get_mode() == MODE_GUIDED then
    local here = ahrs:get_location()
    if here == nil or not vehicle:set_target_location(here) then
      vehicle:set_mode(MODE_HOLD)
    end
  end
  phase = PHASE_HALTED
end

-- ===================== MAIN TICK =====================
local last_halt_hint_ms

local function update(now_ms)
  if phase == PHASE_HALTED then
    vehicle:set_mode(MODE_HOLD)
    if last_halt_hint_ms == nil or (now_ms - last_halt_hint_ms) > 5000 then
      gcs:send_text(4, "SURVEY: halted - set SCR_USER1 to 0 then 3 to restart the survey")
      last_halt_hint_ms = now_ms
    end
    return
  end
  if phase == PHASE_DONE then
    return
  end

  -- F2: one location fetch per tick, nil-checked.
  local here = ahrs:get_location()
  if here == nil then
    enter_hold("no position estimate (GPS/EKF)")
    return
  end

  -- Captured once per activation, never touched again.
  if not hub_captured then
    hub = here:copy()
    forward_bearing_deg = wrap_deg(math.deg(ahrs:get_yaw_rad()))
    offset_bearing_deg  = wrap_deg(forward_bearing_deg + 90)
    hub_captured = true
    mission_start_ms = now_ms   -- init() ran at boot; the mission starts NOW
    need_new_leg = true
    gcs:send_text(6, string.format("SURVEY: hub set, forward bearing %.0f, offset bearing %.0f",
      forward_bearing_deg, offset_bearing_deg))
  end

  -- F4: are we still slewing? Used to suspend the distance-based stall clock.
  local yaw = ahrs:get_yaw_rad()
  local turning = false
  if last_yaw_rad ~= nil then
    turning = math.abs(wrap_pi(yaw - last_yaw_rad)) > math.rad(TURNING_YAW_DEG_PER_TICK)
  end
  last_yaw_rad = yaw

  -- ---------- depth + watchdogs ----------
  local depth, depth_fresh = read_depth(now_ms)
  if depth ~= nil then
    last_valid_depth_ms = now_ms
    have_valid_depth    = true
    depth_lost_warned   = false
  end

  if (not have_valid_depth)
     and ((now_ms - mission_start_ms) > (DEPTH_FIRST_FIX_TIMEOUT_S * 1000)) then
    enter_hold("no valid depth reading since activation - check the sonar")
    return
  end

  -- depth_lost_warned is a latch: it only clears when a FRESH valid reading
  -- arrives (above), never just because a new leg started. Without that,
  -- "signal lost" (true for every tick until recovery) would re-trigger the
  -- leg-ending logic below on every single 100ms tick -- burning through
  -- many strips in a couple of seconds on any sensor blip. This latch makes
  -- it a one-shot event per loss episode, same intent as the old design's
  -- state-gated version of this check.
  local depth_signal_lost = have_valid_depth
    and ((now_ms - last_valid_depth_ms) > (DEPTH_STALE_TIMEOUT_S * 1000))
  local depth_lost_ends_leg = false
  if depth_signal_lost and not depth_lost_warned then
    depth_lost_warned = true
    if phase == PHASE_SURVEY and sub_leg == SUBLEG_RUN then
      -- Real shore can present as signal loss if the remaining open-water
      -- chord is shorter than the sensor's own settle time -- treat it as a
      -- confirmed-shallow equivalent, same as the debounce path below.
      depth_lost_ends_leg = true
    else
      -- Approach transit / return-to-hub: sonar isn't in the critical path
      -- for either, and stopping mid-lake is worse than finishing the trip.
      gcs:send_text(4, "SURVEY: depth signal lost - not needed for this leg, continuing")
    end
  end

  -- Outer absolute safety net.
  if ((now_ms - mission_start_ms) > (MISSION_TIMEOUT_S * 1000)) and phase == PHASE_SURVEY then
    phase = PHASE_RETURN
    need_new_leg = true
  end

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

  -- phase == PHASE_SURVEY
  if need_new_leg then
    local entry, far = strip_endpoints(side, strip_k, leg_dir)
    if entry == nil then
      advance_side_or_finish(now_ms)
      return
    end
    local target = (sub_leg == SUBLEG_APPROACH) and entry or far
    if not begin_leg(here, target, sub_leg == SUBLEG_APPROACH and "approach" or "run") then return end
    need_new_leg = false
    shallow_since_ms, shallow_samples = nil, 0
    if DEBUG then
      gcs:send_text(6, string.format("SURVEY: side %d strip %d %s, leg %.0fm",
        side, strip_k, sub_leg == SUBLEG_APPROACH and "approach" or "run", leg_length_m))
    end
  end

  local prog = leg_progress_m(here)
  local dist = leg_start_loc:get_distance(here)
  local hub_dist = hub:get_distance(here)

  -- A new leg often begins EXACTLY where the previous one just triggered
  -- shallow (a RUN starting right where its APPROACH ended on shore, or vice
  -- versa) -- the sensor is still reading the same shallow water, so without
  -- this the debounce would re-confirm almost instantly, ending the leg
  -- before any real travel. Ignore depth for the first COOLDOWN_DISTANCE_M
  -- of THIS leg's own progress.
  local in_cooldown = prog < COOLDOWN_DISTANCE_M

  -- F6: shallow must persist in time AND across sonar updates.
  if (not in_cooldown) and depth ~= nil and depth < DEPTH_THRESHOLD_M then
    if shallow_since_ms == nil then
      shallow_since_ms = now_ms
      shallow_samples  = 0
    end
    if depth_fresh then shallow_samples = shallow_samples + 1 end
  else
    shallow_since_ms = nil
    shallow_samples  = 0
  end
  local shallow_confirmed = (shallow_since_ms ~= nil)
    and ((now_ms - shallow_since_ms) >= DEPTH_CONFIRM_MS)
    and (shallow_samples >= DEPTH_MIN_SAMPLES)

  local arrived = (leg_length_m < 0.01) or (prog >= arrive_m(leg_length_m))
  local out_of_range = hub_dist > (MAX_RANGE_M + RANGE_MARGIN_M)
  local stalled = nav_stall_check(now_ms, dist, turning,
    function() command_target(current_target, "leg retry") end)

  if out_of_range then
    -- Defensive only: leg targets are always pre-computed inside the range
    -- circle, so this firing means the boat overshot. Graceful handoff, not
    -- a halt -- the lesson from the earlier design's MAX_LEG_DISTANCE_M bug.
    advance_side_or_finish(now_ms)
    return
  end

  if shallow_confirmed or depth_lost_ends_leg or arrived or stalled then
    if sub_leg == SUBLEG_RUN then
      if prog < MIN_LEG_LENGTH_M then
        streak = streak + 1
      else
        streak = 0
      end
    end
    if streak >= STREAK_LIMIT then
      advance_side_or_finish(now_ms)
      return
    end
    if sub_leg == SUBLEG_APPROACH then
      sub_leg = SUBLEG_RUN
    else
      sub_leg = SUBLEG_APPROACH
      strip_k = strip_k + 1
      leg_dir = -leg_dir
    end
    need_new_leg = true
  end
end

gcs:send_text(6, "SURVEY: depth_survey_mk3.lua loaded")

return {
  NAME   = "mk3 hub-anchored grid, 100m cap",
  init   = init,
  update = update,
  stop   = stop,
  resume = resume,
}
