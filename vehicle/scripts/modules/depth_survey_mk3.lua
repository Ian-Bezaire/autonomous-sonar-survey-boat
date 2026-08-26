--[[
  DEPTH-TRIGGERED SERPENTINE SURVEY -- MARK 3  (guaranteed both-sides, capped)
  ArduPilot Rover, skid-steer catamaran.
  A require()-able MODULE driven by survey_dispatcher.lua. Belongs in
  scripts/modules/, NOT in scripts/.

  Built from depth_survey_mk2.lua. Every F1-F12 fix from mk2's failsafe
  hardening pass is unchanged and inherited (pcall-safe dispatcher contract,
  nil-location guard, HOLD-not-zero-speed stop, F8 resume-on-reentry,
  projected offset arrival, pivot-aware nav-stall watchdog, etc.) -- see
  depth_survey_mk2.lua's own header for that history. Only the three things
  below are new.

  ---------------------------------------------------------------------------
  REVISION: mk3. Three changes, each closing a specific gap found by reading
  mk2's own logic (not a guess -- confirmed against mk2:270-300, mk2:400-484):

  1. INDEPENDENT PER-SIDE TIME BUDGET (the actual bug behind "only covers one
     side"). mk2 checked MISSION_TIMEOUT_S against the whole mission's
     elapsed time, not per side. If it fired while still on side 1
     (sweep_phase == 1), final_return was forced immediately and side 2 NEVER
     STARTED AT ALL -- regardless of how little time side 2 would actually
     have needed. mk3 gives each side its own SIDE_TIMEOUT_S: if a side runs
     out its budget, that side ends (exactly like reaching a real edge) and
     hands off to the other side, or to final return if both are done.
     MISSION_TIMEOUT_S still exists, but only as an outer absolute safety net
     that should rarely fire now that each side is bounded on its own.

  2. MAX_DIST_FROM_HUB_M safety cap. mk2's only defense against an
     unexpectedly large lake was MAX_LEG_DISTANCE_M -- a PER-LEG cap that, if
     exceeded, halted the entire survey outright (enter_hold). mk3 adds a
     genuine overall-roaming cap measured from the hub: forward-leg targets
     are clipped so they never land farther than MAX_DIST_FROM_HUB_M from
     start_loc, and running into that cap ends the current side gracefully
     (like reaching a real edge) instead of stopping the whole mission.
     MAX_LEG_DISTANCE_M's own halt-on-exceeded check remains as a secondary
     anomaly catch (the boat travelling past its own clipped target without
     ever triggering shallow or the hub cap first) -- it should now be rare.

  3. DEPTH_THRESHOLD_M lowered 5.0 -> 1.0 (shoreline is what should trigger a
     turn, not open water). The existing preflight_check() RNGFND1_MIN_CM
     cross-check (mk2:305-311) is unchanged code but is now far more likely
     to actually warn -- that's intentional, not a bug: a 1 m trigger needs
     RNGFND1_MIN_CM comfortably below 1 m or the sensor reports out-of-range
     instead of a valid near-shore reading right at the point that matters.
     Not a script value to invent; measure it (vehicle/params/verified.md).

  Known limitation, stated plainly: two sides at +/-90 deg from whatever
  bearing the compass read at activation, each zigzagging along the forward
  axis, covers a rectangle-ish envelope around the hub -- full coverage for
  any reasonably convex/rectangular lake from any start point inside it. It
  is NOT a geometric guarantee for a genuinely non-convex shoreline (e.g. an
  inlet running perpendicular to the sweep axes, outside any single leg's
  reach). That class of guarantee needs a pre-loaded boundary, out of scope
  here -- this module stays reactive, like mk1 and mk2.
  ---------------------------------------------------------------------------

  PATTERN (inherited from mk2, budget model changed)
    base_bearing_deg  captured once per activation from the compass.
    start_loc         captured once -- the hub both sides sweep from.
    FORWARD  aim (up to) MAX_LEG_DISTANCE_M along forward_bearing_deg, clipped
             to MAX_DIST_FROM_HUB_M from the hub, so AR_WPNav holds a straight
             line by cross-track correction; depth triggers a turn.
    OFFSET   step OFFSET_DISTANCE_M along offset_bearing_deg, then flip the
             forward bearing 180 deg. New legs ignore depth until they have
             travelled FORWARD_COOLDOWN_DISTANCE_M (measured as distance, so
             an opening pivot does not eat the cooldown budget).
    Side 1 sweeps base+90, side 2 sweeps base-90, both from the same hub, so
    a mid-lake start still covers the whole lake -- and each side gets its
    own SIDE_TIMEOUT_S, so side 2 always gets to run. A side ends when legs
    get short (EDGE_STREAK_COUNT of them), the depth signal is lost mid-leg,
    the hub-distance cap is reached, or the side's own time budget runs out.

  API NOTE: Location_ud:offset_bearing() mutates in place and returns nothing.
  Always call it on a :copy().

  REQUIREMENTS: SCR_ENABLE=1; RNGFND1 configured and wet; SCR_USER1=3; armed
  and in GUIDED; WP_PIVOT_ANGLE / WP_PIVOT_RATE set for clean skid-steer
  pivots (note: the real parameter names are WP_PIVOT_*, not PIVOT_TURN_*).
  TEST IN SITL, THEN RESTRAINED ON THE BENCH, THEN WATER.
--]]

-- ===================== TUNABLES =====================
local DEPTH_THRESHOLD_M           = 1.0   -- mk3: turn when the bottom comes up to this (was 5.0)
local DEPTH_CONFIRM_MS            = 1200  -- shallow must hold this long before it counts (F6)
local DEPTH_MIN_SAMPLES           = 2     -- ...across at least this many sonar updates (F6)
local DEPTH_SAMPLE_GAP_MS         = 500   -- an unchanged value this old still counts as a new sample
local DEPTH_STALE_TIMEOUT_S       = 3.0   -- no valid depth this long -> forward legs give up
local DEPTH_FIRST_FIX_TIMEOUT_S   = 10.0  -- refuse to survey at all if the sonar has never read since activation
local OFFSET_DISTANCE_M           = 4.0   -- strip spacing. MUST be comfortably larger than WP_RADIUS (F12)
local FORWARD_SPEED_MPS           = 1.0
local MAX_LEG_DISTANCE_M          = 60.0  -- far aim point; also a secondary anomaly cap (see MAX_DIST_FROM_HUB_M)
local MAX_DIST_FROM_HUB_M         = 150.0 -- mk3: overall roaming cap from the hub; ends a side gracefully, not a halt
local FORWARD_COOLDOWN_DISTANCE_M = 3.0   -- ignore depth for this much travel at the start of a leg
local MIN_LEG_LENGTH_M            = 8.0   -- shorter completed leg => "near the sideways edge"
local EDGE_STREAK_COUNT           = 2     -- consecutive short legs before ending a side
local RETURN_ARRIVE_RADIUS_M      = 3.0   -- floor only; the real test is max(this, 1.5 * WP_RADIUS) (F12)
local NAV_STALL_TIMEOUT_S         = 15.0  -- pivot-aware (F4)
local NAV_STALL_PROGRESS_M        = 0.2
local TURNING_YAW_DEG_PER_TICK    = 0.5   -- yaw change per tick that still counts as "pivoting" (F4)
local SIDE_TIMEOUT_S              = 750   -- mk3: each side's own time budget -- side 2 ALWAYS gets to run
local MISSION_TIMEOUT_S           = 1800  -- mk3: outer absolute safety net only (was the sole cap in mk2)
local DONE_STATION_KEEP           = true  -- finish in LOITER (hold the hub) rather than HOLD (drift)
local DEBUG                       = true

-- ============ FIRMWARE CONSTANTS (verified in source) ============
local RNGFND_ORIENT_DOWN = 25
local RNGFND_STATUS_GOOD = 4    -- AP_RangeFinder.h: Status::Good
local MODE_HOLD          = 4    -- Rover/mode.h
local MODE_LOITER        = 5
local MODE_GUIDED        = 15

-- ===================== STATES =====================
local STATE_FORWARD         = 1
local STATE_OFFSET          = 2
local STATE_RETURN_TO_START = 3
local STATE_DONE            = 4
local STATE_HALTED          = 5   -- terminal: only a SCR_USER1 toggle restarts

-- ===================== MUTABLE STATE =====================
-- All of this is (re)assigned in init(), which the dispatcher calls every time
-- the module is activated, so switching away and back never resumes stale
-- progress. NOTE: re-entering GUIDED does NOT call init() -- it resumes the
-- mission in progress. To genuinely restart a survey, set SCR_USER1 0 then 3.
local state
local base_bearing_deg, start_loc, offset_bearing_deg, forward_bearing_deg
local sweep_phase, edge_streak, final_return, long_legs_this_phase
local need_new_leg, leg_start_loc, leg_target, offset_start_loc, offset_target
local wp_radius_m
local shallow_since_ms, shallow_samples
local last_depth_value, last_depth_sample_ms
local last_valid_depth_ms, have_valid_depth, depth_lost_warned
local failsafe_triggered, mission_done_announced, mission_start_ms, side_start_ms
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

-- Signed distance from `from` to `to` measured ALONG `bearing_deg` (F5).
-- get_distance_NE returns a Vector2f of metres (x = North, y = East), and a
-- compass bearing decomposes as North = cos(b), East = sin(b).
local function distance_along(from, to, bearing_deg)
  local v = from:get_distance_NE(to)
  local b = math.rad(bearing_deg)
  return v:x() * math.cos(b) + v:y() * math.sin(b)
end

-- mk3: how far (up to desired_dist) can a leg run from `from` along
-- bearing_deg before it would land more than MAX_DIST_FROM_HUB_M from the
-- hub? Exact circle intersection in the local North/East frame (same
-- decomposition distance_along() uses). Returns 0 if `from` is already
-- outside the circle -- callers check that separately and treat it as
-- reaching the edge, not as a zero-length leg.
local function clip_to_hub_radius(from, bearing_deg, desired_dist)
  local v = start_loc:get_distance_NE(from)   -- from - hub, (north, east) metres
  local b = math.rad(bearing_deg)
  local cx, cy = math.cos(b), math.sin(b)
  local dot   = v:x() * cx + v:y() * cy
  local d0_sq = v:x() * v:x() + v:y() * v:y()
  local r_sq  = MAX_DIST_FROM_HUB_M * MAX_DIST_FROM_HUB_M
  local disc  = dot * dot - d0_sq + r_sq
  if disc < 0 then return 0 end
  local t_max = -dot + math.sqrt(disc)
  if t_max < 0 then t_max = 0 end
  return math.min(desired_dist, t_max)
end

-- F12: the vehicle declares a destination reached once it is within WP_RADIUS
-- (default 2.0 m) and, being a boat, holds station there -- so it never travels
-- the last WP_RADIUS metres of any commanded step. Demanding the full commanded
-- distance is therefore a test that CANNOT pass when the step is comparable to
-- WP_RADIUS. The achievable projected step is OFFSET_DISTANCE_M - wp_radius_m
-- in the worst case; the 1.25 factor buys margin back, the 0.4 floor keeps the
-- test meaningful if the spacing is ever set very tight.
local function offset_arrive_m()
  return math.max(OFFSET_DISTANCE_M - wp_radius_m * 1.25, OFFSET_DISTANCE_M * 0.4)
end

local function return_arrive_m()
  return math.max(RETURN_ARRIVE_RADIUS_M, wp_radius_m * 1.5)
end

-- ===================== FAILSAFE =====================
-- F1: the only thing that actually stops this boat is a mode change.
local function enter_hold(reason)
  if not failsafe_triggered then
    gcs:send_text(3, "SURVEY: HOLD - " .. reason)   -- 3 = ERROR
    failsafe_triggered = true
  end
  vehicle:set_mode(MODE_HOLD)
  state = STATE_HALTED
end

-- F7: a rejected target is a failure, not something to drive on through.
local function command_target(loc, what)
  if not vehicle:set_target_location(loc) then
    enter_hold("nav target rejected (" .. what .. ") - outside fence, or bad location")
    return false
  end
  -- must follow set_target_location: speed only applies once guided is in WP submode
  if not vehicle:set_desired_speed(FORWARD_SPEED_MPS) then
    gcs:send_text(4, "SURVEY: set_desired_speed rejected (" .. what .. ")")
  end
  return true
end

-- ===================== NAV STALL WATCHDOG =====================
-- set_target_location() is fire-and-forget; ArduPilot is trusted to drive the
-- rest of the way. In SITL that trust has occasionally been misplaced -- the
-- commanded target simply was not pursued, with no error and no failsafe. This
-- watches the leg's distance metric for any change at all (direction-agnostic),
-- re-sends once, and only then reports a stall.
-- F4: a pivot moves no distance, so any tick where yaw is still slewing resets
-- the clock. Otherwise a slow 180 deg pivot reads as a dead nav controller.
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
      nav_stall_last_progress_ms = now_ms   -- give the retry its own window
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
-- ArduPilot's NMEA backend holds the last parsed value for up to 3 s, so
-- "the value changed" is the only cheap proxy for "a new sentence arrived".
-- The DEPTH_SAMPLE_GAP_MS clause keeps that from stalling over a flat bottom
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

-- ===================== PHASE / RETURN =====================
local function begin_return(now_ms, reason)
  state = STATE_RETURN_TO_START
  gcs:send_text(6, string.format("SURVEY: returning to hub (%s)%s",
    reason, final_return and " - final" or " - then side 2"))
  if not command_target(start_loc, "return") then return end
  nav_stall_reset(now_ms)
end

-- Shared by every way of concluding a side is out of lake to sweep into: the
-- shrinking-leg-length streak, losing the depth signal mid-leg, the hub-
-- distance cap, and a side's own time budget running out.
local function reached_edge(now_ms, reason)
  -- F9 diagnostic: if not one leg in this side ran to full length, the lake is
  -- almost certainly shallower than DEPTH_THRESHOLD_M -- every leg triggered
  -- immediately, the edge streak filled up, and nothing was actually surveyed.
  if long_legs_this_phase == 0 and reason == "ran out of lake sideways" then
    gcs:send_text(4, string.format(
      "SURVEY: WARN side %d ended with no full-length leg - DEPTH_THRESHOLD_M %.1fm is probably deeper than this lake",
      sweep_phase, DEPTH_THRESHOLD_M))
  end
  long_legs_this_phase = 0
  if sweep_phase == 1 then
    sweep_phase = 2
    offset_bearing_deg = wrap_deg(base_bearing_deg - 90)
    edge_streak = 0
    final_return = false
    side_start_ms = now_ms   -- mk3: side 2 gets its own fresh time budget
  else
    final_return = true
  end
  begin_return(now_ms, reason)
end

-- ===================== PREFLIGHT SANITY (F9) =====================
-- Cheap cross-checks between the tunables above and what is actually set on
-- the flight controller. Warnings only -- it will not stop you flying.
local function preflight_check()
  -- mk3: DEPTH_THRESHOLD_M dropping to 1.0 makes this check far more likely
  -- to fire than it was in mk2 -- that is intentional, not a regression.
  local min_m = (param:get("RNGFND1_MIN_CM") or 0) / 100.0
  if min_m > 0 and DEPTH_THRESHOLD_M < min_m * 1.5 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN trigger band %.1f-%.1fm is very narrow; raise DEPTH_THRESHOLD_M or lower RNGFND1_MIN_CM",
      min_m, DEPTH_THRESHOLD_M))
  end
  if MIN_LEG_LENGTH_M < FORWARD_COOLDOWN_DISTANCE_M * 1.5 then
    gcs:send_text(4, "SURVEY: WARN MIN_LEG_LENGTH_M too close to the cooldown; normal legs will read as the lake edge")
  end
  if OFFSET_DISTANCE_M < wp_radius_m * 2.0 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN OFFSET_DISTANCE_M %.1fm vs WP_RADIUS %.1fm - the step sits inside the arrival radius; widen the spacing or lower WP_RADIUS",
      OFFSET_DISTANCE_M, wp_radius_m))
  end
  local fence_on = (param:get("FENCE_ENABLE") or 0) > 0
  local fence_r  = param:get("FENCE_RADIUS") or 0
  if fence_on and fence_r > 0 and MAX_LEG_DISTANCE_M >= fence_r * 0.9 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN MAX_LEG_DISTANCE_M %.0fm vs FENCE_RADIUS %.0fm - leg targets will be rejected",
      MAX_LEG_DISTANCE_M, fence_r))
  end
  -- mk3: the new hub-distance cap should also sit inside a fence, if one is set.
  if fence_on and fence_r > 0 and MAX_DIST_FROM_HUB_M >= fence_r * 0.9 then
    gcs:send_text(4, string.format(
      "SURVEY: WARN MAX_DIST_FROM_HUB_M %.0fm vs FENCE_RADIUS %.0fm - hub-distance cap may exceed the fence",
      MAX_DIST_FROM_HUB_M, fence_r))
  end
end

-- ===================== LIFECYCLE =====================
local function init(now_ms)
  state = STATE_FORWARD

  base_bearing_deg, start_loc = nil, nil
  offset_bearing_deg, forward_bearing_deg = nil, nil

  sweep_phase, edge_streak, final_return = 1, 0, false
  long_legs_this_phase = 0

  need_new_leg = true
  leg_start_loc, leg_target = nil, nil
  offset_start_loc, offset_target = nil, nil

  shallow_since_ms, shallow_samples = nil, 0
  last_depth_value, last_depth_sample_ms = nil, nil

  last_valid_depth_ms = now_ms          -- F3: NOT 0
  have_valid_depth = false              -- F3: watchdog stays disarmed until a real reading
  depth_lost_warned = false

  failsafe_triggered = false
  mission_done_announced = false
  mission_start_ms = now_ms
  side_start_ms = now_ms                -- mk3
  last_yaw_rad = nil

  wp_radius_m = param:get("WP_RADIUS") or 2.0
  if wp_radius_m <= 0 then wp_radius_m = 2.0 end

  nav_stall_reset(now_ms)
  preflight_check()
end

-- F8: called by the dispatcher when the vehicle re-enters armed+GUIDED.
-- ModeGuided::_enter() discarded whatever target we had set, so re-send it.
-- Deliberately re-sends the SAME leg target rather than starting a new leg,
-- so leg_start_loc (and therefore edge detection) is not corrupted by an
-- operator taking manual control for a few seconds mid-leg.
local function resume(now_ms)
  nav_stall_reset(now_ms)
  if state == STATE_FORWARD then
    if leg_target ~= nil then
      command_target(leg_target, "leg resume")
    else
      need_new_leg = true
    end
  elseif state == STATE_OFFSET and offset_target ~= nil then
    command_target(offset_target, "offset resume")
  elseif state == STATE_RETURN_TO_START and start_loc ~= nil then
    command_target(start_loc, "return resume")
  end
  if DEBUG then gcs:send_text(6, "SURVEY: re-entered GUIDED, target re-sent") end
end

-- F1: stop() must not silently do nothing. It is called on a deliberate
-- SCR_USER1 switch, so parking the boat where it is (a boat holds station on
-- an arrived waypoint) is the least surprising safe-neutral available; HOLD is
-- the fallback if that target is refused.
local function stop(now_ms)
  if arming:is_armed() and vehicle:get_mode() == MODE_GUIDED then
    local here = ahrs:get_location()
    if here == nil or not vehicle:set_target_location(here) then
      vehicle:set_mode(MODE_HOLD)
    end
  end
  state = STATE_HALTED
end

-- ===================== MAIN TICK =====================
local last_halt_hint_ms

local function update(now_ms)
  -- Terminal states first. Both of these have already commanded a mode change,
  -- so ArduPilot will normally have closed the dispatcher's gate and we will
  -- not be called at all -- this is here for the case where the operator flips
  -- back into GUIDED and would otherwise get a silent no-op.
  if state == STATE_HALTED then
    vehicle:set_mode(MODE_HOLD)
    if last_halt_hint_ms == nil or (now_ms - last_halt_hint_ms) > 5000 then
      gcs:send_text(4, "SURVEY: halted - set SCR_USER1 to 0 then 3 to restart the survey")
      last_halt_hint_ms = now_ms
    end
    return
  end
  if state == STATE_DONE then
    return
  end

  -- F2: one location fetch per tick, nil-checked. ahrs:get_location() is bound
  -- Location'Null and returns nil with no position estimate; using it blind
  -- raises a Lua error that would take the dispatcher down with this module
  -- and leave the boat driving to its last target.
  local here = ahrs:get_location()
  if here == nil then
    enter_hold("no position estimate (GPS/EKF)")
    return
  end

  -- Captured once per activation, never touched again.
  if base_bearing_deg == nil then
    base_bearing_deg    = wrap_deg(math.deg(ahrs:get_yaw_rad()))
    start_loc           = here:copy()
    offset_bearing_deg  = wrap_deg(base_bearing_deg + 90)
    forward_bearing_deg = base_bearing_deg
    mission_start_ms    = now_ms   -- init() ran at boot; the mission starts NOW
    side_start_ms       = now_ms   -- mk3: side 1's own budget starts now too
    gcs:send_text(6, string.format("SURVEY: hub set, base bearing %.0f, side 1 sweep %.0f",
      base_bearing_deg, offset_bearing_deg))
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

  -- Never surveyed with this sensor at all: refuse rather than sweep blind.
  if (not have_valid_depth)
     and ((now_ms - mission_start_ms) > (DEPTH_FIRST_FIX_TIMEOUT_S * 1000)) then
    enter_hold("no valid depth reading since activation - check the sonar")
    return
  end

  -- F3: watchdog only arms once a real reading has been seen.
  if have_valid_depth and ((now_ms - last_valid_depth_ms) > (DEPTH_STALE_TIMEOUT_S * 1000)) then
    if state == STATE_FORWARD then
      -- Losing depth mid-leg is how a real lake edge presents when the
      -- remaining open-water chord is shorter than the cooldown: the boat
      -- crosses the shore while depth checking is still suppressed, so no
      -- shallow reading is ever taken. Treat it as reaching the edge.
      reached_edge(now_ms, "lost depth signal")
      return
    elseif not depth_lost_warned then
      -- Offsetting or heading home. Sonar is not in the critical path for
      -- either, and stopping mid-lake is worse than finishing the trip back.
      gcs:send_text(4, "SURVEY: depth signal lost - not needed for this leg, continuing")
      depth_lost_warned = true
    end
  end

  -- mk3: per-side budget ends THIS side (hands off to the other side, or to
  -- final return) instead of truncating the whole mission -- this is the fix
  -- for the "side 2 never runs" bug. See header REVISION note 1.
  if ((now_ms - side_start_ms) > (SIDE_TIMEOUT_S * 1000))
     and state ~= STATE_RETURN_TO_START then
    reached_edge(now_ms, "side time budget reached")
    return
  end

  -- F11: outer absolute safety net. Should rarely fire now that each side is
  -- bounded on its own -- a script that can run forever is still its own
  -- hazard, so this stays as the final backstop. Worst case is "comes home early".
  if ((now_ms - mission_start_ms) > (MISSION_TIMEOUT_S * 1000))
     and state ~= STATE_RETURN_TO_START then
    final_return = true
    begin_return(now_ms, "mission timeout")
    return
  end

  -- ---------- state machine ----------
  if state == STATE_FORWARD then
    if need_new_leg then
      leg_start_loc = here:copy()

      -- mk3: hub-distance safety cap. If already at/past it, this side is
      -- done -- same as any other edge. Otherwise clip the aim point so it
      -- never lands farther than MAX_DIST_FROM_HUB_M from the hub.
      local hub_dist_now = start_loc:get_distance(here)
      if hub_dist_now >= MAX_DIST_FROM_HUB_M then
        reached_edge(now_ms, "reached max distance from hub")
        return
      end
      local aim_dist = clip_to_hub_radius(here, forward_bearing_deg, MAX_LEG_DISTANCE_M)

      leg_target = here:copy()
      leg_target:offset_bearing(forward_bearing_deg, aim_dist)
      if not command_target(leg_target, "forward leg") then return end
      need_new_leg    = false
      shallow_since_ms, shallow_samples = nil, 0
      nav_stall_reset(now_ms)
      if DEBUG then
        gcs:send_text(6, string.format("SURVEY: side %d forward leg, bearing %.0f",
          sweep_phase, forward_bearing_deg))
      end
    end

    -- F5: progress ALONG the leg for cooldown and edge detection; straight-line
    -- distance for the safety cap and the stall metric (deliberately the more
    -- conservative of the two in each case).
    local leg_along   = distance_along(leg_start_loc, here, forward_bearing_deg)
    local leg_dist    = leg_start_loc:get_distance(here)
    local in_cooldown = leg_along < FORWARD_COOLDOWN_DISTANCE_M

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

    if shallow_confirmed then
      shallow_since_ms, shallow_samples = nil, 0

      if leg_along < MIN_LEG_LENGTH_M then
        edge_streak = edge_streak + 1
      else
        edge_streak = 0
        long_legs_this_phase = long_legs_this_phase + 1
      end

      if edge_streak >= EDGE_STREAK_COUNT then
        reached_edge(now_ms, "ran out of lake sideways")
      else
        offset_start_loc = here:copy()
        offset_target    = here:copy()
        offset_target:offset_bearing(offset_bearing_deg, OFFSET_DISTANCE_M)
        if not command_target(offset_target, "offset") then return end
        state = STATE_OFFSET
        nav_stall_reset(now_ms)
        gcs:send_text(6, string.format("SURVEY: shallow %.1fm after %.0fm, offsetting",
          depth or -1, leg_along))
      end

    elseif leg_dist > MAX_LEG_DISTANCE_M then
      -- mk3: this is NOT rare with a tight DEPTH_THRESHOLD_M -- a bottom that
      -- hovers right at the trigger (textured, not a clean slope) can run out
      -- a full leg without ever holding "shallow" for DEPTH_CONFIRM_MS/
      -- DEPTH_MIN_SAMPLES straight. mk2 halted the WHOLE mission here; that
      -- would silently defeat the guaranteed-both-sides fix (side 1 hangs,
      -- side 2 never runs). Treat it exactly like any other edge instead --
      -- this side is done, hand off to the other side or final return.
      reached_edge(now_ms, "leg reached MAX_LEG_DISTANCE_M without a confirmed shallow reading")

    elseif nav_stall_check(now_ms, leg_dist, turning,
             function() command_target(leg_target, "forward leg retry") end) then
      reached_edge(now_ms, "forward leg stalled")
    end

  elseif state == STATE_OFFSET then
    -- F5: projection onto the offset bearing, not a scalar radius -- coasting
    -- forward must not be able to satisfy a sideways step.
    local along = distance_along(offset_start_loc, here, offset_bearing_deg)
    if along >= offset_arrive_m() then
      forward_bearing_deg = wrap_deg(forward_bearing_deg + 180)
      need_new_leg = true
      state = STATE_FORWARD
      if DEBUG then
        gcs:send_text(6, string.format("SURVEY: offset complete, cooldown %.1fm",
          FORWARD_COOLDOWN_DISTANCE_M))
      end
    elseif nav_stall_check(now_ms, along, turning,
             function() command_target(offset_target, "offset retry") end) then
      -- A 2 m step is not worth aborting a survey over.
      gcs:send_text(4, "SURVEY: offset stalled after retry, continuing from here")
      forward_bearing_deg = wrap_deg(forward_bearing_deg + 180)
      need_new_leg = true
      state = STATE_FORWARD
    end

  elseif state == STATE_RETURN_TO_START then
    local dist = start_loc:get_distance(here)
    if dist <= return_arrive_m() then
      if final_return then
        gcs:send_text(6, "SURVEY: back at hub, survey complete")
        mission_done_announced = true
        state = STATE_DONE
        -- F1: an actual stop. LOITER holds the hub; HOLD just cuts the motors
        -- and lets the boat drift, which on water is not the same thing.
        if DONE_STATION_KEEP then
          vehicle:set_mode(MODE_LOITER)
        else
          vehicle:set_mode(MODE_HOLD)
        end
      else
        forward_bearing_deg = base_bearing_deg
        need_new_leg = true
        state = STATE_FORWARD
        gcs:send_text(6, string.format("SURVEY: at hub, starting side 2 sweep bearing %.0f",
          offset_bearing_deg))
      end
    elseif nav_stall_check(now_ms, dist, turning,
             function() command_target(start_loc, "return retry") end) then
      -- No safe "close enough" fallback for failing to get home.
      enter_hold("return-to-hub stalled after retry - GPS nav not responding")
    end
  end
end

gcs:send_text(6, "SURVEY: depth_survey_mk3.lua loaded")

return {
  NAME   = "mk3 guaranteed both-sides sweep",
  init   = init,
  update = update,
  stop   = stop,
  resume = resume,
}
