--[[
  DEPTH-TRIGGERED SERPENTINE SURVEY -- MARK 2  (GPS-anchored, two-phase)
  ArduPilot Rover, skid-steer catamaran.
  A require()-able MODULE driven by survey_dispatcher.lua. Belongs in
  scripts/modules/, NOT in scripts/.

  ---------------------------------------------------------------------------
  REVISION: failsafe hardening pass. Every change below is keyed to a finding
  in mk2_failsafe_review.md so you can diff intent, not just text.

  F1  Stops now change MODE. vehicle:set_desired_speed(0) is silently rejected
      by AR_WPNav::set_speed_max() (it returns false for anything under
      AR_WPNAV_SPEED_MIN = 0.05 m/s), so every old halt path was a print
      statement while the boat kept driving to a target up to MAX_LEG_DISTANCE_M
      away. enter_hold() commands MODE_HOLD instead. Rover only reads the mode
      switch on a CHANGE of switch position, so a script-commanded HOLD sticks
      until you physically move SWC -- and it drops the vehicle out of GUIDED,
      which closes the dispatcher's gate so this module stops being called.
  F2  ahrs:get_location() is bound as Location'Null and returns nil with no
      position estimate. It is now fetched ONCE per tick and nil-checked; a nil
      is a hold, not a Lua error that would take the dispatcher down with it.
  F3  last_valid_depth_ms no longer starts at 0. init() runs at BOOT but
      update() does not run until armed+GUIDED, so a 0 meant the stale-depth
      watchdog was already expired on the first tick -- one nil depth reading
      there used to declare phase 1 finished before a leg had run. The watchdog
      now also refuses to arm until at least one valid reading has been seen.
  F4  The nav-stall watchdog is pivot-aware. Distance is the wrong progress
      metric while the boat is pivoting in place; heading is. Any tick where
      yaw is still slewing resets the stall clock, and the timeout is raised.
  F5  OFFSET arrival is measured as the PROJECTION onto the offset bearing,
      not as a scalar radius. A boat has no brakes: coasting 2 m along the old
      forward heading used to satisfy a 2 m offset, flipping the bearing with
      no lateral movement at all and retracing the same line. Leg length for
      edge detection is projected onto the forward bearing for the same reason.
  F6  Depth debounce is time- and sample-based, not tick-based. The old
      "3 consecutive readings" counted three 100 ms ticks against a ~1 Hz NMEA
      sensor whose value ArduPilot holds for up to read_timeout_ms() = 3000, so
      it was usually the same sentence counted three times.
  F7  Every set_target_location() return value is checked. Rover's guided
      destination setter validates the destination and returns false when it
      rejects one -- notably against the geofence. MAX_LEG_DISTANCE_M is also
      dropped to a lake-sized value so it can live INSIDE a FENCE_RADIUS.
  F8  resume() added to the module contract. ModeGuided::_enter() calls
      wp_nav.init() and keeps no target, so flipping Hold -> Guided used to
      leave this module believing a target was loaded that ArduPilot had
      already discarded. resume() re-sends the current state's target.
  F9  preflight_check() warns over the GCS when the tunables below and the
      flight-controller parameters contradict each other.
  F11 MISSION_TIMEOUT_S bounds the run. Worst case is "comes home early".
  F12 Arrival tests now account for WP_RADIUS (default 2.0 m). ArduPilot marks
      a destination reached within that radius and a boat then holds station,
      so the last WP_RADIUS metres of any commanded step are never travelled.
      With the old OFFSET_DISTANCE_M of 2.0 m the offset target was ALREADY
      "reached" the moment it was set, so the boat never moved and the script
      waited forever for 2.0 m of travel -- which is the unexplained SITL hang
      in this project's notes ("groundspeed and wp_dist both sat near zero...
      even though the Lua-side target math was verified correct"). The
      nav-stall watchdog was papering over it. Spacing is now 6.0 m by default
      and both arrival tests are scaled off the real WP_RADIUS.

  Also changed: losing the depth signal outside STATE_FORWARD no longer halts.
  Sonar is not in the critical path for offsetting or for getting home, and
  stopping mid-lake is strictly worse than completing the trip back to the hub.
  It warns once and carries on; if the sensor is still dead when the next
  forward leg starts, THAT is where the edge logic catches it.
  ---------------------------------------------------------------------------

  PATTERN (unchanged in concept)
    base_bearing_deg  captured once per activation from the compass.
    start_loc         captured once -- the hub both sweep phases sweep from.
    FORWARD  aim MAX_LEG_DISTANCE_M along forward_bearing_deg so AR_WPNav holds
             a straight line by cross-track correction; depth triggers a turn.
    OFFSET   step OFFSET_DISTANCE_M along offset_bearing_deg, then flip the
             forward bearing 180 deg. New legs ignore depth until they have
             travelled FORWARD_COOLDOWN_DISTANCE_M (measured as distance, so an
             opening pivot does not eat the cooldown budget).
    Phase 1 sweeps base+90, phase 2 sweeps base-90, both from the same hub, so
    a mid-lake start still covers the whole lake. A phase ends when legs get
    short (EDGE_STREAK_COUNT of them) or the depth signal is lost mid-leg.

  API NOTE: Location_ud:offset_bearing() mutates in place and returns nothing.
  Always call it on a :copy().

  REQUIREMENTS: SCR_ENABLE=1; RNGFND1 configured and wet; SCR_USER1=2; armed
  and in GUIDED; WP_PIVOT_ANGLE / WP_PIVOT_RATE set for clean skid-steer
  pivots (note: the real parameter names are WP_PIVOT_*, not PIVOT_TURN_*).
  TEST IN SITL, THEN RESTRAINED ON THE BENCH, THEN WATER.
--]]

-- ===================== TUNABLES =====================
local DEPTH_THRESHOLD_M           = 5.0   -- turn when the bottom comes up to this
local DEPTH_CONFIRM_MS            = 1200  -- shallow must hold this long before it counts (F6)
local DEPTH_MIN_SAMPLES           = 2     -- ...across at least this many sonar updates (F6)
local DEPTH_SAMPLE_GAP_MS         = 500   -- an unchanged value this old still counts as a new sample
local DEPTH_STALE_TIMEOUT_S       = 3.0   -- no valid depth this long -> forward legs give up
local DEPTH_FIRST_FIX_TIMEOUT_S   = 10.0  -- refuse to survey at all if the sonar has never read since activation
local OFFSET_DISTANCE_M           = 6.0   -- strip spacing. MUST be comfortably larger than WP_RADIUS (F12)
local FORWARD_SPEED_MPS           = 1.0
local MAX_LEG_DISTANCE_M          = 60.0  -- far aim point AND safety cap. MUST sit inside FENCE_RADIUS (F7)
local FORWARD_COOLDOWN_DISTANCE_M = 3.0   -- ignore depth for this much travel at the start of a leg
local MIN_LEG_LENGTH_M            = 8.0   -- shorter completed leg => "near the sideways edge"
local EDGE_STREAK_COUNT           = 2     -- consecutive short legs before ending a phase
local RETURN_ARRIVE_RADIUS_M      = 3.0   -- floor only; the real test is max(this, 1.5 * WP_RADIUS) (F12)
local NAV_STALL_TIMEOUT_S         = 15.0  -- raised, and now pivot-aware (F4)
local NAV_STALL_PROGRESS_M        = 0.2
local TURNING_YAW_DEG_PER_TICK    = 0.5   -- yaw change per tick that still counts as "pivoting" (F4)
local MISSION_TIMEOUT_S           = 1800  -- 30 min hard cap on a survey (F11)
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
-- mission in progress. To genuinely restart a survey, set SCR_USER1 0 then 2.
local state
local base_bearing_deg, start_loc, offset_bearing_deg, forward_bearing_deg
local sweep_phase, edge_streak, final_return, long_legs_this_phase
local need_new_leg, leg_start_loc, leg_target, offset_start_loc, offset_target
local wp_radius_m
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

-- Signed distance from `from` to `to` measured ALONG `bearing_deg` (F5).
-- get_distance_NE returns a Vector2f of metres (x = North, y = East), and a
-- compass bearing decomposes as North = cos(b), East = sin(b).
local function distance_along(from, to, bearing_deg)
  local v = from:get_distance_NE(to)
  local b = math.rad(bearing_deg)
  return v:x() * math.cos(b) + v:y() * math.sin(b)
end

-- F12: the vehicle declares a destination reached once it is within WP_RADIUS
-- (default 2.0 m) and, being a boat, holds station there -- so it never travels
-- the last WP_RADIUS metres of any commanded step. Demanding the full commanded
-- distance is therefore a test that CANNOT pass when the step is comparable to
-- WP_RADIUS. That is the "commanded target simply was not pursued, groundspeed
-- and wp_dist both sat near zero indefinitely" hang recorded in this project's
-- notes: with OFFSET_DISTANCE_M and WP_RADIUS both 2.0 m, every offset target
-- was already "reached" the instant it was set.
local function offset_arrive_m()
  return math.max(OFFSET_DISTANCE_M - wp_radius_m, OFFSET_DISTANCE_M * 0.5)
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
    reason, final_return and " - final" or " - then phase 2"))
  if not command_target(start_loc, "return") then return end
  nav_stall_reset(now_ms)
end

-- Shared by both ways of concluding a phase is out of lake to sweep into:
-- the shrinking-leg-length streak, and losing the depth signal mid-leg.
local function reached_edge(now_ms, reason)
  -- F9 diagnostic: if not one leg in this phase ran to full length, the lake is
  -- almost certainly shallower than DEPTH_THRESHOLD_M -- every leg triggered
  -- immediately, the edge streak filled up, and nothing was actually surveyed.
  if long_legs_this_phase == 0 and reason == "ran out of lake sideways" then
    gcs:send_text(4, string.format(
      "SURVEY: WARN phase %d ended with no full-length leg - DEPTH_THRESHOLD_M %.1fm is probably deeper than this lake",
      sweep_phase, DEPTH_THRESHOLD_M))
  end
  long_legs_this_phase = 0
  if sweep_phase == 1 then
    sweep_phase = 2
    offset_bearing_deg = wrap_deg(base_bearing_deg - 90)
    edge_streak = 0
    final_return = false
  else
    final_return = true
  end
  begin_return(now_ms, reason)
end

-- ===================== PREFLIGHT SANITY (F9) =====================
-- Cheap cross-checks between the tunables above and what is actually set on
-- the flight controller. Warnings only -- it will not stop you flying.
local function preflight_check()
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
      gcs:send_text(4, "SURVEY: halted - set SCR_USER1 to 0 then 2 to restart the survey")
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
    gcs:send_text(6, string.format("SURVEY: hub set, base bearing %.0f, phase 1 sweep %.0f",
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

  -- F11: bound the run. Worst case is "comes home early".
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
      leg_target    = here:copy()
      leg_target:offset_bearing(forward_bearing_deg, MAX_LEG_DISTANCE_M)
      if not command_target(leg_target, "forward leg") then return end
      need_new_leg    = false
      shallow_since_ms, shallow_samples = nil, 0
      nav_stall_reset(now_ms)
      if DEBUG then
        gcs:send_text(6, string.format("SURVEY: phase %d forward leg, bearing %.0f",
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
      enter_hold("leg exceeded MAX_LEG_DISTANCE_M - lake bigger than expected, or depth not triggering")

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
        gcs:send_text(6, string.format("SURVEY: at hub, starting phase 2 sweep bearing %.0f",
          offset_bearing_deg))
      end
    elseif nav_stall_check(now_ms, dist, turning,
             function() command_target(start_loc, "return retry") end) then
      -- No safe "close enough" fallback for failing to get home.
      enter_hold("return-to-hub stalled after retry - GPS nav not responding")
    end
  end
end

gcs:send_text(6, "SURVEY: depth_survey_mk2.lua loaded")

return {
  NAME   = "mk2 GPS-anchored two-phase",
  init   = init,
  update = update,
  stop   = stop,
  resume = resume,
}
