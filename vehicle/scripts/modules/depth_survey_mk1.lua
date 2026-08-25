--[[
  DEPTH-TRIGGERED SERPENTINE SURVEY PATTERN - MARK 1
  ArduPilot Rover, skid-steer catamaran

  PATTERN:
    1. Drive forward until depth < DEPTH_THRESHOLD_M (debounced)
    2. Pivot turn TURN_ANGLE_DEG in current direction
    3. Drive forward OFFSET_DISTANCE_M
    4. Pivot turn TURN_ANGLE_DEG again, same direction (net 180 flip)
    5. Turn direction flips for the NEXT cycle (right,right / left,left / repeat)
    6. Repeat from step 1

  This file is a require()-able MODULE, not a standalone script -- it's loaded
  and driven by survey_dispatcher.lua (scripts/survey_dispatcher.lua), which
  selects it via SCR_USER1 = 1. It does not self-schedule and does not gate on
  armed/GUIDED itself; the dispatcher owns both of those. Do not upload this
  file directly into scripts/ -- it belongs in scripts/modules/.

  REQUIREMENTS BEFORE RUNNING:
    - SCR_ENABLE = 1, rebooted
    - RNGFND1_TYPE set, RNGFND1_ORIENT = 25 (down), sonar confirmed working
    - SCR_USER1 = 1 (selects this module in survey_dispatcher.lua)
    - Vehicle ARMED and in GUIDED mode (idles otherwise; will NOT arm itself)
    - SKID-STEER rover able to pivot in place. In SITL:
        sim_vehicle.py -v Rover -f rover-skid
    - TEST IN SITL FIRST, then bench-test secured, then open water.

  API NOTE (ArduPilot 4.7+/4.8-dev):
    Uses ahrs:get_yaw_rad() and rangefinder:distance_orient() (returns METRES).
    vehicle:set_desired_turn_rate_and_speed(turn_rate, speed) takes turn_rate in
    DEGREES/sec (see Rover.cpp set_desired_turn_rate_and_speed(turn_rate_degs, ...)),
    NOT radians/sec -- despite the vehicle-agnostic Lua binding name. Passing a
    radian value here (e.g. 3.0 meant as ~172 deg/s) instead commands ~3 deg/s,
    silently turning the vehicle >50x slower than intended and starving
    TURN_TIMEOUT_S. ATC_STR_RAT_MAX (default 120 deg/s) is the achievable ceiling.

  TURN COMPLETION (fixed):
    Turn progress is now measured by INTEGRATING the per-step heading change
    (robust to angle wrap and to turns larger than 180 deg), instead of
    comparing against a single start yaw. A TURN_TIMEOUT_S failsafe stops the
    vehicle if a turn never completes (e.g. skid-steer rate tuning too weak),
    so it can never spin forever. Commanded turn rate ramps down linearly over
    the last TURN_DECEL_DEG degrees (floor TURN_MIN_RATE_FRACTION) to curb
    overshoot from momentum -- commanding full rate right up to the 90 deg
    check let the boat coast several extra degrees past target before the
    rate controller caught up.

  FORWARD COOLDOWN:
    Right after a turn/offset/turn cycle completes, the boat is often still
    over the same shallow spot that triggered it. Without a cooldown, the
    very next reading could immediately re-trigger a turn in the opposite
    direction (right-turn cycle followed instantly by a left-turn cycle,
    with no real forward progress between them). FORWARD_COOLDOWN_S makes
    FORWARD drive straight, ignoring depth entirely, for a few seconds after
    every completed cycle before depth-triggered turning resumes.

  TUNE THESE BEFORE YOUR FIRST RUN:
--]]

local DEPTH_THRESHOLD_M     = 5.0     -- trigger depth
local DEPTH_DEBOUNCE_COUNT  = 3       -- consecutive shallow readings required
local TURN_ANGLE_DEG        = 90
local OFFSET_DISTANCE_M     = 0.6096  -- 2 ft
local FORWARD_SPEED_MPS     = 1.0     -- tune to your boat
local TURN_RATE_DEGPS       = 60.0    -- deg/s; tune to your boat's pivot capability (cap: ATC_STR_RAT_MAX, default 120)
local TURN_DECEL_DEG        = 25.0    -- start ramping turn rate down this many degrees before target (fights overshoot from momentum)
local TURN_MIN_RATE_FRACTION = 0.25   -- floor on the rate scale during deceleration, so it doesn't crawl to a stop
local RNGFND_ORIENT_DOWN    = 25
local FORWARD_COOLDOWN_S    = 3.0     -- after completing a turn/offset/turn cycle, drive forward blind to depth this long before checking again (prevents an immediate re-trigger while still over the same shallow spot)
local DEPTH_STALE_TIMEOUT_S = 3.0     -- failsafe: no valid depth this long -> stop
local TURN_TIMEOUT_S        = 8.0     -- failsafe: a single turn taking longer -> stop
local DEBUG                 = true    -- print yaw/turn progress to Messages

-- state machine
local STATE_FORWARD  = 1
local STATE_TURN_1   = 2
local STATE_OFFSET   = 3
local STATE_TURN_2   = 4

-- mutable state -- assigned real starting values in init(), which the
-- dispatcher calls every time this module is (re)activated, so switching
-- away and back never resumes stale progress
local state
local turn_dir
local shallow_count
local last_valid_depth_ms
local offset_start_loc
local failsafe_triggered
local cooldown_until_ms -- while now_ms < this, FORWARD drives blind (no depth trigger check)

-- turn tracking (integrated)
local turn_accum_rad
local turn_last_yaw
local turn_start_ms
local turn_dbg_ms

local TURN_ANGLE_RAD = math.rad(TURN_ANGLE_DEG)

local function wrap_pi(angle)
  while angle > math.pi do angle = angle - 2 * math.pi end
  while angle < -math.pi do angle = angle + 2 * math.pi end
  return angle
end

local function get_depth_m()
  if not rangefinder:has_data_orient(RNGFND_ORIENT_DOWN) then
    return nil
  end
  if rangefinder:status_orient(RNGFND_ORIENT_DOWN) ~= 4 then -- 4 = Good
    return nil
  end
  return rangefinder:distance_orient(RNGFND_ORIENT_DOWN) -- METRES
end

local function stop_vehicle(reason)
  vehicle:set_desired_turn_rate_and_speed(0, 0)
  if not failsafe_triggered then
    gcs:send_text(4, "SURVEY: STOPPED - " .. reason) -- 4 = WARNING
    failsafe_triggered = true
  end
end

-- call when entering a turn state
local function begin_turn(now)
  turn_accum_rad = 0
  turn_last_yaw = ahrs:get_yaw_rad()
  turn_start_ms = now
  turn_dbg_ms = now
end

local function init(now_ms)
  state = STATE_FORWARD
  turn_dir = 1          -- 1 = right, -1 = left (right on first trigger)
  shallow_count = 0
  last_valid_depth_ms = 0
  offset_start_loc = nil
  failsafe_triggered = false
  cooldown_until_ms = 0
  turn_accum_rad = 0
  turn_last_yaw = 0
  turn_start_ms = 0
  turn_dbg_ms = 0
end

local function update(now_ms)
  if state == STATE_FORWARD then
    local depth = get_depth_m()
    local in_cooldown = now_ms < cooldown_until_ms

    if depth ~= nil then
      last_valid_depth_ms = now_ms
      failsafe_triggered = false
      if not in_cooldown then
        if depth < DEPTH_THRESHOLD_M then
          shallow_count = shallow_count + 1
        else
          shallow_count = 0
        end
      end
    end

    -- stale-depth failsafe stays live even during cooldown; losing the sensor is always unsafe
    if (now_ms - last_valid_depth_ms) > (DEPTH_STALE_TIMEOUT_S * 1000) then
      stop_vehicle("no valid depth data")
      return
    end

    if (not in_cooldown) and shallow_count >= DEPTH_DEBOUNCE_COUNT then
      shallow_count = 0
      begin_turn(now_ms)
      state = STATE_TURN_1
      gcs:send_text(6, "SURVEY: shallow trigger, turning " ..
        (turn_dir == 1 and "right" or "left"))
    else
      vehicle:set_desired_turn_rate_and_speed(0, FORWARD_SPEED_MPS)
    end

  elseif state == STATE_TURN_1 or state == STATE_TURN_2 then
    -- integrate heading change since last step
    local yaw = ahrs:get_yaw_rad()
    turn_accum_rad = turn_accum_rad + math.abs(wrap_pi(yaw - turn_last_yaw))
    turn_last_yaw = yaw

    if DEBUG and (now_ms - turn_dbg_ms) >= 1000 then
      turn_dbg_ms = now_ms
      gcs:send_text(6, string.format("SURVEY: turning... %.0f/%.0f deg (yaw %.0f)",
        math.deg(turn_accum_rad), TURN_ANGLE_DEG, math.deg(yaw)))
    end

    if turn_accum_rad >= TURN_ANGLE_RAD then
      vehicle:set_desired_turn_rate_and_speed(0, 0)
      if state == STATE_TURN_1 then
        offset_start_loc = ahrs:get_location()
        state = STATE_OFFSET
      else
        turn_dir = -turn_dir      -- flip for next cycle
        state = STATE_FORWARD
        shallow_count = 0
        cooldown_until_ms = now_ms + (FORWARD_COOLDOWN_S * 1000)
        gcs:send_text(6, string.format("SURVEY: cycle complete, forward cooldown %.1fs", FORWARD_COOLDOWN_S))
      end
    elseif (now_ms - turn_start_ms) > (TURN_TIMEOUT_S * 1000) then
      stop_vehicle("turn timeout - check skid-steer rate tuning (ATC_STR_RAT_*)")
    else
      local remaining_deg = TURN_ANGLE_DEG - math.deg(turn_accum_rad)
      local rate_scale = 1.0
      if remaining_deg < TURN_DECEL_DEG then
        rate_scale = math.max(TURN_MIN_RATE_FRACTION, remaining_deg / TURN_DECEL_DEG)
      end
      vehicle:set_desired_turn_rate_and_speed(turn_dir * TURN_RATE_DEGPS * rate_scale, 0)
    end

  elseif state == STATE_OFFSET then
    local dist = offset_start_loc:get_distance(ahrs:get_location())
    if dist >= OFFSET_DISTANCE_M then
      vehicle:set_desired_turn_rate_and_speed(0, 0)
      begin_turn(now_ms)
      state = STATE_TURN_2
    else
      vehicle:set_desired_turn_rate_and_speed(0, FORWARD_SPEED_MPS)
    end
  end
end

local function stop(now_ms)
  vehicle:set_desired_turn_rate_and_speed(0, 0)
end

gcs:send_text(6, "SURVEY: depth_survey_mk1.lua loaded")

return { NAME = "mk1 dead-reckoning", init = init, update = update, stop = stop }
