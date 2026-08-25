--[[
  SURVEY DISPATCHER
  ArduPilot Rover, skid-steer catamaran.

  Top-level entry point for the boat's pathfinding scripts. This is the ONLY
  file that belongs directly in scripts/ -- every algorithm lives in
  scripts/modules/ instead, as a require()-able module this dispatcher loads,
  activates and drives. ArduPilot's scripting engine only auto-runs .lua files
  found directly in scripts/ (non-recursive), so files under modules/ are inert
  until something require()s them -- which is what lets several algorithms sit
  on the flight controller at once without fighting for control.

  SELECTING AN ALGORITHM -- set SCR_USER1; the switch happens on the next tick,
  no re-upload and no reboot:
      0 -> idle (default; safe out of the box)
      1 -> depth_survey_mk1 (dead-reckoning)
      2 -> depth_survey_mk2 (GPS-anchored, two-phase)

  MODULE CONTRACT -- every entry in ALGORITHMS is a table with:
    NAME    string
    init    function(now_ms)  (re)initialise ALL mutable state. Called once
                              every time the module is activated.
    update  function(now_ms)  one tick. Only called while this module is active
                              AND the vehicle is armed and in GUIDED.
    stop    function(now_ms)  safe-neutral, nothing else. Called once before
                              the module is deactivated for any reason.
    resume  function(now_ms)  OPTIONAL. Called when the vehicle RE-ENTERS
                              armed+GUIDED having previously left it. See below.

  ---------------------------------------------------------------------------
  REVISION: failsafe hardening. Two changes, both from mk2_failsafe_review.md.

  F2  update() is wrapped in pcall. A module lives in this dispatcher's own
      chunk via require(), so an uncaught Lua error in a module used to kill
      the dispatcher too -- no more ticks, no more mode gate, and the boat
      still driving to whatever Guided target was last set. Now an error
      commands HOLD, deactivates the module, and the dispatcher keeps ticking.

  F8  The armed+GUIDED gate is edge-detected, and the module gets resume() on
      the rising edge. ModeGuided::_enter() calls wp_nav.init() and preserves
      no target, so the flip to Hold and back that you WILL do on the water
      used to leave the module believing a target was loaded that ArduPilot had
      already discarded -- the boat then sat still until a stall watchdog
      noticed. resume() re-sends it on the next tick instead.
      A module without resume() (mk1) is simply not called; nothing breaks.

  Note what is deliberately NOT here: re-entering GUIDED does not re-init the
  module, it resumes the mission in progress. To genuinely restart a survey,
  set SCR_USER1 to 0 and back.
  ---------------------------------------------------------------------------

  SAFETY NOTE: switching is stop() -> [lookup] -> init() inside a single tick,
  with no scheduler cycle in between, so no stale command from the outgoing
  module survives into the incoming one.
--]]

local ALGORITHMS = {
  [1] = require("depth_survey_mk1"),
  [2] = require("depth_survey_mk2"),
}

local UPDATE_PERIOD_MS = 100
local MODE_HOLD        = 4    -- Rover/mode.h
local MODE_GUIDED      = 15

local active_index   = nil
local active_module  = nil
local was_gate_open  = false

local function update_dispatch()
  local now = millis():toint()

  local new_index = math.floor((param:get("SCR_USER1") or 0) + 0.5)

  if new_index ~= active_index then
    if active_module ~= nil then
      -- a module's own stop() may itself fail; never let that kill the loop
      pcall(active_module.stop, now)
    end

    local new_module = ALGORITHMS[new_index]
    if new_module == nil then
      gcs:send_text(4, string.format("SURVEY: SCR_USER1=%d has no matching algorithm, pathfinding idle", new_index))
    else
      local ok, err = pcall(new_module.init, now)
      if ok then
        gcs:send_text(6, string.format("SURVEY: switched to '%s' (SCR_USER1=%d)", new_module.NAME, new_index))
      else
        gcs:send_text(3, "SURVEY: init failed, staying idle: " .. tostring(err))
        new_module = nil
      end
    end

    active_module  = new_module
    active_index   = new_index
    was_gate_open  = false   -- force a resume() on the next rising edge
  end

  local gate_open = (active_module ~= nil) and arming:is_armed() and (vehicle:get_mode() == MODE_GUIDED)

  if gate_open then
    -- F8: the vehicle has just come back under script control
    if not was_gate_open and active_module.resume ~= nil then
      pcall(active_module.resume, now)
    end

    -- F2: a module error must not take the supervisor with it
    local ok, err = pcall(active_module.update, now)
    if not ok then
      gcs:send_text(3, "SURVEY: module error, holding: " .. tostring(err))
      vehicle:set_mode(MODE_HOLD)
      active_module = nil          -- stays down until SCR_USER1 is toggled
      gate_open = false
    end
  end

  was_gate_open = gate_open

  return update_dispatch, UPDATE_PERIOD_MS
end

gcs:send_text(6, "SURVEY: dispatcher loaded -- SCR_USER1: 0=idle, 1=mk1 dead-reckoning, 2=mk2 GPS-anchored")

return update_dispatch, 1000
