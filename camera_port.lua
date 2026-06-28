------------------------------------------------------------------------
-- CustomCam - Freecam math adapted for the Intuix framework
--
-- Core camera math from Roblox Freecam:
--   - Spring damper for smooth interpolation
--   - FOV zoom factor for consistent pan speed at any zoom level
--   - CFrame composition: position * rotation * velocity offset
--
-- Framework contract:
--   - Dependencies may be injected globals OR required from GitHub URLs.
--     If your framework uses GitHub requires, add them at the top:
--       require("https://raw.githubusercontent.com/.../ClientInputGroup.luau")
--       require("https://raw.githubusercontent.com/.../InputType.luau")
--       require("https://raw.githubusercontent.com/.../input.luau")
--       require("https://raw.githubusercontent.com/.../register_camera_mode.luau")
--   - self.input.movementX/Y/Z injected by framework each frame
--   - self.camera_cframe and self.cam_position read by framework after update
--   - get_head_cframe passed to constructor by framework
--   - No return from module or update(); registration via register_camera_mode()
------------------------------------------------------------------------

------------------------------------------------------------------------
-- GitHub requires (Intuix pattern)
-- Uncomment and fill in the correct URLs for your framework's modules:
------------------------------------------------------------------------
-- require("https://raw.githubusercontent.com/.../ClientInputGroup.luau")
-- require("https://raw.githubusercontent.com/.../InputType.luau")
-- require("https://raw.githubusercontent.com/.../input.luau")
-- require("https://raw.githubusercontent.com/.../register_camera_mode.luau")

------------------------------------------------------------------------
-- Validate framework dependencies
-- If globals are injected by setfenv, these will exist.
-- If they need GitHub requires, the stubs above must be uncommented.
------------------------------------------------------------------------
--set_require_domain("https://raw.githubusercontent.com/blackshibe/deadline-insitux-core-scripts/master/")

--local _ClientInputGroup = ClientInputGroup
--local _InputType = InputType
--local _input = input
--local _register_camera_mode = register_camera_mode

--if not _ClientInputGroup then error("CustomCam: 'ClientInputGroup' not found. Inject it via setfenv or require() it from GitHub.") end
--if not _InputType then error("CustomCam: 'InputType' not found. Inject it via setfenv or require() it from GitHub.") end
--if not _input then error("CustomCam: 'input' not found. Inject it via setfenv or require() it from GitHub.") end
--if not _register_camera_mode then error("CustomCam: 'register_camera_mode' not found. Inject it via setfenv or require() it from GitHub.") end

local pi    = math.pi
local clamp = math.clamp
local exp   = math.exp
local rad   = math.rad
local sqrt  = math.sqrt
local tan   = math.tan

------------------------------------------------------------------------
-- Spring Damper
-- Critically-damped second-order system for smooth interpolation.
------------------------------------------------------------------------

local Spring = {}
Spring.__index = Spring

function Spring.new(freq, pos)
	local self = setmetatable({}, Spring)
	self.f = freq   -- stiffness / angular frequency
	self.p = pos    -- current position
	self.v = pos * 0 -- current velocity
	return self
end

function Spring:Update(dt, goal)
	local f = self.f * 2 * pi
	local p0 = self.p
	local v0 = self.v

	local offset = goal - p0
	local decay = exp(-f * dt)

	local p1 = goal + (v0 * dt - offset * (f * dt + 1)) * decay
	local v1 = (f * dt * (offset * f - v0) + v0) * decay

	self.p = p1
	self.v = v1

	return p1
end

function Spring:SetFreq(freq)
	self.f = freq
end

function Spring:Reset(pos)
	self.p = pos
	self.v = pos * 0
end

------------------------------------------------------------------------
-- CustomCam
------------------------------------------------------------------------

local CustomCam = {}
CustomCam.__index = CustomCam

-- Gain constants (from Freecam)
local NAV_GAIN   = Vector3.new(1, 1, 1) * 64
local PAN_GAIN   = Vector2.new(0.75, 1) * 8
local FOV_GAIN   = 300
local ROLL_GAIN  = -pi / 2
local PITCH_LIMIT = rad(90)

-- Spring stiffness defaults (higher = snappier, lower = smoother)
local VEL_STIFFNESS  = 1.5
local PAN_STIFFNESS  = 1.0
local FOV_STIFFNESS  = 4.0
local ROLL_STIFFNESS = 1.0

-- Mouse sensitivity scale (matches the framework's 0.0075 baseline)
local MOUSE_SENS_SCALE = 0.0075

------------------------------------------------------------------------
-- Constructor
-- get_head_cframe: function provided by the framework to get character head CFrame
------------------------------------------------------------------------
function CustomCam.new(get_head_cframe)
	local head_cframe = get_head_cframe and get_head_cframe() or CFrame.new()

	local self = {
		get_head_cframe = get_head_cframe,

		-- Camera state (read by framework after update)
		cam_position = CFrame.new(head_cframe.Position),
		camera_cframe = head_cframe,
		rot_x = 0,   -- pitch
		rot_y = 0,   -- yaw
		rot_z = 0,   -- roll
		fov    = 70,

		-- Pitch limits (framework may read these to clamp vertical rotation)
		-- NOTE: Named min_roll/max_roll to match the framework's expected property
		-- names, but the values are PITCH limits (±90°), not roll limits.
		-- Roll (rot_z) is NOT clamped in the Freecam math — it wraps naturally.
		min_roll = -PITCH_LIMIT,
		max_roll =  PITCH_LIMIT,

		-- self.input is injected by the framework with movementX/Y/Z each frame.
		-- We provide a safe default so the script doesn't crash if injection
		-- hasn't happened yet (e.g. on the very first frame).
		input = { movementX = 0, movementY = 0, movementZ = 0 },

		-- Extra input state (managed by our key binds)
		_rollInput  = 0,  -- -1 to 1
		_fovInput   = 0,  -- -1 to 1
		_speedMul   = 1,  -- shift = slow

		-- Springs
		_velSpring  = Spring.new(VEL_STIFFNESS, Vector3.new()),
		_panSpring  = Spring.new(PAN_STIFFNESS, Vector2.new()),
		_fovSpring  = Spring.new(FOV_STIFFNESS, 0),
		_rollSpring = Spring.new(ROLL_STIFFNESS, 0),

		-- Stiffness (tweakable at runtime)
		velStiffness  = VEL_STIFFNESS,
		panStiffness  = PAN_STIFFNESS,
		fovStiffness  = FOV_STIFFNESS,
		rollStiffness = ROLL_STIFFNESS,

		-- Key bind group
		_inputGroup = ClientInputGroup.new(),
	}

	setmetatable(self, CustomCam)
	self:_bindKeys()

	return self
end

------------------------------------------------------------------------
-- Bind extra keys (roll, fov, speed) via ClientInputGroup
------------------------------------------------------------------------
function CustomCam:_bindKeys()
	local ig = self._inputGroup

	-- Roll (Z = left, C = right)
	ig:bind_user_setting(function() self._rollInput = -1 end, InputType.Began, "lean_left")
	ig:bind_user_setting(function() self._rollInput = 0 end,  InputType.Ended, "lean_left")
	ig:bind_user_setting(function() self._rollInput = 1 end,  InputType.Began, "lean_right")
	ig:bind_user_setting(function() self._rollInput = 0 end,  InputType.Ended, "lean_right")

	-- FOV zoom (scroll or keys)
	ig:bind_key(function() self._fovInput = 1 end,  InputType.Began, false, Enum.KeyCode.Equals)
	ig:bind_key(function() self._fovInput = 0 end,  InputType.Ended, false, Enum.KeyCode.Equals)
	ig:bind_key(function() self._fovInput = -1 end, InputType.Began, false, Enum.KeyCode.Minus)
	ig:bind_key(function() self._fovInput = 0 end,  InputType.Ended, false, Enum.KeyCode.Minus)

	-- Speed modifier (shift = 0.25x)
	ig:bind_key(function() self._speedMul = 0.25 end, InputType.Began, false, Enum.KeyCode.LeftShift)
	ig:bind_key(function() self._speedMul = 1 end,    InputType.Ended, false, Enum.KeyCode.LeftShift)
end

------------------------------------------------------------------------
-- Initialize from current camera
------------------------------------------------------------------------
function CustomCam:init()
	local cframe = self.get_head_cframe and self.get_head_cframe() or CFrame.new()

	self.rot_x, self.rot_y, self.rot_z = cframe:ToEulerAnglesYXZ()
	self.cam_position = CFrame.new(cframe.Position)
	self.fov = 70

	self._velSpring:Reset(Vector3.new())
	self._panSpring:Reset(Vector2.new())
	self._fovSpring:Reset(0)
	self._rollSpring:Reset(0)
end

------------------------------------------------------------------------
-- Core camera math step (called every frame by the framework)
--
-- This is the Freecam StepFreecam math, adapted to the framework:
--   1. Feed inputs through springs for smooth interpolation
--   2. Compute FOV zoom factor to normalize pan speed
--   3. Update FOV, rotation, position
--   4. Compose final CFrame: position * rotation * velocity offset
--   5. Set self.camera_cframe and self.cam_position (framework reads these)
------------------------------------------------------------------------
function CustomCam:update(delta_time)
	local dt = delta_time

	-- Read mouse input from framework (with safety fallback)
	local mouse_delta = (input.get_mouse_delta and input.get_mouse_delta() or Vector2.new())
		* MOUSE_SENS_SCALE
		* (input.get_mouse_sensitivity and input.get_mouse_sensitivity() or 1)

	-- Build raw input vectors from framework-injected self.input
	-- self.input is overwritten each frame by the framework, but we
	-- initialized a safe default in the constructor as a fallback.
	local movInput = self.input or { movementX = 0, movementY = 0, movementZ = 0 }
	local movX = movInput.movementX or 0
	local movY = movInput.movementY or 0
	local movZ = movInput.movementZ or 0

	local vel = Vector3.new(
		movX,    -- right/left
		movZ,    -- up/down
		-movY    -- forward/back (negated to match Freecam convention)
	) * self._speedMul

	local pan = Vector2.new(
		-mouse_delta.Y,  -- pitch
		-mouse_delta.X   -- yaw
	)

	local fov  = self._fovInput
	local roll = self._rollInput

	-- Feed through springs for smooth interpolation
	local smoothVel  = self._velSpring:Update(dt, vel)
	local smoothPan  = self._panSpring:Update(dt, pan)
	local smoothFov  = self._fovSpring:Update(dt, fov)
	local smoothRoll = self._rollSpring:Update(dt, roll)

	-- Sync spring stiffness (in case tweaked at runtime)
	self._velSpring:SetFreq(self.velStiffness)
	self._panSpring:SetFreq(self.panStiffness)
	self._fovSpring:SetFreq(self.fovStiffness)
	self._rollSpring:SetFreq(self.rollStiffness)

	-- FOV zoom factor: normalizes pan/roll speed relative to current FOV
	-- At 70° FOV this equals 1.0; zooming in makes it larger (slowing pan proportionally)
	local zoomFactor = sqrt(tan(rad(70 / 2)) / tan(rad(self.fov / 2)))

	-- Update FOV (clamped 1-120)
	self.fov = clamp(
		self.fov + smoothFov * FOV_GAIN * (dt / zoomFactor),
		1, 120
	)

	-- Update rotation (pitch, yaw, roll)
	local panVector = smoothPan * PAN_GAIN * (dt / zoomFactor)
	self.rot_x = self.rot_x + panVector.X   -- pitch
	self.rot_y = self.rot_y + panVector.Y   -- yaw
	self.rot_z = self.rot_z + smoothRoll * ROLL_GAIN * (dt / zoomFactor)  -- roll

	-- Clamp pitch, wrap yaw
	self.rot_x = clamp(self.rot_x, -PITCH_LIMIT, PITCH_LIMIT)
	self.rot_y = self.rot_y % (2 * pi)

	-- Compose final CFrame: position * rotation * velocity offset
	-- Velocity offset is in local space (after rotation), so movement is relative to look direction
	local camera_cframe = self.cam_position
		* CFrame.fromOrientation(self.rot_x, self.rot_y, self.rot_z)
		* CFrame.new(smoothVel * NAV_GAIN * dt)

	-- Set properties the framework reads (no return needed)
	self.camera_cframe = camera_cframe
	self.cam_position = CFrame.new(camera_cframe.Position)
end

------------------------------------------------------------------------
-- Cleanup
------------------------------------------------------------------------
function CustomCam:destroy()
	self._inputGroup:disconnect_all_binds()
end

------------------------------------------------------------------------
-- Register with the framework (no module return)
------------------------------------------------------------------------
register_camera_mode("CustomCam", CustomCam)
