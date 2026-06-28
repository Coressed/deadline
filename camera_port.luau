local pi    = math.pi
local abs   = math.abs
local clamp = math.clamp
local exp   = math.exp
local rad   = math.rad
local sign  = math.sign
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
	self.v = pos*0  -- current velocity
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
-- CustomCam Module
------------------------------------------------------------------------

local CustomCam = {}
CustomCam.__index = CustomCam

-- Default gain constants (same as Freecam)
local DEFAULT_NAV_GAIN  = Vector3.new(1, 1, 1) * 64
local DEFAULT_PAN_GAIN  = Vector2.new(0.75, 1) * 8
local DEFAULT_FOV_GAIN  = 300
local DEFAULT_ROLL_GAIN  = -pi / 2
local DEFAULT_PITCH_LIMIT = rad(90)

-- Default spring stiffness values
local DEFAULT_VEL_STIFFNESS = 1.5
local DEFAULT_PAN_STIFFNESS = 1.0
local DEFAULT_FOV_STIFFNESS = 4.0
local DEFAULT_ROLL_STIFFNESS = 1.0

function CustomCam.new()
	local self = {
		-- Gain multipliers
		navGain    = DEFAULT_NAV_GAIN,
		panGain    = DEFAULT_PAN_GAIN,
		fovGain    = DEFAULT_FOV_GAIN,
		rollGain   = DEFAULT_ROLL_GAIN,
		pitchLimit = DEFAULT_PITCH_LIMIT,

		-- Spring stiffness (higher = snappier, lower = smoother)
		velStiffness = DEFAULT_VEL_STIFFNESS,
		panStiffness = DEFAULT_PAN_STIFFNESS,
		fovStiffness = DEFAULT_FOV_STIFFNESS,
		rollStiffness = DEFAULT_ROLL_STIFFNESS,

		-- Internal state
		_cameraPos = Vector3.new(),
		_cameraRot = Vector3.new(),  -- x=pitch, y=yaw, z=roll
		_cameraFov = 70,

		-- Springs
		_velSpring  = Spring.new(DEFAULT_VEL_STIFFNESS, Vector3.new()),
		_panSpring  = Spring.new(DEFAULT_PAN_STIFFNESS, Vector2.new()),
		_fovSpring  = Spring.new(DEFAULT_FOV_STIFFNESS, 0),
		_rollSpring = Spring.new(DEFAULT_ROLL_STIFFNESS, 0),
	}

	setmetatable(self, CustomCam)
	return self
end

------------------------------------------------------------------------
-- Initialize camera state from a given CFrame + FOV
------------------------------------------------------------------------
function CustomCam:init(cframe, fov)
	cframe = cframe or CFrame.new()
	fov = fov or 70

	self._cameraRot = Vector3.new(cframe:ToEulerAnglesYXZ())
	self._cameraPos = cframe.Position
	self._cameraFov = fov

	self._velSpring  = Spring.new(self.velStiffness, Vector3.new())
	self._panSpring  = Spring.new(self.panStiffness, Vector2.new())
	self._fovSpring  = Spring.new(self.fovStiffness, 0)
	self._rollSpring = Spring.new(self.rollStiffness, 0)
end

------------------------------------------------------------------------
-- Core camera math step
--
-- vel:  Vector3  -- desired translation input (right, up, forward in local space)
-- pan:  Vector2  -- desired rotation input (pitch, yaw in radians)
-- fov:  number   -- desired FOV change input
-- roll: number   -- desired roll input (radians)
-- dt:   number   -- frame delta time
--
-- Returns: cameraCFrame, cameraFov
------------------------------------------------------------------------
function CustomCam:update(dt, vel, pan, fov, roll)
	-- Feed inputs through springs for smooth interpolation
	local smoothVel  = self._velSpring:Update(dt, vel or Vector3.new())
	local smoothPan  = self._panSpring:Update(dt, pan or Vector2.new())
	local smoothFov  = self._fovSpring:Update(dt, fov or 0)
	local smoothRoll = self._rollSpring:Update(dt, roll or 0)

	-- Update spring stiffness if changed externally
	self._velSpring:SetFreq(self.velStiffness)
	self._panSpring:SetFreq(self.panStiffness)
	self._fovSpring:SetFreq(self.fovStiffness)
	self._rollSpring:SetFreq(self.rollStiffness)

	-- FOV zoom factor: normalizes pan/roll speed relative to current FOV
	-- At 70° FOV this is 1.0; zooming in makes it larger (slowing pan proportionally)
	local zoomFactor = sqrt(tan(rad(70 / 2)) / tan(rad(self._cameraFov / 2)))

	-- Update FOV (clamped 1-120)
	self._cameraFov = clamp(
		self._cameraFov + smoothFov * self.fovGain * (dt / zoomFactor),
		1, 120
	)

	-- Update rotation (pitch, yaw, roll)
	local panVector = smoothPan * self.panGain * (dt / zoomFactor)
	self._cameraRot = self._cameraRot + Vector3.new(
		panVector.X,                                    -- pitch
		panVector.Y,                                    -- yaw
		smoothRoll * self.rollGain * (dt / zoomFactor)  -- roll
	)

	-- Clamp pitch, wrap yaw and roll to [0, 2pi)
	self._cameraRot = Vector3.new(
		clamp(self._cameraRot.x, -self.pitchLimit, self.pitchLimit),
		self._cameraRot.y % (2 * pi),
		self._cameraRot.z
	)

	-- Compose final CFrame: position * rotation * velocity offset
	-- Velocity offset is in local space (after rotation), so movement is relative to look direction
	local cameraCFrame = CFrame.new(self._cameraPos)
		* CFrame.fromOrientation(self._cameraRot.x, self._cameraRot.y, self._cameraRot.z)
		* CFrame.new(smoothVel * self.navGain * dt)

	-- Store new position for next frame
	self._cameraPos = cameraCFrame.Position

	return cameraCFrame, self._cameraFov
end

------------------------------------------------------------------------
-- Getters / Setters
------------------------------------------------------------------------
function CustomCam:getPosition()
	return self._cameraPos
end

function CustomCam:getRotation()
	return self._cameraRot
end

function CustomCam:getFov()
	return self._cameraFov
end

function CustomCam:setPosition(pos)
	self._cameraPos = pos
end

function CustomCam:setRotation(rot)
	self._cameraRot = rot
end

function CustomCam:setFov(fov)
	self._cameraFov = clamp(fov, 1, 120)
end

return CustomCam
