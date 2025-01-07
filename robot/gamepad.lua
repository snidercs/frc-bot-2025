--- The Gamepad.
--  Provides direct access to the gamepad button, trigger, and dpad values.

--[[
    ### Possible button/stick indexes
    - STICK_LEFT_X
    - STICK_LEFT_Y
    - STICK_RIGHT_X
    - STICK_RIGHT_Y
    - TRIGGER_LEFT
    - TRIGGER_RIGHT

    - BUTTON_A
    - BUTTON_B
    - BUTTON_X
    - BUTTON_Y
    - BUMPER_LEFT
    - BUMPER_RIGHT
--]]

local M = {}

-- cache some c++ functions
local impl = cxx.gamepad
local raw_axis = impl.raw_axis
local raw_button = impl.raw_button
local raw_button_pressed = impl.raw_button_pressed
local raw_button_released = impl.raw_button_released

-- Copy constants from c++ bindings
for k, v in pairs(impl) do
    if type(v) == "number" then
        M[k] = v
    end
end

--[[
-- Uncomment when using regular keyboard as a gamepad in the sim.
M.BUMPER_LEFT = 2
M.BUMPER_RIGHT = 3
--]]

---Return an axis value by index.
---@param index integer Index of the axis control
---@return number
function M.axis(index)
    return raw_axis(index)
end


---Get the angle in degrees of a POV on the HID.
---The POV angles start at 0 in the up direction, and increase clockwise
---(e.g. right is 90, upper-left is 315).
---@return integer angle of the POV in degrees, or -1 if the POV is not pressed.
function M.pov()
    return impl.pov()
end

---Returns true if button 'B' is currently down.
---@return boolean
function M.A()
    return raw_button(M.BUTTON_A)
end

---Returns true if button 'B' is currently down.
---@return boolean
function M.B()
    return raw_button(M.BUTTON_B)
end

---Returns true if button 'X' is currently down.
---@return boolean
function M.X()
    return raw_button(M.BUTTON_X)
end

---Returns true if button 'Y' is currently down.
---@return boolean
function M.Y()
    return raw_button(M.BUTTON_Y)
end

---Get the left trigger value (0.0 to 1.0)
---@return number
function M.left_trigger()
    return raw_axis(M.TRIGGER_LEFT)
end

---Get the right trigger value (0.0 to 1.0)
---@return number
function M.right_trigger()
    return raw_axis(M.TRIGGER_RIGHT)
end

---Returns true if the left bumper was pressed since the last check.
---@return boolean
function M.left_bumper_pressed()
    return raw_button_pressed(M.BUMPER_LEFT)
end

function M.left_bumper_released()
    return raw_button_released(M.BUMPER_LEFT)
end

---Returns true if the left bumper is currently held down.
---@return boolean
function M.left_bumper()
    return raw_button(M.BUMPER_LEFT)
end

---Returns true if the right bumper was pressed since the last check.
---@return boolean
function M.right_bumper_pressed()
    return raw_button_pressed(M.BUMPER_RIGHT)
end

---Returns true if the right bumper was pressed since the last check.
---@return boolean
function M.right_bumper_released()
    return raw_button_released(M.BUMPER_RIGHT)
end

---Returns true if the right bumper is currently held down.
---@return boolean
function M.right_bumper()
    return raw_button(M.BUMPER_RIGHT)
end

return M
