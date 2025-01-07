---Main robot.
---Facilities for controlling the bot.
---@class robot

local drivetrain = cxx.drivetrain;

---Drive the robot.
---@param xSpeed number Speed in the x direction ranged -1 to 1
---@param ySpeed number Speed in the y direction ranged -1 to 1
---@param rot number Rotation ranged -1 to 1
local function drive(xSpeed, ySpeed, rot)
    drivetrain.driveNormalized(xSpeed, ySpeed, rot)
end

local M = {
    drive = drive,
}

return M
