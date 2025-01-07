---Access to bot Parameters.
---@class params
local M = {}

local impl = cxx.params

---Returns the speed ranged from  -1.0 to 1.0
function M.speed() return impl.speed() end

---Returns rotation ranged from -1.0 to 1.0
function M.rotation() return impl.rotation() end

---Returns the amount to brake by (0.0 to 1.0)
function M.brake() return impl.brake() end

return M
