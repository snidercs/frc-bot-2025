#pragma once

#include <string>
#include <string_view>

#include "sol/function.hpp"
#include "sol/state.hpp"

namespace lua {

/** Initialize and destroy Lua with RAII pattern. Instantiating this class more 
    than once will throw a runtime exception. Using lua::state() before the 
    Lifecycle is present will crash badly.
 */
struct Lifecycle final {
    Lifecycle();
    ~Lifecycle();

    // no copy
    Lifecycle (const Lifecycle&)            = delete;
    Lifecycle& operator= (const Lifecycle&) = delete;
    // no move
    Lifecycle (Lifecycle&&)            = delete;
    Lifecycle& operator= (Lifecycle&&) = delete;
};

/** Returns the global Lua context. */
sol::state& state();

/** Prints the Lua version to console. */
void print_version();

/** Set the Lua search path.
    Does nothing if passed an empty string.
    @param path A non empty Lua path string.
*/
void set_path (std::string_view path);

/** Returns the search directory for Lua. */
const std::string& search_directory();

/** Bootstrap the interpreter (call once before robot init)
    @returns true if Lua could be bootstrapped.
*/
bool bootstrap();

/** Lua to C++ config bindings.
    
    Functions in this namespace call in to Lua to retrieve values
    from the module.
 */
namespace config {

/** Get a value from the 'general' settings. See `robot/config.lua` 
    @param symbol The symbol key to lookup.
    @returns the value or an invalid object.
*/
sol::object get (std::string_view symbol);

/** Get a value from any category
    @param category The category to check
    @param symbol The value key to get.
    @returns The value or an invalid object.
 */
sol::object get (std::string_view category, std::string_view symbol);

} // namespace config

} // namespace lua
