#pragma once

#include <string>
#include <vector>

namespace config {

/** Return a number (double) by category and symbol. */
double number (std::string_view cat, std::string_view sym, double fallback = 0.0);

/** Return an integer by category and symbol. sol will throw an exception when
    a lua_Number doesn't fit inside a 32bit int.  This helper does a "safe" 
    conversion.
*/
int integer (std::string_view cat, std::string_view sym, int fallback = 0);

/** Returns the default gamepad skew factor. */
double gamepad_skew_factor();

/** Returns the total number of port indexes. */
int num_ports();

/** Returns a port index by symbol. */
int port (std::string_view symbol);

/** Returns the team name. */
std::string team_name();

/** Returns the team number. */
int team_number();

/** Match start position. */
std::string match_start_position();

/** Trajectory names */
std::vector<std::string> trajectory_names();

} // namespace config
