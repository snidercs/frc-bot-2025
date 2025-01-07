#include "config.hpp"
#include "scripting.hpp"

namespace config {

double number (std::string_view cat, std::string_view sym, double fallback) {
    if (cat.empty() || sym.empty())
        return fallback;

    auto obj = lua::state()["config"][cat];
    if (obj.is<sol::table>()) {
        sol::table tbl = obj;
        return tbl.get_or (sym, fallback);
    }

    return fallback;
}

int integer (std::string_view cat, std::string_view sym, int fallback) {
    return static_cast<int> (number (cat, sym, (double) fallback));
}

double gamepad_skew_factor() {
    sol::table tbl = lua::state()["config"]["gamepad"];
    return tbl.get_or ("skew_factor", 1.0);
}

std::string team_name() {
    sol::table general = lua::state()["config"]["general"];
    return general.get_or ("team_name", std::string());
}

int team_number() {
    sol::table general = lua::state()["config"]["general"];
    return general.get_or ("team_number", 0);
}

/** Match start position. */
std::string match_start_position() {
    sol::table general = lua::state()["config"]["general"];
    return general.get_or ("match_start_position", std::string { "Left" });
}

int num_ports() {
    auto& ls = lua::state();
    return (int) ls["config"]["num_ports"]();
}

int port (std::string_view symbol) {
    auto& ls = lua::state();
    return symbol.empty() ? -1 : (int) ls["config"]["port"](symbol);
}

std::vector<std::string> trajectory_names() {
    std::vector<std::string> out;
    const sol::table trajectories = lua::state()["config"]["trajectories"];

    for (const auto& i : trajectories)
        if (i.first.is<std::string>())
            out.push_back (i.first.as<std::string>());

    return out;
}

} // namespace config
