#pragma once

#include <filesystem>
#include <string>

#include "scripting.hpp"
#include "sol/sol.hpp"

/** Instantiate and run a Lua 'bot' file */
class Engine final {
public:
    ~Engine() {}

    /** Create a new instance from a file
        @param state The lua state to use
        @param bot_file The file to load from
    */
    static std::unique_ptr<Engine> instantiate (lua_State* state, std::string_view bot_file) {
        auto self = std::unique_ptr<Engine> (new Engine (state));

        if (! std::filesystem::exists (bot_file)) {
            self->_error = "file does not exist: ";
            self->_error += bot_file;
            return self;
        }

        sol::state_view L (state);
        Path path (bot_file);
        path.make_preferred();

        try {
            LoadResult res { L.load_file (path.string()) };
            if (! res.valid()) {
                sol::error err = res;
                self->_error   = err.what();
                return self;
            }

            {
                sol::object obj = res;
                if (obj.get_type() != sol::type::function) {
                    self->_error = "Did not get a factory function";
                    return self;
                }
            }

            Factory factory                   = res;
            sol::protected_function_result pr = factory();
            if (! pr.valid() || pr.get_type() != sol::type::table) {
                self->_error = "Did not get an engine descriptor table";
                return self;
            }

            self->M = pr;

#define ASSIGN_F(name, var)                                 \
    if (self->M[#name].get_type() == sol::type::function) { \
        self->var = self->M[#name];                         \
    }
            ASSIGN_F (init, pf_init)
            ASSIGN_F (prepare, pf_prepare)
            ASSIGN_F (run, f_run)
            ASSIGN_F (run, pf_safe_run)
            ASSIGN_F (cleanup, pf_cleanup)
#undef ASSIGN_PF

        } catch (const std::exception& e) {
            self->_error = e.what();
        }

        return self;
    }

    /** Returns an error string if present. */
    const std::string error() const noexcept { return _error; }

    /** Returns true if an error is present. */
    bool have_error() const noexcept { return ! _error.empty(); }

    /** Run the engine without error handling. */
    void run() {
        if (f_run)
            f_run();
    }

#define PMETHOD(name)                                         \
    bool name() {                                             \
        _error.clear();                                       \
        if (! pf_##name)                                      \
            return true;                                      \
        try {                                                 \
            sol::protected_function_result res = pf_##name(); \
            if (! res.valid()) {                              \
                sol::error err = res;                         \
                _error         = err.what();                  \
            }                                                 \
        } catch (const std::exception& e) {                   \
            _error = e.what();                                \
        }                                                     \
                                                              \
        return ! have_error();                                \
    }

    /** Initialize the engine.  Returns false on error. */
    PMETHOD (init)

    /** Prepare to enter realtime run callbacks. Returns false on error. */
    PMETHOD (prepare)

    /** Safely run the engine.  Will return false on error, in which case calling
     * code should stop running.
    */
    PMETHOD (safe_run)

    /** Clean up the engine. e.g. free resources.  Returns false on error. */
    PMETHOD (cleanup)
#undef PMETHOD

private:
    Engine (lua_State* state)
        : L (state) {
    }
    using Factory    = sol::protected_function;
    using LoadResult = sol::load_result;
    using Path       = std::filesystem::path;
    sol::state_view L;
    sol::table M;
    std::string _error;
    sol::function f_run;
    sol::protected_function pf_init, pf_prepare, pf_safe_run, pf_cleanup;
};

using EnginePtr = std::unique_ptr<Engine>;
