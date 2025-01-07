#pragma once

#include <iostream>
#include <string>

namespace snider {

/** A type of logger that will print a string message every so many seconds. */
class MessageTicker {
public:
    MessageTicker() = delete;
    /** A new ticker.  Instantiate one and call tick() to print a message,
        every period of @delaySeconds

        @param str The message to display
        @param delaySeconds How long between readouts.
        @param intervalMs The interval in milliseconds of the realtime calling 
                          function.
     */
    MessageTicker (std::string_view str, int delaySeconds = 2, int intervalMs = 20)
        : _message (str),
          // Using std::max (1, ...) because dividing by zero will result in a nasty crash
          _throttleTime (delaySeconds * 1000 / std::max (1, intervalMs)),
          _tick (_throttleTime) {
    }

    /** Call this inside the realtime function cycling at `intervalMs` per 
        second.
     */
    void tick() noexcept {
        // the tick counter
        if (--_tick <= 0) {
            if (enabled())
                std::clog << std::to_string (++_count) << ": " << _message << std::endl;
            _tick = _throttleTime;
        }
    }

    /** Enable or disable. 
        
        @param yn Set false to disable printing.
    */
    void enable (bool yn = true) noexcept {
        if (_enabled == yn)
            return;
        _enabled = yn;
    }

    /** Returns true if enabled and printing. */
    constexpr bool enabled() const noexcept { return _enabled; }

private:
    // message to print.
    std::string _message;
    // How many ticks it takes to equal the delay time in seconds based on the
    // millisecond interval.
    int _throttleTime = 0;
    // Total times the message has printed.
    int _count = 0;
    // The tick we're currently on.
    int _tick { 0 };
    // self explanatory.
    bool _enabled { true };
};

} // namespace snider
