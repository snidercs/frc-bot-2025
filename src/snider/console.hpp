#pragma once

#include <iostream>
#include <string>
#if __has_include(<boost/algorithm/string/trim.hpp>)
#    include <boost/algorithm/string/trim.hpp>
#endif

namespace snider {

/** Logging helpers. */
namespace console {

/** Log a message or messages to the console. This version can only take string
    type arguments at the moment.  So int and float types should be converted
    before passing in here with `std::to_string` or some other way.

    Not passing string types only will result in insane template errors.
 */
template <typename... Str>
inline static void log (Str&&... msgs) {
    std::string out;
    // loop through messages and var args and store in a string buffer.
    ([&, &out = out] {
        out += msgs + std::string ("   ");
    }(),
     ...);

#if __has_include(<boost/algorithm/string/trim.hpp>)
    // trim the last tab
    boost::trim (out);
#endif

    // render to console
    std::clog << out << std::endl;
}

} // namespace console
} // namespace snider
