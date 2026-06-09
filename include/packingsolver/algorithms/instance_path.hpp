#pragma once

#include <boost/filesystem.hpp>

#include <string>
#include <stdexcept>

namespace packingsolver
{

/**
 * Resolve where an instance's CSV files live.
 *
 * The CLIs accept either an exact path to an items file, or a prefix /
 * directory from which "<prefix>items.csv" (and the sibling bins.csv,
 * defects.csv, parameters.csv) are derived. This returns the prefix to prepend
 * to "items.csv" etc.; an empty string means @p items_arg is itself the exact
 * items file. Throws std::invalid_argument naming the variants tried when no
 * candidate file exists, so a mistyped path fails with a readable message
 * instead of an opaque crash.
 */
inline std::string resolve_instance_path(const std::string& items_arg)
{
    namespace fs = boost::filesystem;
    if (fs::is_regular_file(items_arg))
        return "";
    if (fs::is_regular_file(items_arg + "_items.csv"))
        return items_arg + "_";
    if (fs::is_regular_file(items_arg + "items.csv"))
        return items_arg;
    if (fs::is_regular_file(items_arg + "/items.csv"))
        return items_arg + "/";
    throw std::invalid_argument(
            "resolve_instance_path: "
            "unable to find an items file for \"" + items_arg + "\"; "
            "expected a file at that exact path, or one of "
            "\"" + items_arg + "items.csv\", "
            "\"" + items_arg + "_items.csv\", "
            "\"" + items_arg + "/items.csv\".");
}

}
