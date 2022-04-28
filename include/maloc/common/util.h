#pragma once

#include <string>

namespace maloc {

int InitGlog(const char *argv0, const char *log_dir = "./log",
             int stderr_log_level = 0, int min_log_level = 0);

}// namespace maloc