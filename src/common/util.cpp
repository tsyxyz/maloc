#include "maloc/common/util.h"
#include <boost/filesystem.hpp>
#include <glog/logging.h>


namespace maloc {

int InitGlog(const char *argv0, const char *log_dir,
             int stderr_log_level, int min_log_level) {
    boost::filesystem::path log_path(log_dir);
    if (!boost::filesystem::exists(log_dir) &&
        !boost::filesystem::create_directory(log_dir)) {
        return -1;
    }

    google::InitGoogleLogging(argv0);
    FLAGS_log_dir = log_dir;
    FLAGS_stderrthreshold = stderr_log_level;
    FLAGS_minloglevel = min_log_level;
    return 0;
}


}// namespace maloc