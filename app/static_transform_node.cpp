#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "maloc/common/calibration.h"
#include "maloc/common/util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "static_transform");
    ros::NodeHandle nh;

    std::string log_dir = "./log";
    if (!nh.getParam("log_dir", log_dir)) {
        LOG(WARNING) << "getParam log_dir failed, using default " << log_dir;
    }
    int stderr_log_level = 0;
    if (!nh.getParam("stderr_log_level", stderr_log_level)) {
        LOG(WARNING) << "getParam stderr_log_level failed, using default " << stderr_log_level;
    }
    int min_log_level = 0;
    if (!nh.getParam("min_log_level", min_log_level)) {
        LOG(WARNING) << "getParam min_log_level failed, using default " << min_log_level;
    }

    std::string calibration;
    if (!nh.getParam("calibration_file", calibration)) {
        LOG(ERROR) << "getParam calibration_file failed";
        return -1;
    }

    maloc::InitGlog(argv[0], log_dir.c_str(), stderr_log_level, min_log_level);

    if (maloc::CalibInit(calibration) < 0) {
        LOG(ERROR) << "calibration init failed";
        return -1;
    }

    std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> all_broads;
    std::vector<geometry_msgs::TransformStamped> all_tfs;

    std::vector<maloc::TransformInfo> all_tfi;
    maloc::CalibGetAllTransform(all_tfi);
    for (auto tfi : all_tfi) {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = tfi.from;
        tf.child_frame_id = tfi.to;
        tf.transform.translation.x = tfi.transform.translation[0];
        tf.transform.translation.y = tfi.transform.translation[1];
        tf.transform.translation.z = tfi.transform.translation[2];
        Eigen::Quaterniond q(tfi.transform.rotation);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        all_tfs.push_back(tf);

        auto tb = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
        all_broads.push_back(tb);
    }

    for (size_t i = 0; i < all_tfs.size(); ++i) {
        all_broads[i]->sendTransform(all_tfs[i]);
    }

    ros::spin();

    return 0;
}