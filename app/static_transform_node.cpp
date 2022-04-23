#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_transform");
  ros::NodeHandle nh;




  std::string config;
  if (!nh.getParam("config_file", config)) {
    std::cerr << "getParam config_file failed";
    return -1;
  }
  std::cout << "====: " << config << std::endl;

  YAML::Node node;
  try {
    node = YAML::LoadFile(config);
  } catch (const YAML::Exception &e) {
    std::cerr << "load yaml " << config << " failed, throw: " << e.what();
    return -1;
  }

  std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> all_broads;
  std::vector<geometry_msgs::TransformStamped> all_tfs;

  try {
    for (YAML::const_iterator it = node["extrinsic"].begin(); it != node["extrinsic"].end(); ++it) {
      std::string from, to;
      std::vector<double> rotation, translation;
      for (YAML::const_iterator itt = it->begin(); itt != it->end(); ++itt) {
        std::string key = itt->first.as<std::string>();
        std::cerr << "key: " << key << std::endl;
        if (key == "from") {
          from = itt->second.as<std::string>();
        } else if (key == "to") {
          to = itt->second.as<std::string>();
        } else if (key == "rotation") {
          rotation = itt->second.as<std::vector<double>>();
        } else if (key == "translation") {
          translation = itt->second.as<std::vector<double>>();
        }
      }

      if (from.empty() || to.empty() || rotation.size() != 3 || translation.size() != 3) {
        std::cerr << "invalid extrinsic" << std::endl;
        return -1;
      }

      auto tb = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
      all_broads.push_back(tb);
      geometry_msgs::TransformStamped tf;
      tf.header.stamp = ros::Time::now();
      tf.header.frame_id = from;
      tf.child_frame_id = to;
      tf.transform.translation.x = translation[0];
      tf.transform.translation.y = translation[1];
      tf.transform.translation.z = translation[2];
      tf2::Quaternion q;
      q.setRPY(rotation[0] * M_PI / 180,
               rotation[1] * M_PI / 180,
               rotation[2] * M_PI / 180);
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      all_tfs.push_back(tf);
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "parse yaml config failed, throw " << e.what();
    return -1;
  }

  for (size_t i = 0; i < all_tfs.size(); ++i) {
    all_broads[i]->sendTransform(all_tfs[i]);
  }

  ros::spin();

  return 0;
}