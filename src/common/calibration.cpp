#include "maloc/common/calibration.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace maloc {

int Calibration::Init(const std::string &config) {
    all_transforms_.clear();

    YAML::Node node;
    try {
        node = YAML::LoadFile(config);
    } catch (const YAML::Exception &e) {
        LOG(ERROR) << "Load yaml " << config << " failed, throw: " << e.what();
        return -1;
    }

    try {
        for (YAML::const_iterator it = node["extrinsic"].begin(); it != node["extrinsic"].end(); ++it) {
            std::string from, to;
            std::vector<double> euler_angles, translation;
            for (YAML::const_iterator itt = it->begin(); itt != it->end(); ++itt) {
                std::string key = itt->first.as<std::string>();
                if (key == "from") {
                    from = itt->second.as<std::string>();
                } else if (key == "to") {
                    to = itt->second.as<std::string>();
                } else if (key == "euler_angles") {
                    euler_angles = itt->second.as<std::vector<double>>();
                } else if (key == "translation") {
                    translation = itt->second.as<std::vector<double>>();
                }
            }

            if (from.empty() || to.empty() || euler_angles.size() != 3 || translation.size() != 3) {
                LOG(ERROR) << "Invalid extrinsic, please check";
                return -1;
            }

            if (all_transforms_.find(from) == all_transforms_.end()) {
                all_transforms_[from] = std::map<std::string, Transform>();
            }
            auto &tos = all_transforms_[from];

            Transform trans;
            trans.translation << translation[0], translation[1], translation[2];
            double roll = euler_angles[0] * M_PI / 180;
            double pitch = euler_angles[1] * M_PI / 180;
            double yaw = euler_angles[2] * M_PI / 180;
            trans.rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
            if (tos.find(to) != tos.end()) {
                LOG(WARNING) << "Extrinsic from `" << from << "` to `" << to << "` is already exist, now overwrite";
                LOG(WARNING) << "Overwrite extrinsic, from `" << from << "` to `" << to << "`";
            } else {
                LOG(INFO) << "Add extrinsic, from `" << from << "` to `" << to << "`";
            }
            tos[to] = trans;
        }
    } catch (const YAML::Exception &e) {
        LOG(ERROR) << "Parse yaml " << config << " failed, throw: " << e.what();
        return -1;
    }

    return 0;
}

int Calibration::GetTransform(const std::string &from, const std::string &to,
                              Eigen::Matrix3d &rotation, Eigen::Vector3d &translation) {
    if (all_transforms_.find(from) == all_transforms_.end()) {
        return -1;
    }
    const auto &tos = all_transforms_[from];
    if (tos.find(to) == tos.end()) {
        return -1;
    }
    rotation = tos.at(to).rotation;
    translation = tos.at(to).translation;

    return 0;
}

void Calibration::GetAllTransform(std::vector<TransformInfo> &all) {
    all.clear();
    for (auto every_from : all_transforms_) {
        for (auto every_to : every_from.second) {
            TransformInfo ti;
            ti.from = every_from.first;
            ti.to = every_to.first;
            ti.transform = every_to.second;
            all.emplace_back(ti);
        }
    }
}

int CalibInit(const std::string &config) {
    return Calibration::Instance().Init(config);
}

int CalibGetTransform(const std::string &from, const std::string &to,
                      Eigen::Matrix3d &rotation, Eigen::Vector3d &translation) {
    return Calibration::Instance().GetTransform(from, to, rotation, translation);
}

void CalibGetAllTransform(std::vector<TransformInfo> &all) {
    Calibration::Instance().GetAllTransform(all);
}

}// namespace maloc