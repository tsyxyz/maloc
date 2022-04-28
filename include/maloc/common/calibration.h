#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace maloc {

struct Transform {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
};

struct TransformInfo {
    std::string from;
    std::string to;
    Transform transform;
};

class Calibration {
public:
    int Init(const std::string &config);
    int GetTransform(const std::string &from, const std::string &to,
                     Eigen::Matrix3d &rotation, Eigen::Vector3d &translation);
    void GetAllTransform(std::vector<TransformInfo> &all);

public:
    static Calibration &Instance() {
        static Calibration instance;
        return instance;
    }

    Calibration(const Calibration &) = delete;
    Calibration &operator=(const Calibration &) = delete;

private:
    Calibration() = default;
    ~Calibration() = default;

    std::map<std::string, std::map<std::string, Transform>> all_transforms_;
};

int CalibInit(const std::string &config);
int CalibGetTransform(const std::string &from, const std::string &to,
                      Eigen::Matrix3d &rotation, Eigen::Vector3d &translation);
void CalibGetAllTransform(std::vector<TransformInfo> &all);

}// namespace maloc