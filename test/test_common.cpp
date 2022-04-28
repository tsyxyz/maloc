#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>

#include <maloc/common/calibration.h>

using namespace maloc;

class TestCalibration : public testing::Test {
protected:
    const std::string kTestYamlPath = "/tmp/test_calibration.yaml";

    virtual void SetUp() {
        const std::string yaml_content = R"(
extrinsic:
  - from: "vehicle"
    to: "left_velodyne"
    euler_angles: [1.42947, 44.5011, 136.385]
    translation: [-0.334623, 0.431973, 1.94043]

  - from: "vehicle"
    to: "right_velodyne"
    euler_angles: [178.899, 135.416, 43.7457]
    translation: [-0.333596, -0.373928, 1.94377]

  - from: "vehicle"
    to: "imu"
    euler_angles: [0, 0, 0]
    translation: [-0.07, 0, 1.7]

  - from: "vehicle"
    to: "right_velodyne"
    euler_angles: [0, 0, 0]
    translation: [-0.32, 0, 1.7]

  - from: "vrs"
    to: "vehicle"
    euler_angles: [0, 0, 0]
    translation: [0, 0, 1.93]
)";
        std::ofstream ofs(kTestYamlPath);
        ofs << yaml_content;
        ofs.close();
    }

    virtual void TearDown() {
        std::remove(kTestYamlPath.c_str());
    }
};

TEST_F(TestCalibration, GetTransform) {
    Calibration::Instance().Init(kTestYamlPath);

    Eigen::Matrix3d veh_to_lv_r;
    Eigen::Vector3d veh_to_lv_t;
    int ret = Calibration::Instance().GetTransform("vehicle", "invalid",
                                                   veh_to_lv_r, veh_to_lv_t);
    EXPECT_NE(ret, 0);

    ret = Calibration::Instance().GetTransform("vehicle", "left_velodyne",
                                               veh_to_lv_r, veh_to_lv_t);
    EXPECT_EQ(ret, 0);

    std::cout << "vehicle to left velodyne:\n"
              << "R:\n"
              << veh_to_lv_r << "\n"
              << "t:\n"
              << veh_to_lv_t.transpose() << std::endl;

    // ground truth
    Eigen::Matrix3d veh_to_lv_r_gt;
    Eigen::Vector3d veh_to_lv_t_gt;
    veh_to_lv_r_gt << -0.516377, -0.702254, -0.490096,
            0.491997, -0.711704, 0.501414,
            -0.700923, 0.0177927, 0.713015;
    veh_to_lv_t_gt << -0.334623, 0.431973, 1.94043;

    auto check_r = (veh_to_lv_r - veh_to_lv_r_gt).cwiseAbs();
    auto check_t = (veh_to_lv_t - veh_to_lv_t_gt).cwiseAbs();
    for (auto r = 0; r < check_r.rows(); ++r) {
        for (auto c = 0; c < check_r.cols(); ++c) {
            EXPECT_LT(check_r(r, c), 1e-6);
        }
    }
    for (auto i = 0; i < check_t.size(); ++i) {
        EXPECT_LT(check_t(i), 1e-6);
    }
}

TEST_F(TestCalibration, GetAllTransform) {
    Calibration::Instance().Init(kTestYamlPath);
    std::vector<TransformInfo> all;
    Calibration::Instance().GetAllTransform(all);

    EXPECT_EQ(all.size(), 4);
    for (auto t : all) {
        std::cout << "from " << t.from << " to " << t.to << "\n"
                  << "R:\n"
                  << t.transform.rotation << "\n"
                  << "t:\n"
                  << t.transform.translation.transpose() << "\n"
                  << "---" << std::endl;
    }
}