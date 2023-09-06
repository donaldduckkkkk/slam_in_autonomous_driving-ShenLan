#ifndef SELF_RUN_ESKF_GINS_CC
#define SELF_RUN_ESKF_GINS_CC

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sophus/se3.hpp>
#include "common/gnss.h"
#include "common/nav_state.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_string(imu_data, "/home/nio/文档/slam_in_autonomous_driving/data/ch3/10.txt", "imu信息文件");

class GNSS {
   public:
    GNSS(double antenna_angle, double antenna_pox_x, double antenna_pox_y) {
        antenna_angle_ = antenna_angle;
        antenna_pox_x_ = antenna_pox_x;
        antenna_pox_y_ = antenna_pox_y;
    };
    void addgnss(double timestamp, Eigen::Vector3d pos, double heading, bool heading_valid){
        // TODO ADD
    };
    void get_state(Eigen::Vector3d& pos, double& heading) {
        pos = pos_;
        heading = heading_;
    };

   private:
    double antenna_angle_;
    double antenna_pox_x_;
    double antenna_pox_y_;
    double last_timestamp_;
    double timestamp_;
    Eigen::Vector3d pos_;
    double heading_;
    bool heading_valid_;
};
// 为了初始化
class IMU {
   public:
    IMU(Eigen::Vector3d gravity, Eigen::Vector3d init_bg, Eigen::Vector3d init_ba) {
        R_ = Sophus::SO3d::exp(Eigen::Vector3d::Zero());
        V_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Vector3d::Zero();
        gravity_ = gravity;
        init_bg_ = init_bg;
        init_ba_ = init_ba;
    };
    void addimu(double timestamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
        // TODO ADD
        imu_vector[timestamp] = std::make_pair(acc, gyro);
    };
    void get_state(Sophus::SO3d& R, Eigen::Vector3d& V, Eigen::Vector3d& P) {
        R = R_;
        V = V_;
        P = P_;
    }

   private:
    Sophus::SO3d R_;
    Eigen::Vector3d V_;
    Eigen::Vector3d P_;
    Eigen::Vector3d gravity_;
    Eigen::Vector3d init_bg_;
    Eigen::Vector3d init_ba_;
    double last_timestamp_;
    bool init_flag_ = false;
    std::map<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>> imu_vector;
};
class ODOM {
   public:
   private:
};
class ESKF {
   public:
    ESKF(){};
    void predict() {}
    void update();

    //    private:
    IMU imu;
    GNSS gnss;
    ODOM odom;
};
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_imu_data.empty()) {
        return -1;
    }

    std::ifstream fin(fLS::FLAGS_imu_data);
    std::ofstream fout("./data/ch3/gins.txt");
    Eigen::Vector3d gravity(0, 0, -9.8);
    Eigen::Vector3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Eigen::Vector3d init_ba(-0.165205, 0.0926887, 0.0058049);
    // 噪音
    Eigen::Vector3d gyro_noise(0.0001, 0.0001, 0.0001);
    Eigen::Vector3d acc_noise(0.001, 0.001, 0.001);

    sad::ui::PangolinWindow ui;
    ui.Init();

    ESKF eskf;
    eskf.imu = IMU(gravity, init_bg, init_ba);
    eskf.gnss = GNSS(FLAGS_antenna_angle, FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);
    while (ui.ShouldQuit()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            break;
        }
        std::stringstream ss(line);
        std::string type;
        ss >> type;
        if (type == "IMU") {
            double timestamp;
            ss >> timestamp;
            Eigen::Vector3d acc, gyro;
            ss >> gyro[0] >> gyro[1] >> gyro[2] >> acc[0] >> acc[1] >> acc[2];
            eskf.imu.addimu(timestamp, acc, gyro);
        } else if (type == "GNSS") {
            double timestamp;
            ss >> timestamp;
            Eigen::Vector3d pos;
            ss >> pos[0] >> pos[1] >> pos[2];
            double heading;
            ss >> heading;
            bool heading_valid;
            ss >> heading_valid;
            eskf.gnss.addgnss(timestamp, pos, heading, heading_valid);
        } else if (type == "ODOM") {
            continue;
        }
    }
    ui.Quit();
    return 0;
}

#endif  // SELF_RUN_ESKF_GINS_CC