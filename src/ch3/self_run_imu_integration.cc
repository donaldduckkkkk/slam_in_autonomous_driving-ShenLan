#ifndef SELF_RUN_IMU_INTEGRATION_CC
#define SELF_RUN_IMU_INTEGRATION_CC

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
auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v, const Vec3d& p) {
    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
    save_vec3(fout, p);
    save_quat(fout, R.unit_quaternion());
    save_vec3(fout, v);
    fout << std::endl;
};
class imu_integration {
   public:
    imu_integration(Eigen::Vector3d gravity, Eigen::Vector3d init_bg, Eigen::Vector3d init_ba) {
        R_ = Sophus::SO3d::exp(Eigen::Vector3d::Zero());
        V_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Vector3d::Zero();
        gravity_ = gravity;
        init_bg_ = init_bg;
        init_ba_ = init_ba;
    };
    int add_imu(double timestamp, Eigen::Vector3d acc, Eigen::Vector3d gyro);

    int get_state(Sophus::SO3d& R, Eigen::Vector3d& V, Eigen::Vector3d& P);
    double last_timestamp_;

    //    private:
    Sophus::SO3d R_;
    Eigen::Vector3d V_;
    Eigen::Vector3d P_;
    Eigen::Vector3d gravity_;
    Eigen::Vector3d init_bg_;
    Eigen::Vector3d init_ba_;
};

int imu_integration::add_imu(double timestamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
    if (last_timestamp_ == 0) {
        last_timestamp_ = timestamp;
        return 1;
    }
    double dt = timestamp - last_timestamp_;
    last_timestamp_ = timestamp;
    if (dt > 0 && dt < 0.1) {
        P_ = P_ + V_ * dt + R_ * (acc - init_ba_) * dt * dt * 0.5 + 0.5 * gravity_ * dt * dt;
        V_ = V_ + R_ * (acc - init_ba_) * dt + gravity_ * dt;
        R_ = R_ * Sophus::SO3d::exp((gyro - init_bg_) * dt);
    }
    return 1;
};
// 一定要写成取地址
int imu_integration::get_state(Sophus::SO3d& R, Eigen::Vector3d& V, Eigen::Vector3d& P) {
    R = R_;
    V = V_;
    P = P_;
    return 1;
};

class gnss_process {
   public:
    gnss_process(double angle, double p_x, double p_y) {
        an_angle_ = angle;
        an_p_x_ = p_x;
        an_p_y_ = p_y;
    };
    int add_gnss(double timestamp, Eigen::Vector3d pos, double heading, bool heading_valid);
    int get_state(Sophus::SO3d& R, Eigen::Vector3d& P) {
        R = R_;
        P = P_;
        return 1;
    };

   private:
    double an_angle_;
    double an_p_x_;
    double an_p_y_;
    double last_timestamp_;
    double lat;
    double lon;
    double alt;
    Sophus::SO3d R_;
    Eigen::Vector3d P_;
};

int gnss_process::add_gnss(double timestamp, Eigen::Vector3d pos, double heading, bool heading_valid) {
    if (heading_valid) {
        an_angle_ = heading;
    }

    double dt = timestamp - last_timestamp_;
    last_timestamp_ = timestamp;
    lat = pos[0];
    lon = pos[1];
    alt = pos[2];
    // todo 转换 rt
    // Tbg
    Sophus::SO3d R_z = Sophus::SO3d::rotZ(an_angle_ * M_PI / 180);
    Eigen::Vector3d Tbg(an_p_x_, an_p_y_, 0);
    Sophus::SE3d Tbg_se3(R_z, Tbg);
    sad::UTMCoordinate utm_coor;
    if (!sad::LatLon2UTM(Eigen::Vector2d(lat, lon), utm_coor)) return 0;
    utm_coor.z_ = alt;
    if (heading_valid) {
        LOG(INFO) << "heading_valid";
        heading = 90 - heading;
    } else {
        LOG(INFO) << "heading_invalid";
        heading = 0;
    }

    Sophus::SO3d R_z2 = Sophus::SO3d::rotZ((heading)*M_PI / 180);
    Eigen::Vector3d Twg(utm_coor.xy_[0], utm_coor.xy_[1], utm_coor.z_);
    Sophus::SE3d Twg_se3(R_z2, Twg);

    Sophus::SE3d Twb = Twg_se3 * Tbg_se3.inverse();
    R_ = Twb.so3();
    if (heading_valid) {
        R_ = Sophus::SO3d::exp(Eigen::Vector3d::Zero());
    }
    P_ = Twb.translation();
    LOG(INFO) << "P:" << P_.transpose();
    sad::GNSS gnss_data(timestamp, 4, Eigen::Vector3d(lat, lon, alt), heading, heading_valid);
    sad::ConvertGps2UTM(gnss_data, Eigen::Vector2d(an_p_x_, an_p_y_), an_angle_, Eigen::Vector3d::Zero());
    LOG(INFO) << "utm:" << gnss_data.utm_.xy_[0] << " " << gnss_data.utm_.xy_[1] << " " << gnss_data.utm_.z_;
    return 1;
};
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);
    sad::ui::PangolinWindow ui;
    ui.Init();

    std::ifstream fin(FLAGS_imu_data);
    std::ofstream out_file("/home/nio/文档/slam_in_autonomous_driving/data/ch3/state1.txt");

    Eigen::Vector3d gravity(0, 0, -9.8);
    Eigen::Vector3d init_bg(0.000224886, -7.61038e-05, -0.000742259);
    Eigen::Vector3d init_ba(-0.165205, 0.0926887, 0.0058049);

    imu_integration imu(gravity, init_bg, init_ba);
    gnss_process gnss(FLAGS_antenna_angle, FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);
    Eigen::Vector3d pos_0;
    bool gnss_set = false;
    while (ui.ShouldQuit() == false) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            break;
        }
        std::stringstream ss(line);
        std::string data_type;
        ss >> data_type;
        if (data_type == "IMU") {
            continue;
            double timestamp;
            ss >> timestamp;
            Eigen::Vector3d acc, gyro;
            ss >> gyro[0] >> gyro[1] >> gyro[2] >> acc[0] >> acc[1] >> acc[2];
            imu.add_imu(timestamp, acc, gyro);
            Sophus::SO3d R;
            Eigen::Vector3d V, P;
            imu.get_state(R, V, P);
            LOG(INFO) << "R:" << R.matrix();
            LOG(INFO) << "V:" << V;
            LOG(INFO) << "P:" << P;
            save_result(out_file, imu.last_timestamp_, imu.R_, imu.V_, imu.P_);
            ui.UpdateNavState(sad::NavState(timestamp, R, P, V, init_bg, init_ba));
            sleep(1e-2);
        } else if (data_type == "GNSS") {
            double timestamp;
            ss >> timestamp;
            Eigen::Vector3d pos;
            ss >> pos[0] >> pos[1] >> pos[2];
            double heading;
            ss >> heading;
            bool heading_valid;
            ss >> heading_valid;
            if (!gnss.add_gnss(timestamp, pos, heading, heading_valid)) continue;
            Sophus ::SO3d R;
            Eigen::Vector3d P;
            gnss.get_state(R, P);
            if (!gnss_set) {
                pos_0 = P;
                gnss_set = true;
            }
            P = P - pos_0;
            ui.UpdateNavState(sad::NavState(timestamp, R, P));
            usleep(1e4);
        }
    }
    ui.Quit();
    return 0;
}

#endif  // SELF_RUN_IMU_INTEGRATION_CC