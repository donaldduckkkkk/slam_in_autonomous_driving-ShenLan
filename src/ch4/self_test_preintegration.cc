#ifndef SELF_TEST_PREINTEGRATION_CC
#define SELF_TEST_PREINTEGRATION_CC

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <sophus/se3.hpp>
#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "ch4/g2o_types.h"
#include "ch4/imu_preintegration.h"
#include "common/g2o_types.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/io_utils.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "tools/ui/pangolin_window.h"
DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
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
        return 0;
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
    void init(double angle, double p_x, double p_y) {
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

    //    private:
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
class imu_preintegration {
   public:
    void integration(double time, Eigen::Vector3d g, Eigen::Vector3d a, double dt) {
        time_ = time;
        acc = a - init_ba_;
        gyro = g - init_bg_;
        dt = dt - last_dt;
        dP = dP + dV * dt + dR * (acc - init_ba_) * dt * dt * 0.5;
        dV = dV + dR * (acc - init_ba_) * dt;
        Eigen::Matrix<double, 9, 9> A;
        A.setIdentity();
        Eigen::Matrix<double, 9, 6> B;
        B.setZero();

        Mat3d acc_hat = SO3::hat(acc);
        double dt2 = dt * dt;

        // NOTE A, B左上角块与公式稍有不同
        A.block<3, 3>(3, 0) = -dR.matrix() * dt * acc_hat;
        A.block<3, 3>(6, 0) = -0.5f * dR.matrix() * acc_hat * dt2;
        A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

        B.block<3, 3>(3, 3) = dR.matrix() * dt;
        B.block<3, 3>(6, 3) = 0.5f * dR.matrix() * dt2;
        // 更新各雅可比，见式(4.39)
        dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR.matrix() * dt2;                      // (4.39d)
        dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dR.matrix() * dt2 * acc_hat * dR_dbg_;  // (4.39e)
        dV_dba_ = dV_dba_ - dR.matrix() * dt;                                             // (4.39b)
        dV_dbg_ = dV_dbg_ - dR.matrix() * dt * acc_hat * dR_dbg_;                         // (4.39c)

        dR = dR * Sophus::SO3d::exp((gyro)*dt);
        // 更新噪声项
        cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();
        Mat3d rightJ = SO3::jr((gyro)*dt);                              // 右雅可比
                                                                        // 更新dR_dbg
        Sophus::SO3 deltaR = SO3::exp((gyro)*dt);                       // exp后
        dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;  // (4.39a)

        // 增量积分时间
        dt_ += dt;
    };
    void init(Eigen::Vector3d gravity, Eigen::Vector3d init_bg, Eigen::Vector3d init_ba) {
        gravity_ = gravity;
        init_bg_ = init_bg;
        init_ba_ = init_ba;
    };
    double time_;
    Mat6d noise_gyro_acce_ = Mat6d::Zero();  // 测量噪声矩阵
    double dt_ = 0;                          // 整体预积分时间
    Mat9d cov_ = Mat9d::Zero();              // 累计噪声矩阵
    double last_dt;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d dP = Eigen::Vector3d::Zero();
    Eigen::Vector3d dV = Eigen::Vector3d::Zero();
    Sophus::SO3d dR = Sophus::SO3d::exp(Eigen::Vector3d::Zero());

    Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d noise_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d noise_acce = Eigen::Vector3d::Zero();
    Mat3d dR_dbg_ = Mat3d::Zero();
    Mat3d dV_dbg_ = Mat3d::Zero();
    Mat3d dV_dba_ = Mat3d::Zero();
    Mat3d dP_dbg_ = Mat3d::Zero();
    Mat3d dP_dba_ = Mat3d::Zero();

   private:
};
class optimization {
   public:
    struct optimization_data {
        double timestamp;
        Eigen::Vector3d P;
        Eigen::Vector3d V;
        Sophus::SO3d R;
        Eigen::Vector3d bg;
        Eigen::Vector3d ba;
    };
    void set_last_data(double timestamp, Eigen::Vector3d P, Eigen::Vector3d V, Sophus::SO3d R, Eigen::Vector3d bg,
                       Eigen::Vector3d ba) {
        last_data_.timestamp = timestamp;
        last_data_.P = P;
        last_data_.V = V;
        last_data_.R = R;
        last_data_.bg = bg;
        last_data_.ba = ba;
    };
    void set_current_data(double timestamp, Eigen::Vector3d P, Eigen::Vector3d V, Sophus::SO3d R, Eigen::Vector3d bg,
                          Eigen::Vector3d ba) {
        current_data_.timestamp = timestamp;
        current_data_.P = P;
        current_data_.V = V;
        current_data_.R = R;
        current_data_.bg = bg;
        current_data_.ba = ba;
    };
    void run(sad::GNSS current_gnss);
    optimization_data last_data_;
    optimization_data current_data_;
    sad::GNSS last_gnss_;
};
void optimization::run(sad::GNSS current_gnss) {
    // TODO 优化
    // optimization_data last_data_;
    // optimization_data current_data_;
    // sad::GNSS last_gnss_;
    // sad::GNSS current_gnss
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto V0_pose = new sad::VertexPose();
    V0_pose->setEstimate(Sophus::SE3d(last_data_.R, last_data_.P));
    V0_pose->setId(0);
    optimizer.addVertex(V0_pose);

    auto V0_vel = new sad::VertexVelocity();
    V0_vel->setEstimate(last_data_.V);
    V0_vel->setId(1);
    optimizer.addVertex(V0_vel);

    auto v0_bg = new sad::VertexGyroBias();
    v0_bg->setId(2);
    v0_bg->setEstimate(last_data_.bg);
    optimizer.addVertex(v0_bg);

    auto v0_ba = new sad::VertexAccBias();
    v0_ba->setId(3);
    v0_ba->setEstimate(last_data_.ba);
    optimizer.addVertex(v0_ba);

    auto V1_pose = new sad::VertexPose();
    V1_pose->setEstimate(Sophus::SE3d(current_data_.R, current_data_.P));
    V1_pose->setId(4);
    optimizer.addVertex(V1_pose);

    auto V1_vel = new sad::VertexVelocity();
    V1_vel->setEstimate(current_data_.V);
    V1_vel->setId(5);
    optimizer.addVertex(V1_vel);

    auto v1_bg = new sad::VertexGyroBias();
    v1_bg->setId(6);
    v1_bg->setEstimate(current_data_.bg);
    optimizer.addVertex(v1_bg);

    auto v1_ba = new sad::VertexAccBias();
    v1_ba->setId(7);
    v1_ba->setEstimate(current_data_.ba);
    optimizer.addVertex(v1_ba);
}
TEST(PREINTEGRATION_TEST, ROTATION_TEST) {
    Sophus::SO3d R(Eigen::AngleAxisd(FLAGS_antenna_angle * M_PI / 180, Eigen::Vector3d::UnitZ()).matrix());
    Eigen::Vector3d P(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y, 0);
    sad::ui::PangolinWindow ui;
    ui.Init();
    optimization opti;

    gnss_process gnss;
    imu_preintegration imu_pre;
    Eigen::Vector3d pos_0;
    sad::StaticIMUInit imu_init;  // 使用默认配置
    bool imu_inited = false, gnss_inited = false;
    bool gnss_set = false;
    std::ifstream fin(FLAGS_txt_path);
    std::string line;
    while (ui.ShouldQuit() == false) {
        std::getline(fin, line);
        std::stringstream ss(line);
        if (line.empty()) {
            break;
        }
        std::stringstream ss(line);
        std::string data_type;
        ss >> data_type;
        if (data_type == "IMU") {
            // 直到初始化成功
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            if (!imu_init.InitSuccess()) {
                imu_init.AddIMU(sad::IMU(time, Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az)));
                return;
            }
            if (!imu_inited) {
                imu_pre.init(imu_init.GetGravity(), imu_init.GetInitBg(), imu_init.GetInitBa());
                imu_inited = true;
            }
            if (!gnss_inited) {
                /// 等待有效的RTK数据
                continue;
            }
            // todo 要写之后的预积分过程

        } else if (data_type == "GNSS") {
            double timestamp;
            ss >> timestamp;
            Eigen::Vector3d pos;
            ss >> pos[0] >> pos[1] >> pos[2];
            double heading;
            ss >> heading;
            bool heading_valid;
            ss >> heading_valid;
            if (!gnss_inited) {
                gnss.add_gnss(timestamp, pos, heading, heading_valid);
                opti.last_gnss_ = sad::GNSS(timestamp, 4, pos, heading, heading_valid);
                gnss_inited = true;
                continue;
            }
            Sophus ::SO3d R;
            Eigen::Vector3d P;
            gnss.get_state(R, P);
            if (!gnss_set) {
                pos_0 = P;
                gnss_set = true;
            }
            P = P - pos_0;
            opti.run(sad::GNSS(sad::GNSS(timestamp, 4, pos, heading, heading_valid)));
            opti.set_last_data(imu_pre.time_, imu_pre.dP, Eigen::Vector3d::Zero(), R, Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero());
            opti.last_gnss_ = sad::GNSS(timestamp, 4, pos, heading, heading_valid);
            ui.UpdateNavState(sad::NavState(timestamp, R, P));
            usleep(1e4);
        }
    }
    ui.Quit();
}
#endif  // SELF_TEST_PREINTEGRATION_CC