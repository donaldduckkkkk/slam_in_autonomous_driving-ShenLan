//
// Created by xiang on 22-12-29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 本节程序演示一个正在作圆周运动的车辆
/// 车辆的角速度与线速度可以在flags中设置

// 通过Gflags设置程序运行参数
// Gflags的使用方法为：DEFINE_数据类型(参数名, 默认值, 参数描述)
// 使用时，可以通过FLAGS_参数名来访问参数  例如：FLAGS_angular_velocity
// 初始化参数时，可以通过 --参数名=参数值 来设置参数
DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_double(init_height, 100.0, "初始高度 m");
DEFINE_double(gravitational, -9.8, "重力加速度 m/s^2");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char** argv) {
    // 初始化Glog

    // 传入argv[0]是什么意思
    // 在C++的main函数中，第一个参数argv[0]通常是程序的名称或路径。
    // 这个参数在使用Glog库时，用于将日志输出到标准错误流中，并在日志中显示程序名称或路径。
    google::InitGoogleLogging(argv[0]);
    // 设置日志输出级别
    FLAGS_stderrthreshold = google::INFO;
    // 设置输出到屏幕
    FLAGS_colorlogtostderr = true;
    // ParseCommandLineFlags()函数解析命令行参数，将命令行参数存储到FLAGS_变量中
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                    // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    // 为什么本体系下还有速度，不应该是相对于世界坐标系的速度吗？
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);  // 本体系速度 前左上
    const double dt = 0.05;                     // 每次更新的时间

    while (ui.ShouldQuit() == false) {
        // 更新自身位置
        v_body[2] = FLAGS_gravitational* dt;
        Vec3d v_world = pose.so3() * v_body;

        pose.translation()[0] += v_world[0] * dt;
        pose.translation()[1] += v_world[1] * dt;
        pose.translation()[2] += v_body[2] * dt + 0.5 * FLAGS_gravitational * dt * dt;

        // 更新自身旋转
        if (FLAGS_use_quaternion) {
            // unit_quaternion()返回的是四元数
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            //
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }

        LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}