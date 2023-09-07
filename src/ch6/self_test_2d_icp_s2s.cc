//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <opencv2/imgproc.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include "boost/cast.hpp"
#include "boost/numeric/conversion/cast.hpp"
#include "ch6/icp_2d.h"
#include "ch6/lidar_2d_utils.h"
#include "common/io_utils.h"
DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "point2point", "2d icp方法：point2point/point2plane");
/// 测试从rosbag中读取2d scan并plot的结果
/// 通过选择method来确定使用点到点或点到面的ICP
class ICP_my {
    // ICP_my(){};
   public:
    void SetTarget(Scan2d::Ptr scan_tar) { scan_tar_ = scan_tar; }
    void SetSource(Scan2d::Ptr scan_sou) { scan_sou_ = scan_sou; }
    bool AlignGaussNewton(Sophus::SE2d pose);
    bool AlignGaussNewtonPoint2Plane(Sophus::SE2d pose);
    void line_fitting(std::vector<Eigen::Vector3d> data, Eigen::Vector3d lineinit, Eigen::Vector3d& line,
                      double threshold = 0.2);
    void BuildTree();
    Scan2d::Ptr scan_tar_;
    Scan2d::Ptr scan_sou_;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    double lost_threshold_ = 0.1;
};
void ICP_my::BuildTree() {
    for (int i = 0; i < scan_tar_->ranges.size(); i++) {
        if (scan_tar_->ranges[i] < scan_tar_->range_min || scan_tar_->ranges[i] > scan_tar_->range_max) continue;
        double real_angle = scan_tar_->angle_min + i * scan_tar_->angle_increment;
        pcl::PointXY point;
        point.x = scan_tar_->ranges[i] * std::cos(real_angle);
        point.y = scan_tar_->ranges[i] * std::sin(real_angle);
        if (real_angle < scan_tar_->angle_min + 30 * M_PI / 180 || real_angle > scan_tar_->angle_max - 30 * M_PI / 180)
            continue;
        cloud->points.push_back(point);
        kdtree.setInputCloud(cloud);
    }
}
bool ICP_my::AlignGaussNewton(Sophus::SE2d pose) {
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
    int k = 1;
    double cost;
    int num = 0;
    int iter_num = 5;
    std::vector<int> nn_idx(k);
    std::vector<float> nn_squared_distance(k);
    for (int iter = 0; iter < iter_num; iter++) {
        Eigen::Matrix<double, 3, 3> H = Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, 3, 1> b = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 2> J = Eigen::Matrix<double, 3, 2>::Zero();
        Eigen::Matrix<double, 2, 1> e = Eigen::Matrix<double, 2, 1>::Zero();
        for (int i = 0; i < scan_sou_->ranges.size(); i++) {
            if (scan_sou_->ranges[i] < scan_sou_->range_min || scan_sou_->ranges[i] > scan_sou_->range_max) continue;
            double real_angle = scan_sou_->angle_min + i * scan_sou_->angle_increment;
            pcl::PointXY search_point_;
            search_point_.x = scan_sou_->ranges[i] * std::cos(real_angle);
            search_point_.y = scan_sou_->ranges[i] * std::sin(real_angle);
            Eigen::Vector2d search_point = pose.so2().matrix() * Eigen::Vector2d(search_point_.x, search_point_.y);
            if (real_angle < scan_sou_->angle_min + 30 * M_PI / 180 ||
                real_angle > scan_sou_->angle_max - 30 * M_PI / 180)
                continue;
            double theta = pose.so2().log();
            pcl::PointXY pt;
            pt.x = search_point[0];
            pt.y = search_point[1];
            if (kdtree.nearestKSearch(pt, k, nn_idx, nn_squared_distance) > 0) {
                for (size_t i = 0; i < nn_idx.size(); ++i) {
                    if (!nn_squared_distance[i] < 0.01) {
                        continue;
                    }
                    std::cout << "point " << i + 1 << ", x:" << cloud->points[nn_idx[i]].x
                              << ", y:" << cloud->points[nn_idx[i]].y << ", distance:" << nn_squared_distance.at(i)
                              << ")" << std::endl;
                    J << 1, 0, 0, 1, -scan_sou_->ranges[i] * std::sin(real_angle + theta),
                        scan_sou_->ranges[i] * std::cos(real_angle + theta);
                    H += J * J.transpose();
                    e[0] = double(pt.x - cloud->points[nn_idx[i]].x);
                    e[1] = (pt.y - cloud->points[nn_idx[i]].y);
                    b += -J * e;
                    cost += e.transpose() * e;
                    num++;
                }
            }
        }
        if (num < 10 || cost / num > lost_threshold_ || iter == 0) return false;

        Eigen::Vector3d update = H.ldlt().solve(b);

        if (isnan(update[0])) return false;
        pose.translation() += update.head<2>();
        pose.so2() = Sophus::SO2d::exp(update[2]) * pose.so2();
    }
    // LOG(INFO) << "estimated pose: " << pose.translation().transpose() << ", theta: " << pose.so2().log();

    return true;
}
void ICP_my::line_fitting(std::vector<Eigen::Vector3d> data, Eigen::Vector3d lineinit, Eigen::Vector3d& line,
                          double threshold = 0.2) {
    Eigen::Vector3d origin = std::accumulate(data.begin(), data.end(), Eigen::Vector3d(0, 0, 0)) / data.size();
    const int num = data.size();
    Eigen::MatrixXd Y(num, 3);
    for (int i = 0; i < data.size(); i++) {
        data[i] = data[i] - origin;
    }
    Eigen::JacobiSVD(Eigen::Matrix3d::Zero(), Eigen::ComputeFullU | Eigen::ComputeFullV);
}
bool ICP_my::AlignGaussNewtonPoint2Plane(Sophus::SE2d pose) {
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
    int k = 10;
    double cost;
    int num = 0;
    int iter_num = 5;
    std::vector<int> nn_idx(k);
    std::vector<float> nn_squared_distance(k);
    for (int iter = 0; iter < iter_num; iter++) {
        Eigen::Matrix<double, 3, 3> H = Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, 3, 1> b = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix<double, 3, 2> J = Eigen::Matrix<double, 3, 2>::Zero();
        Eigen::Matrix<double, 2, 1> e = Eigen::Matrix<double, 2, 1>::Zero();
        for (int i = 0; i < scan_sou_->ranges.size(); i++) {
            if (scan_sou_->ranges[i] < scan_sou_->range_min || scan_sou_->ranges[i] > scan_sou_->range_max) continue;
            double real_angle = scan_sou_->angle_min + i * scan_sou_->angle_increment;
            pcl::PointXY search_point_;
            search_point_.x = scan_sou_->ranges[i] * std::cos(real_angle);
            search_point_.y = scan_sou_->ranges[i] * std::sin(real_angle);
            Eigen::Vector2d search_point = pose.so2().matrix() * Eigen::Vector2d(search_point_.x, search_point_.y);
            if (real_angle < scan_sou_->angle_min + 30 * M_PI / 180 ||
                real_angle > scan_sou_->angle_max - 30 * M_PI / 180)
                continue;
            double theta = pose.so2().log();
            pcl::PointXY pt;
            pt.x = search_point[0];
            pt.y = search_point[1];
            if (kdtree.nearestKSearch(pt, k, nn_idx, nn_squared_distance) > 0) {
                // todo 改成直线拟合
                if (nn_idx.size() < 5) continue;

                // for (size_t i = 0; i < nn_idx.size(); ++i) {

                // if (!nn_squared_distance[i] < 0.01) {
                //     continue;
                // }
                // std::cout << "point " << i + 1 << ", x:" << cloud->points[nn_idx[i]].x
                //           << ", y:" << cloud->points[nn_idx[i]].y << ", distance:" << nn_squared_distance.at(i)
                //           << ")" << std::endl;
                // J << 1, 0, 0, 1, -scan_sou_->ranges[i] * std::sin(real_angle + theta),
                //     scan_sou_->ranges[i] * std::cos(real_angle + theta);
                // H += J * J.transpose();
                // e[0] = double(pt.x - cloud->points[nn_idx[i]].x);
                // e[1] = (pt.y - cloud->points[nn_idx[i]].y);
                // b += -J * e;
                // cost += e.transpose() * e;
                // num++;
                // }
            }
        }
        if (num < 10 || cost / num > lost_threshold_ || iter == 0) return false;

        Eigen::Vector3d update = H.ldlt().solve(b);

        if (isnan(update[0])) return false;
        pose.translation() += update.head<2>();
        pose.so2() = Sophus::SO2d::exp(update[2]) * pose.so2();
    }
    // LOG(INFO) << "estimated pose: " << pose.translation().transpose() << ", theta: " << pose.so2().log();

    return true;
}
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap, int width) {
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));

        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            // image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
            cv::circle(image, cv::Point2f(image_x, image_y), width, cv::Scalar(color[0], color[1], color[2]), 3);
        }
    }

    // 同时画出pose自身所在位置
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), width, cv::Scalar(color[0], color[1], color[2]),
               2);
}
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

    /// 我们将上一个scan与当前scan进行配准
    rosbag_io
        .AddScan2DHandle(
            "/pavo_scan_bottom",
            [&](Scan2d::Ptr scan) {
                current_scan = scan;

                if (last_scan == nullptr) {
                    last_scan = current_scan;
                    return true;
                }

                sad::Icp2d icp;
                icp.SetTarget(last_scan);
                icp.SetSource(current_scan);

                SE2 pose;
                if (fLS::FLAGS_method == "point2point") {
                    icp.AlignGaussNewton(pose);
                } else if (fLS::FLAGS_method == "point2plane") {
                    icp.AlignGaussNewtonPoint2Plane(pose);
                }
                ICP_my icp_my;
                icp_my.SetTarget(last_scan);
                icp_my.SetSource(current_scan);

                SE2 pose_init;
                icp.AlignGaussNewton(pose_init);

                cv::Mat image;
                sad::Visualize2DScan(last_scan, SE2(), image, Vec3b(255, 0, 0));  // target是蓝的
                Visualize2DScan(current_scan, pose_init, image, Vec3b(0, 255, 0), 800, 20.0, SE2(),
                                8);                                                 // source——my是绿的
                sad::Visualize2DScan(current_scan, pose, image, Vec3b(0, 0, 255));  // source是红的

                std::cout << "so2_error:" << (pose_init.so2().inverse() * pose_init.so2()).log() << std::endl;

                std::cout << "translation_error:" << pose.translation() - pose_init.translation() << std::endl;

                usleep(1000);
                cv::imshow("scan", image);
                cv::waitKey(20);

                last_scan = current_scan;
                return true;
            })
        .Go();

    return 0;
}