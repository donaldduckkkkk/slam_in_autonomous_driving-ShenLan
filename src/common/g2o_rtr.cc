//
// Created by xiang on 2021/7/20.
//

#ifndef SLAM_IN_AUTO_DRIVING_COMMON_G2O_RTR_H
#define SLAM_IN_AUTO_DRIVING_COMMON_G2O_RTR_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "common/gnss.h"
#include "common/nav_state.h"

#include "ch4/imu_preintegration.h"
#include "g2o/core/robust_kernel_impl.h"

#include <glog/logging.h>

// 顶点 尺度

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class sVertex : public g2o::BaseVertex<1, double> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    sVertex(bool fixed = false) {
        setToOriginImpl();
        setFixed(fixed);
    }

    sVertex(double e, bool fixed = false) {
        _estimate = e;
        setFixed(fixed);
    }

    // 重置
    virtual void setToOriginImpl() override { _estimate = 0; }

    // 更新
    virtual void oplusImpl(const double* update) override { _estimate += *update; }

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
};

// 顶点 T
// VertexPose
class VertexPose : public g2o::BaseVertex<6, SE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}

    bool read(std::istream& is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }
        setEstimate(SE3(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2])));
    }

    bool write(std::ostream& os) const override {
        os << "VERTEX_SE3:QUAT ";
        os << id() << " ";
        Quatd q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        return true;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update_) {
        _estimate.so3() = _estimate.so3() * SO3::exp(Eigen::Map<const Vec3d>(&update_[0]));  // 旋转部分
        _estimate.translation() += Eigen::Map<const Vec3d>(&update_[3]);                     // 平移部分
        updateCache();
    }
};

// 边
class EdgeProject : public g2o::BaseBinaryEdge<3, Vec3d, VertexPose, sVertex> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EdgeProject(){};
    EdgeProject(Eigen::Matrix<double, 3, 3> k, Vec3d p, Vec3d ui) : k_(k), p_(p), ui_(ui) {
        resize(2);  // 6个关联顶点
    }
    void computeError() override {
        auto* s1 = dynamic_cast<const sVertex*>(_vertices[1]);
        auto* T1 = dynamic_cast<const VertexPose*>(_vertices[0]);

        double s1_ = s1->estimate();
        SE3 T = T1->estimate();
        Eigen::Matrix3d scalarMatrix = Eigen::Matrix3d::Identity() * s1_;
        Sophus::SO3d s = Sophus::SO3d(scalarMatrix);
        _error = p_ - (s * T.so3() * SO3(k_.inverse()) * ui_ + T.translation());
    };

    void linearizeOplus() override {
        auto* s1 = dynamic_cast<const sVertex*>(_vertices[1]);
        auto* T1 = dynamic_cast<const VertexPose*>(_vertices[0]);

        _jacobianOplusXj.setZero();
        _jacobianOplusXj.block<3, 1>(0, 0) =
            -(T1->estimate().so3() * SO3(k_.inverse()) * ui_ * s1->estimate() + T1->estimate().translation());
        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block<3, 3>(0, 0) = -Mat3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) =
            SO3::hat(T1->estimate().so3() * SO3(k_.inverse()) * ui_ * s1->estimate() + T1->estimate().translation());
    };

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

   private:
    Mat3d k_;
    Vec3d p_;
    Vec3d ui_;
};

int main(int argc, char const* argv[]) {
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    auto pose = new VertexPose();
    pose->setId(0);
    Sophus::SE3d a;
    pose->setEstimate(a);
    optimizer.addVertex(pose);

    auto s = new sVertex();
    s->setId(0);
    float b;
    s->setEstimate(b);
    optimizer.addVertex(s);

    Eigen::Matrix<double, 3, 3> k;
    Vec3d p;
    Vec3d ui;
    auto edge = new EdgeProject(k, p, ui);
    edge->setVertex(0, pose);
    edge->setVertex(1, s);
    Eigen::Matrix<double, 3, 3> information;
    information << 1, 0, 0, 0, 1, 0, 0, 0, 30;
    edge->setInformation(information);
    optimizer.addEdge(edge);
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    std::cout << "pose->estimate():" << pose->estimate().matrix() << std::endl;
    std::cout << "s->estimate():" << s->estimate() << std::endl;
    return 0;
}

#endif  // MAPPING_EIGEN_TYPES_H
