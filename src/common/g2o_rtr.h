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
class EdgeProject : public g2o::BaseBinaryEdge<3, Vec3d, sVertex, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProject();
    // EdgeProject(Eigen::Matrix<double, 3, 3> k, Vec3d p, Vec3d ui);

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError() override;

    void linearizeOplus() override;

   private:
    Mat3d k_;
    Vec3d p_;
    Vec3d ui_;
};
#endif  // MAPPING_EIGEN_TYPES_H
