//
// Created by xiang on 23-1-19.
//

#ifndef SLAM_IN_AUTO_DRIVING_CH4_G2O_TYPES_H
#define SLAM_IN_AUTO_DRIVING_CH4_G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>

#include "ch4/imu_preintegration.h"
#include "common/eigen_types.h"

namespace sad {

/// 与预积分相关的vertex, edge
/**
 * 预积分边
 * 连接6个顶点：上一帧的pose, v, bg, ba，下一帧的pose, v
 * 观测量为9维，即预积分残差, 顺序：R, v, p
 * information从预积分类中获取，构造函数中计算
 */

class Vertex_pose : public g2o::BaseVertex<6, Sophus::SE3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vertex_pose(){};
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d::Zero()), Vec3d(0, 0, 0));
    }
    virtual void oplusImpl(const double* update_) override {
        _estimate.so3() = _estimate.so3() * Sophus::SO3d::exp(Vec3d(update_[0], update_[1], update_[2]));
        _estimate.translation() += Vec3d(update_[3], update_[4], update_[5]);
        updateCache();
    }
};

class Vertex_v : public g2o::BaseVertex<3, Eigen::Vector3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vertex_v(){};
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void setToOriginImpl() {}
    virtual void oplusImpl(const double* update_) override {
        _estimate += Vec3d(update_[0], update_[1], update_[2]);
        updateCache();
    }
};

class Vector_ba : public Vertex_v {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector_ba(){};
};
class Vector_bg : public Vertex_v {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector_bg(){};
};

class Edge_g : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, Vector_bg, Vector_bg> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Edge_g(){};
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void computeError() override {
        const auto bg0 = dynamic_cast<const Vector_bg*>(_vertices[0]);
        const auto bg1 = dynamic_cast<const Vector_bg*>(_vertices[1]);
        _error = bg0->estimate() - bg1->estimate();
    }
    virtual void linearizeOplus() override {
        _jacobianOplusXi = Eigen::Matrix3d::Identity();
        _jacobianOplusXj = -Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix<double, 6, 6> GetHessian() {
        Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

class Edge_a : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, Vector_ba, Vector_ba> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Edge_a(){};
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void computeError() override {
        const auto ba0 = dynamic_cast<const Vector_ba*>(_vertices[0]);
        const auto ba1 = dynamic_cast<const Vector_ba*>(_vertices[1]);
        _error = ba0->estimate() - ba1->estimate();
    }
    virtual void linearizeOplus() override {
        _jacobianOplusXi = Eigen::Matrix3d::Identity();
        _jacobianOplusXj = -Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix<double, 6, 6> GetHessian() {
        Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

class Edge_PriorPose : public g2o::BaseMultiEdge<15, Eigen::Matrix<double, 15, 1>> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Edge_PriorPose(const sad::NavStated& state, const Mat15d& info) {
        resize(4);
        setInformation(info);
        state_ = state;
    };
    sad::NavStated state_;
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void computeError() override {
        const auto vp = dynamic_cast<const Vertex_pose*>(_vertices[0]);
        const auto vv = dynamic_cast<const Vertex_v*>(_vertices[1]);
        const auto vbg = dynamic_cast<const Vector_bg*>(_vertices[2]);
        const auto vba = dynamic_cast<const Vector_ba*>(_vertices[3]);

        const Vec3d er = Sophus::SO3d(state_.R_.matrix().transpose() * vp->estimate().so3().matrix()).log();
        const Vec3d ep = vp->estimate().translation() - state_.p_;
        const Vec3d ev = vv->estimate() - state_.v_;
        const Vec3d ebg = vbg->estimate() - state_.bg_;
        const Vec3d eba = vba->estimate() - state_.ba_;

        _error << er, ep, ev, ebg, eba;
    }
    virtual void linearizeOplus() {
        const auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
        const Vec3d er = SO3(state_.R_.matrix().transpose() * vp->estimate().so3().matrix()).log();

        _jacobianOplus[0].setZero();
        _jacobianOplus[0].block<3, 3>(0, 0) = SO3::jr_inv(er);    // dr/dr
        _jacobianOplus[0].block<3, 3>(3, 3) = Mat3d::Identity();  // dp/dp
        _jacobianOplus[1].setZero();
        _jacobianOplus[1].block<3, 3>(6, 0) = Mat3d::Identity();  // dv/dv
        _jacobianOplus[2].setZero();
        _jacobianOplus[2].block<3, 3>(9, 0) = Mat3d::Identity();  // dbg/dbg
        _jacobianOplus[3].setZero();
        _jacobianOplus[3].block<3, 3>(12, 0) = Mat3d::Identity();  // dba/dba
    }
    sad::NavStated state_;
};

class EdgeInertial : public g2o::BaseMultiEdge<9, Vec9d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * 构造函数中需要指定预积分类对象
     * @param preinteg  预积分对象指针
     * @param gravity   重力矢量
     * @param weight    权重
     */
    EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg, const Vec3d& gravity, double weight = 1.0);

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() override;
    void linearizeOplus() override;

    Eigen::Matrix<double, 24, 24> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

   private:
    const double dt_;
    std::shared_ptr<IMUPreintegration> preint_ = nullptr;
    Vec3d grav_;
};

// class Edge_gnss : public g2o::BaseUnaryEdge<6, Eigen::Matrix<double, 6, 1>, Vertex_pose> {
//    public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     Edge_gnss(const sad::GNSS gnss, Eigen::Matrix<double, 6, 6> info) {
//         gnss_ = gnss;
//         setInformation(info);
//     };
//     virtual bool read(std::ifstream& is){};
//     virtual bool write(std::ofstream& os) const {};
//     virtual void computeError() {
//         const auto vp = dynamic_cast<const Vertex_pose*>(_vertices[0]);
//         Eigen::Vector3d er =
//             Sophus::SO3d(Sophus::SO3d::rotZ(gnss_.heading_).matrix().transpose() *
//             vp->estimate().so3().matrix()).log();
//         Eigen::Vector3d ep = vp->estimate().translation() - gnss_.utm_pose_.translation();
//     }
//     sad::GNSS gnss_;
// };
class Edge_gnss : public g2o::BaseUnaryEdge<6, SE3, Vertex_pose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge_gnss() = default;
    Edge_gnss(Vertex_pose* v, const SE3& obs) {
        setVertex(0, v);
        setMeasurement(obs);
    }

    void computeError() override {
        Vertex_pose* v = (Vertex_pose*)_vertices[0];
        _error.head<3>() = (_measurement.so3().inverse() * v->estimate().so3()).log();
        _error.tail<3>() = v->estimate().translation() - _measurement.translation();
    };

    void linearizeOplus() override {
        Vertex_pose* v = (Vertex_pose*)_vertices[0];
        // jacobian 6x6
        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block<3, 3>(0, 0) = (_measurement.so3().inverse() * v->estimate().so3()).jr_inv();  // dR/dR
        _jacobianOplusXi.block<3, 3>(3, 3) = Mat3d::Identity();                                              // dp/dp
    }

    Mat6d GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

   private:
};

}  // namespace sad
#endif  // SLAM_IN_AUTO_DRIVING_G2O_TYPES_H
