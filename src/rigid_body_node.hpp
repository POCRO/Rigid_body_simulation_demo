#ifndef __RIGID_BODY_NODE_HPP__
#define __RIGID_BODY_NODE_HPP__
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
//#include <stdio.h>
namespace rigid_body_simulation {

// 刚体状态结构体
struct RigidBodyState {
    Eigen::Vector3d position;          // 位移 r
    Eigen::Vector3d velocity;          // 速度 v
    Eigen::Quaterniond orientation;    // 姿态 q
    Eigen::Vector3d angular_velocity;  // 角速度 ω
};

class RigidBodyNode : public rclcpp::Node {
   public:
    /**
     * @brief         Construct a new Rigid Body Node object
     * @author Porcovsky (flyingpocro@gmail.com)
     */
    RigidBodyNode();  //: Node("rigid_body_simulation") override;
    //  ~RigidBodyNode() ;//: Node("rigid_body_simulation") override;
   private:
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d external_force_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d external_torque_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();

    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();

    RigidBodyState initial_state_;

    RigidBodyState current_state = initial_state_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double mass_, friction_, time_step_, dt_, lift_coefficient_,side_coefficient_;
    /**
     * @brief         状态更新
     * @author Porcovsky (flyingpocro@gmail.com)
     */
    void update();
    /**
     * @brief         受力计算函数
     * @param[in] state         My Param doc
     * @return Eigen::Vector3d 
     * @author Porcovsky (flyingpocro@gmail.com)
     */
    Eigen::Vector3d compute_force(const RigidBodyState& state);
    /**
     * @brief         状态空间微分计算
     * @return RigidBodyState 
     * @author Porcovsky (flyingpocro@gmail.com)
     */
    RigidBodyState compute_derivatives(const RigidBodyState& state,
                                       const Eigen::Vector3d& force,
                                       const Eigen::Vector3d& torque,
                                       double mass,
                                       const Eigen::Matrix3d& inertia);
   /**
    * @brief         四阶龙哥库塔法函数
    * @return RigidBodyState 
    * @author Porcovsky (flyingpocro@gmail.com)
    */
    RigidBodyState runge_kutta_step(const RigidBodyState& current_state,
                                    const Eigen::Vector3d& force,
                                    const Eigen::Vector3d& torque, double mass,
                                    const Eigen::Matrix3d& inertia, double dt);
};
}  // namespace rigid_body_simulation

#endif
