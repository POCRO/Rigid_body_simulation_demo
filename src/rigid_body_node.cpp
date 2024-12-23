/**
 * @file rigid_body_node.cpp
 * @brief  刚体实现   https://github.com/POCRO/Rigid_body_simulation_demo     
 * @author Porcovsky (flyingpocro@gmail.com)
 * @date          2024-11-25
 * 
 * 
 * @copyright Copyright (C) 2024, PORCOVSKY, all rights reserved.
 * 

 */
#include "rigid_body_node.hpp"
#include <fstream>



namespace rigid_body_simulation
{

// 假设 position 是 Eigen::Vector3d
void RigidBodyNode::saveTrajectory(const Eigen::Vector3d& position, const std::string& filename) {
    static std::ofstream file(filename, std::ios::app); // 打开文件 (追加模式)
    if (!file.is_open()) {
        throw std::ios_base::failure("Failed to open file for trajectory logging.");
    }

    file << position.x() << "," << position.y() << "," << position.z() << "\n"; // 写入 x, y, z
}

    void RigidBodyNode::publishTrajectory(const std::vector<Eigen::Vector3d>& trajectory) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "trajectory";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.5; // 线条宽度
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for (const auto& state : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = state[0]/1000.0;
        p.y = state[1]/1000.0;
        p.z = state[2]/1000.0;
        marker.points.push_back(p);
    }

    trajectory_publisher_->publish(marker);
    }

    RigidBodyNode::RigidBodyNode() : Node("rigid_body_simulation") {

     //   force_visualizer = std::make_shared<ForceVisualizer>();
       // trajectory_visualizer = std::make_shared<TrajectoryVisualizer>();

        // 从参数服务器读取初始参数
        this->declare_parameter<double>("mass", 1.0);
        this->declare_parameter<double>("friction_coefficient", 0.0);
        this->declare_parameter<double>("time_step", 0.01);
        this->declare_parameter<double>("lift_coefficient", 0.5);
        this->declare_parameter<double>("side_coefficient", 0.0);
        this->declare_parameter<double>("rou_0", 0.0);
        this->declare_parameter<double>("S", 0.0);


        this->declare_parameter<std::vector<double>>(
            "inertia", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        this->declare_parameter<std::vector<double>>("external_force",
                                                     {0.0, 0.0, -9.8});
        this->declare_parameter<std::vector<double>>("external_torque",
                                                     {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("gravity",
                                                     {0.0, 0.0, -9.8});
        this->declare_parameter<std::vector<double>>("initial_position",
                                                     {0.0, 0.0, 10});
        this->declare_parameter<std::vector<double>>("initial_velocity",
                                                     {0.0, 0.0, 0.0});

        mass_ = get_parameter("mass").as_double();
        friction_ = get_parameter("friction_coefficient").as_double();
        dt_ = get_parameter("time_step").as_double();
        lift_coefficient_ = get_parameter("lift_coefficient").as_double();
        side_coefficient_ = get_parameter("side_coefficient").as_double();
        rou_0_ = get_parameter("rou_0").as_double();
        S_ = get_parameter("S").as_double();
        

        std::vector<double> inertia_vec =
            get_parameter("inertia").as_double_array();
        inertia_ = Eigen::Matrix3d::Map(inertia_vec.data());

        // 读取 initial_position 参数并转换为 Eigen::Vector3d
        std::vector<double> initial_position_vec =
            get_parameter("initial_position").as_double_array();
        if (initial_position_vec.size() != 3) {
            RCLCPP_ERROR(
                this->get_logger(),
                "initial_position vector must have exactly 3 elements!");
            throw std::runtime_error("Invalid initial_position parameter size");
        }
        initial_position_ =
            Eigen::Vector3d(initial_position_vec[0], initial_position_vec[1],
                            initial_position_vec[2]);
        // 读取 initial_velocity 参数并转换为 Eigen::Vector3d
        std::vector<double> initial_velocity_vec =
            get_parameter("initial_velocity").as_double_array();
        if (initial_velocity_vec.size() != 3) {
            RCLCPP_ERROR(
                this->get_logger(),
                "initial_velocity vector must have exactly 3 elements!");
            throw std::runtime_error("Invalid initial_velocity parameter size");
        }
        initial_velocity_ =
            Eigen::Vector3d(initial_velocity_vec[0], initial_velocity_vec[1],
                            initial_velocity_vec[2]);

        // 读取 external_force 参数并转换为 Eigen::Vector3d
        std::vector<double> external_force_vec =
            get_parameter("external_force").as_double_array();
        if (external_force_vec.size() != 3) {
            RCLCPP_ERROR(
                this->get_logger(),
                "external_force vector must have exactly 3 elements!");
            throw std::runtime_error("Invalid external_force parameter size");
        }
        external_force_ =
            Eigen::Vector3d(external_force_vec[0], external_force_vec[1],
                            external_force_vec[2]);

        // 读取 external_torque 参数并转换为 Eigen::Vector3d
        std::vector<double> external_torque_vec =
            get_parameter("external_torque").as_double_array();
        if (external_torque_vec.size() != 3) {
            RCLCPP_ERROR(
                this->get_logger(),
                "external_torque vector must have exactly 3 elements!");
            throw std::runtime_error("Invalid external_torque parameter size");
        }
        external_torque_ =
            Eigen::Vector3d(external_torque_vec[0], external_torque_vec[1],
                            external_torque_vec[2]);

        // 读取 gravity 参数并转换为 Eigen::Vector3d
        std::vector<double> gravity_vec =
            get_parameter("gravity").as_double_array();
        if (gravity_vec.size() != 3) {
            RCLCPP_ERROR(
                this->get_logger(),
                "gravity vector must have exactly 3 elements!");
            throw std::runtime_error("Invalid gravity parameter size");
        }
        gravity_ =
            Eigen::Vector3d(gravity_vec[0], gravity_vec[1],
                            gravity_vec[2]);

        // 初始化初始状态
        initial_state_.position = initial_position_;
        initial_state_.velocity = initial_velocity_;
        initial_state_.orientation = Eigen::Quaterniond::Identity();  // 默认单位四元数
        initial_state_.angular_velocity = Eigen::Vector3d::Zero();

        // 将当前状态设置为初始状态
        current_state = initial_state_;

        trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            std::bind(&RigidBodyNode::update, this));
    }

//    private:
//     // 定义刚体状态
//     Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
//     Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
//     Eigen::Vector3d initial_position_, initial_velocity_,external_force_, external_torque_, gravity_;

//     Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
//     Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();

//     RigidBodyState initial_state_;

double cnt = 0;

// 刚体状态更新
    void RigidBodyNode::update() {
        // 使用 Runge-Kutta 方法更新状态
        Eigen::Vector3d force = compute_force(current_state);
        current_state =
            runge_kutta_step(current_state, force, external_torque_,
                             mass_, inertia_, dt_);
        
        trajectory_.push_back(current_state.position);
        
        publishTrajectory(trajectory_);

        saveTrajectory(current_state.position, "trajectory_with_lift.csv");




        // 发布状态更新到 TF
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = current_state.position.x()/1000.0;
        transform.transform.translation.y = current_state.position.y()/1000.0;
        transform.transform.translation.z = current_state.position.z()/1000.0;
        transform.transform.rotation.x = current_state.orientation.x();
        transform.transform.rotation.y = current_state.orientation.y();
        transform.transform.rotation.z = current_state.orientation.z();
        transform.transform.rotation.w = current_state.orientation.w();
        tf_broadcaster_->sendTransform(transform);
    }

// 刚体受力计算

    Eigen::Vector3d RigidBodyNode::compute_force(const RigidBodyState& state) {
        // 重力
        Eigen::Vector3d gravity_force(0, 0, -mass_ * 9.81* pow(6371000 / (6371000 + state.position.z()), 2));

        // 恒外力
        Eigen::Vector3d constant_force = external_force_;

        // 气动力 之 摩擦阻力部分
        
        //更新气体密度
        double air_density = rou_0_ * exp(- state.position.z() / 8500);
        // 摩擦力方向与速度方向相反
        Eigen::Vector3d friction_force = -friction_ * state.velocity* state.velocity.norm();

        // 气动力 之 升力
        // //升力方向垂直与速度方向向上
         Eigen::Vector3d lift_force(0, 0, 0);
        // if (state.velocity.norm() > 1e-6) { // 确保速度非零，避免除以零
        //     Eigen::Vector3d upward_vector(0, 0, 1); // 垂直向上的基准方向
        //     Eigen::Vector3d lift_direction = state.velocity.cross(upward_vector).normalized(); // 升力方向
        //     lift_force = 0.5*air_density*lift_coefficient_ * state.velocity.squaredNorm() * lift_direction; // 升力大小
        // }

        // 气动力 之 侧力 升力
        // 侧力方向垂直于速度方向
        Eigen::Vector3d side_force(0, 0, 0);
        if (state.velocity.norm() > 1e-6) { // 确保速度非零，避免除以零
            Eigen::Vector3d upward_vector(0, 0, 1); // 垂直向上的基准方向
            Eigen::Vector3d side_direction = state.velocity.cross(upward_vector).normalized(); // 侧力方向
            side_force = 0.5*air_density*side_coefficient_ * state.velocity.squaredNorm() * side_direction; // 侧力大小

            Eigen::Vector3d lift_direction = side_direction.cross(state.velocity).normalized(); // 升力方向
            lift_force = 0.5*air_density*lift_coefficient_ * state.velocity.squaredNorm() * lift_direction; // 升力大小
        }



        // 发布力方向 Marker
        // force_visualizer->publish_force_markers(position_, lift_force, constant_force, friction_force);



        // 总力
        return gravity_force + constant_force + friction_force + lift_force + side_force;
    }
    
// 微分方程的计算
RigidBodyState RigidBodyNode::compute_derivatives(const RigidBodyState& state,
                                   const Eigen::Vector3d& force,
                                   const Eigen::Vector3d& torque, double mass,
                                   const Eigen::Matrix3d& inertia) {
    RigidBodyState derivatives;

    // 平动部分
    derivatives.position = state.velocity;
    derivatives.velocity = force / mass;

    // 转动部分
    Eigen::Vector3d angular_momentum = inertia * state.angular_velocity;
    Eigen::Vector3d angular_acceleration =
        inertia.inverse() *
        (torque - state.angular_velocity.cross(angular_momentum));
    derivatives.angular_velocity = angular_acceleration;

    // 姿态四元数微分
    Eigen::Quaterniond omega_quaternion(0, state.angular_velocity.x(),
                                        state.angular_velocity.y(),
                                        state.angular_velocity.z());
    derivatives.orientation.coeffs() =
        0.5 * (omega_quaternion * state.orientation).coeffs();

    return derivatives;
}

// 四阶龙哥库塔

RigidBodyState RigidBodyNode::runge_kutta_step(const RigidBodyState& current_state,
                                const Eigen::Vector3d& force,
                                const Eigen::Vector3d& torque, double mass,
                                const Eigen::Matrix3d& inertia, double dt) {
    // 计算 k1
    RigidBodyState k1 =
        compute_derivatives(current_state, force, torque, mass, inertia);

    // 计算 k2
    RigidBodyState k2_input = current_state;
    k2_input.position += 0.5 * dt * k1.position;
    k2_input.velocity += 0.5 * dt * k1.velocity;
    k2_input.orientation.coeffs() += 0.5 * dt * k1.orientation.coeffs();
    k2_input.angular_velocity += 0.5 * dt * k1.angular_velocity;
    RigidBodyState k2 =
        compute_derivatives(k2_input, force, torque, mass, inertia);

    // 计算 k3
    RigidBodyState k3_input = current_state;
    k3_input.position += 0.5 * dt * k2.position;
    k3_input.velocity += 0.5 * dt * k2.velocity;
    k3_input.orientation.coeffs() += 0.5 * dt * k2.orientation.coeffs();
    k3_input.angular_velocity += 0.5 * dt * k2.angular_velocity;
    RigidBodyState k3 =
        compute_derivatives(k3_input, force, torque, mass, inertia);

    // 计算 k4
    RigidBodyState k4_input = current_state;
    k4_input.position += dt * k3.position;
    k4_input.velocity += dt * k3.velocity;
    k4_input.orientation.coeffs() += dt * k3.orientation.coeffs();
    k4_input.angular_velocity += dt * k3.angular_velocity;
    RigidBodyState k4 =
        compute_derivatives(k4_input, force, torque, mass, inertia);

    // 更新状态
    RigidBodyState next_state = current_state;
    next_state.position +=
        dt / 6.0 *
        (k1.position + 2.0 * k2.position + 2.0 * k3.position + k4.position);
    next_state.velocity +=
        dt / 6.0 *
        (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity);
    next_state.orientation.coeffs() +=
        dt / 6.0 *
        (k1.orientation.coeffs() + 2.0 * k2.orientation.coeffs() +
         2.0 * k3.orientation.coeffs() + k4.orientation.coeffs());
    next_state.angular_velocity +=
        dt / 6.0 *
        (k1.angular_velocity + 2.0 * k2.angular_velocity +
         2.0 * k3.angular_velocity + k4.angular_velocity);

    // 归一化四元数
    next_state.orientation.normalize();

    return next_state;
}

}
// 主函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rigid_body_simulation::RigidBodyNode>());
    rclcpp::shutdown();
    return 0;
}