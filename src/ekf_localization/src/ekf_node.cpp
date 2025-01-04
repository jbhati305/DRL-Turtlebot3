#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node") {
        // Initialize state vector and covariance matrix
        state_ = Eigen::VectorXd::Zero(5); // [x, y, theta, v, omega]
        P_ = Eigen::MatrixXd::Identity(5, 5) * 0.1;

        // Set process and measurement noise covariance
        Q_ = Eigen::MatrixXd::Identity(5, 5) * 0.1;
        R_imu_ = Eigen::MatrixXd::Identity(1, 1) * 0.05;
        R_odom_ = Eigen::MatrixXd::Identity(3, 3) * 0.01;

        // Subscriptions
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&EKFNode::odomCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&EKFNode::imuCallback, this, std::placeholders::_1));

        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_pose", 10);
    }

private:
    Eigen::VectorXd state_;       // State vector [x, y, theta, v, omega]
    Eigen::MatrixXd P_;           // Covariance matrix
    Eigen::MatrixXd Q_;           // Process noise covariance
    Eigen::MatrixXd R_imu_;       // IMU measurement noise covariance
    Eigen::MatrixXd R_odom_;      // Odometry measurement noise covariance
    rclcpp::Time last_time_;      // Last time step

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    void predict(double dt) {
        double x = state_(0), y = state_(1), theta = state_(2);
        double v = state_(3), omega = state_(4);

        // Motion model prediction
        state_(0) = x + v * cos(theta) * dt;
        state_(1) = y + v * sin(theta) * dt;
        state_(2) = theta + omega * dt;

        // Jacobian of the motion model
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 3) = cos(theta) * dt;
        F(1, 3) = sin(theta) * dt;
        F(0, 4) = -v * sin(theta) * dt;
        F(1, 4) = v * cos(theta) * dt;

        // Update covariance
        P_ = F * P_ * F.transpose() + Q_;
    }

    void updateIMU(const sensor_msgs::msg::Imu::SharedPtr &msg) {
        double yaw_meas = msg->orientation.z; // Simplified; convert quaternion to yaw if needed

        // Measurement model (yaw only)
        Eigen::MatrixXd H(1, 5);
        H.setZero();
        H(0, 2) = 1;

        Eigen::VectorXd z(1);
        z(0) = yaw_meas;

        // Innovation
        Eigen::VectorXd y = z - H * state_;
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // Update state and covariance
        state_ += K * y;
        P_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * P_;
    }

    void updateOdom(const nav_msgs::msg::Odometry::SharedPtr &msg) {
        double x_meas = msg->pose.pose.position.x;
        double y_meas = msg->pose.pose.position.y;
        double theta_meas = msg->pose.pose.orientation.z; // Simplified; convert quaternion to yaw if needed

        // Measurement model (x, y, theta)
        Eigen::MatrixXd H(3, 5);
        H.setZero();
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 1;

        Eigen::VectorXd z(3);
        z << x_meas, y_meas, theta_meas;

        // Innovation
        Eigen::VectorXd y = z - H * state_;
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // Update state and covariance
        state_ += K * y;
        P_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * P_;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        if (last_time_.nanoseconds() != 0) {
            double dt = (current_time - last_time_).seconds();
            predict(dt);
            updateOdom(msg);
            publishPose();
        }
        last_time_ = current_time;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        updateIMU(msg);
        publishPose();
    }

    void publishPose() {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = state_(0);
        pose_msg.pose.pose.position.y = state_(1);
        pose_msg.pose.pose.orientation.z = state_(2); // Simplified; convert to quaternion if needed

        for (size_t i = 0; i < 5; ++i) {
            for (size_t j = 0; j < 5; ++j) {
                pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
            }
        }

        pose_pub_->publish(pose_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
