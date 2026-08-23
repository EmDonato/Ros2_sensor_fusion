#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

class Ekf : public rclcpp::Node
{
public:
    Ekf()
    : Node("ekf"),
      verbal_logger_(this->get_logger().get_child("verbal_logger"))
    {
        const auto initial_covariance_diagonal =
            this->declare_parameter<std::vector<double>>(
                "initial_state_covariance_diagonal",
                {0.0001, 0.0001, 0.01, 0.01, 0.1, 0.01});
        const auto process_noise_diagonal =
            this->declare_parameter<std::vector<double>>(
                "process_noise_covariance_diagonal",
                {0.0, 0.0, 0.0, 0.0025, 0.0225, 0.000025});
        const auto encoder_covariance_diagonal =
            this->declare_parameter<std::vector<double>>(
                "encoder_measurement_covariance_diagonal",
                {0.0004, 0.0025});

        declare_and_load_scalar_parameters();
        declare_and_load_interface_parameters();
        validate_parameters(
            initial_covariance_diagonal,
            process_noise_diagonal,
            encoder_covariance_diagonal);

        x_.setZero();

        P_.setZero();
        Q_.setZero();
        for (Eigen::Index i = 0; i < 6; ++i) {
            P_(i, i) = initial_covariance_diagonal.at(i);
            Q_(i, i) = process_noise_diagonal.at(i);
        }

        F_.setIdentity();

        H_enc_ <<
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0;

        R_enc_.setZero();
        R_enc_.diagonal() << encoder_covariance_diagonal.at(0),
                             encoder_covariance_diagonal.at(1);

        H_imu_ << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0;

        R_imu_.setZero();
        R_imu_(0, 0) = imu_measurement_variance_;

        last_predict_time_ = this->now();

        if (publish_tf_) {
            tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);

        ref_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                cmd_vel_topic_,
                rclcpp::QoS(10),
                std::bind(&Ekf::reference_reader, this, std::placeholders::_1)
            );

        imu_sub_ =
            this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&Ekf::imu_correction, this, std::placeholders::_1)
            );

        if (encoder_input_type_ == "twist") {
            encoder_twist_sub_ =
                this->create_subscription<geometry_msgs::msg::TwistStamped>(
                    encoder_twist_topic_,
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &Ekf::encoder_twist_reader,
                        this,
                        std::placeholders::_1)
                );
        } else {
            joint_states_sub_ =
                this->create_subscription<sensor_msgs::msg::JointState>(
                    joint_states_topic_,
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &Ekf::joint_states_reader,
                        this,
                        std::placeholders::_1)
                );
        }

        const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / prediction_frequency_hz_));
        timer_ =
            this->create_wall_timer(
                timer_period,
                std::bind(&Ekf::predict, this)
            );

        RCLCPP_INFO(
            this->get_logger(),
            "EKF node started with '%s' encoder input",
            encoder_input_type_.c_str());
    }

private:
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix2d = Eigen::Matrix<double, 2, 2>;
    using Matrix1d = Eigen::Matrix<double, 1, 1>;

    rclcpp::Logger verbal_logger_;
    std::mutex ekf_mutex_;

    double radius_ = 0.0346;
    double wheel_base_ = 0.2;

    double alpha_v_ = 6.0;
    double alpha_w_ = 10.0;

    double nominal_dt_ = 0.02;
    double max_dt_ = 0.1;

    double imu_gate_ = 9.0;
    double enc_gate_ = 9.21;
    double prediction_frequency_hz_ = 50.0;
    double imu_measurement_variance_ = 0.0009;

    std::string encoder_input_type_ = "twist";
    std::string cmd_vel_topic_ = "cmd_vel";
    std::string imu_topic_ = "imu";
    std::string encoder_twist_topic_ = "enc/twist_meas";
    std::string joint_states_topic_ = "joint_states";
    std::string odometry_topic_ = "odom";
    std::string odom_frame_id_ = "odom";
    std::string base_frame_id_ = "base_link";
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;

    double left_wheel_velocity_multiplier_ = 1.0;
    double right_wheel_velocity_multiplier_ = 1.0;

    bool use_imu_message_covariance_ = true;
    bool publish_tf_ = true;

    double v_ref_ = 0.0;
    double w_ref_ = 0.0;

    rclcpp::Time last_predict_time_;

    /*
        State:
        x_(0) = x
        x_(1) = y
        x_(2) = theta
        x_(3) = v
        x_(4) = w
        x_(5) = gyro bias
    */
    Vector6d x_;

    Matrix6d P_;
    Matrix6d Q_;
    Matrix6d F_;

    Eigen::Matrix<double, 2, 6> H_enc_;
    Matrix2d R_enc_;
    Matrix2d S_enc_;
    Eigen::Matrix<double, 2, 1> diff_enc_;
    Eigen::Matrix<double, 6, 2> K_enc_;

    Eigen::Matrix<double, 1, 6> H_imu_;
    Matrix1d R_imu_;
    Matrix1d S_imu_;
    Matrix1d diff_imu_;
    Eigen::Matrix<double, 6, 1> K_imu_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        encoder_twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_states_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void declare_and_load_scalar_parameters()
    {
        wheel_base_ = this->declare_parameter<double>("wheel_base", 0.2);
        radius_ = this->declare_parameter<double>("radius", 0.0346);
        alpha_v_ = this->declare_parameter<double>("alpha_v", 6.0);
        alpha_w_ = this->declare_parameter<double>("alpha_w", 10.0);
        nominal_dt_ = this->declare_parameter<double>("nominal_dt", 0.02);
        max_dt_ = this->declare_parameter<double>("max_dt", 0.1);
        imu_gate_ = this->declare_parameter<double>(
            "imu_mahalanobis_threshold", 9.0);
        enc_gate_ = this->declare_parameter<double>(
            "enc_mahalanobis_threshold", 9.21);
        prediction_frequency_hz_ = this->declare_parameter<double>(
            "prediction_frequency_hz", 50.0);
        imu_measurement_variance_ = this->declare_parameter<double>(
            "imu_measurement_variance", 0.0009);
        use_imu_message_covariance_ = this->declare_parameter<bool>(
            "use_imu_message_covariance", true);
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        left_wheel_velocity_multiplier_ = this->declare_parameter<double>(
            "left_wheel_velocity_multiplier", 1.0);
        right_wheel_velocity_multiplier_ = this->declare_parameter<double>(
            "right_wheel_velocity_multiplier", 1.0);
    }

    void declare_and_load_interface_parameters()
    {
        encoder_input_type_ = this->declare_parameter<std::string>(
            "encoder_input_type", "twist");
        cmd_vel_topic_ = this->declare_parameter<std::string>(
            "cmd_vel_topic", "cmd_vel");
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "imu");
        encoder_twist_topic_ = this->declare_parameter<std::string>(
            "encoder_twist_topic", "enc/twist_meas");
        joint_states_topic_ = this->declare_parameter<std::string>(
            "joint_states_topic", "joint_states");
        odometry_topic_ = this->declare_parameter<std::string>(
            "odometry_topic", "odom");
        odom_frame_id_ = this->declare_parameter<std::string>(
            "odom_frame_id", "odom");
        base_frame_id_ = this->declare_parameter<std::string>(
            "base_frame_id", "base_link");
        left_wheel_joint_ = this->declare_parameter<std::string>(
            "left_wheel_joint", "");
        right_wheel_joint_ = this->declare_parameter<std::string>(
            "right_wheel_joint", "");
    }

    void validate_parameters(
        const std::vector<double>& initial_covariance_diagonal,
        const std::vector<double>& process_noise_diagonal,
        const std::vector<double>& encoder_covariance_diagonal) const
    {
        require_positive("wheel_base", wheel_base_);
        require_positive("radius", radius_);
        require_nonnegative("alpha_v", alpha_v_);
        require_nonnegative("alpha_w", alpha_w_);
        require_positive("nominal_dt", nominal_dt_);
        require_positive("max_dt", max_dt_);
        require_positive("imu_mahalanobis_threshold", imu_gate_);
        require_positive("enc_mahalanobis_threshold", enc_gate_);
        require_positive("prediction_frequency_hz", prediction_frequency_hz_);
        require_positive("imu_measurement_variance", imu_measurement_variance_);

        validate_diagonal(
            "initial_state_covariance_diagonal",
            initial_covariance_diagonal,
            6,
            false);
        validate_diagonal(
            "process_noise_covariance_diagonal",
            process_noise_diagonal,
            6,
            false);
        validate_diagonal(
            "encoder_measurement_covariance_diagonal",
            encoder_covariance_diagonal,
            2,
            true);

        if (encoder_input_type_ != "twist" &&
            encoder_input_type_ != "joint_states")
        {
            throw std::invalid_argument(
                "encoder_input_type must be 'twist' or 'joint_states'");
        }

        require_nonempty("cmd_vel_topic", cmd_vel_topic_);
        require_nonempty("imu_topic", imu_topic_);
        require_nonempty("odometry_topic", odometry_topic_);
        require_nonempty("odom_frame_id", odom_frame_id_);
        require_nonempty("base_frame_id", base_frame_id_);

        if (encoder_input_type_ == "twist") {
            require_nonempty("encoder_twist_topic", encoder_twist_topic_);
        } else {
            require_nonempty("joint_states_topic", joint_states_topic_);
            require_nonempty("left_wheel_joint", left_wheel_joint_);
            require_nonempty("right_wheel_joint", right_wheel_joint_);
            if (left_wheel_joint_ == right_wheel_joint_) {
                throw std::invalid_argument(
                    "left_wheel_joint and right_wheel_joint must be different");
            }
            require_nonzero_finite(
                "left_wheel_velocity_multiplier",
                left_wheel_velocity_multiplier_);
            require_nonzero_finite(
                "right_wheel_velocity_multiplier",
                right_wheel_velocity_multiplier_);
        }
    }

    static void validate_diagonal(
        const std::string& name,
        const std::vector<double>& values,
        std::size_t expected_size,
        bool strictly_positive)
    {
        if (values.size() != expected_size) {
            throw std::invalid_argument(
                name + " must contain exactly " +
                std::to_string(expected_size) + " values");
        }

        for (const double value : values) {
            if (!std::isfinite(value) ||
                (strictly_positive ? value <= 0.0 : value < 0.0))
            {
                throw std::invalid_argument(
                    name + (strictly_positive ?
                        " values must be finite and positive" :
                        " values must be finite and non-negative"));
            }
        }
    }

    static void require_positive(const std::string& name, double value)
    {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(name + " must be finite and positive");
        }
    }

    static void require_nonnegative(const std::string& name, double value)
    {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(
                name + " must be finite and non-negative");
        }
    }

    static void require_nonzero_finite(const std::string& name, double value)
    {
        if (!std::isfinite(value) || value == 0.0) {
            throw std::invalid_argument(name + " must be finite and non-zero");
        }
    }

    static void require_nonempty(
        const std::string& name,
        const std::string& value)
    {
        if (value.empty()) {
            throw std::invalid_argument(name + " must not be empty");
        }
    }

    void predict()
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);

        const rclcpp::Time now_time = this->now();

        predict_to_time(now_time);

        publish_odometry(now_time);
    }

    void predict_to_time(const rclcpp::Time& target_time)
    {
        double dt = (target_time - last_predict_time_).seconds();

        if (dt <= 0.0) {
            return;
        }

        if (dt > max_dt_) {
            last_predict_time_ = target_time;
            return;
        }

        last_predict_time_ = target_time;

        const double theta = x_(2);
        const double v = x_(3);
        const double w = x_(4);

        const double c = std::cos(theta);
        const double s = std::sin(theta);

        x_(0) += v * c * dt;
        x_(1) += v * s * dt;
        x_(2) += w * dt;

        x_(3) += alpha_v_ * (v_ref_ - x_(3)) * dt;
        x_(4) += alpha_w_ * (w_ref_ - x_(4)) * dt;

        normalize_angle(x_(2));

        F_.setIdentity();

        F_(0, 2) = -v * s * dt;
        F_(0, 3) =  c * dt;

        F_(1, 2) =  v * c * dt;
        F_(1, 3) =  s * dt;

        F_(2, 4) = dt;

        F_(3, 3) = 1.0 - alpha_v_ * dt;
        F_(4, 4) = 1.0 - alpha_w_ * dt;

        const double q_scale = dt / nominal_dt_;

        P_ = F_ * P_ * F_.transpose() + q_scale * Q_;

        symmetrize_covariance();
    }

    void reference_reader(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);

        v_ref_ = msg->linear.x;
        w_ref_ = msg->angular.z;
    }

    void imu_correction(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);

        const rclcpp::Time stamp = get_valid_stamp(msg->header.stamp);
        predict_to_time(stamp);

        Matrix1d z;
        z(0, 0) = msg->angular_velocity.z;

        const double imu_cov_z = msg->angular_velocity_covariance[8];
        R_imu_(0, 0) = imu_measurement_variance_;

        if (use_imu_message_covariance_ &&
            std::isfinite(imu_cov_z) &&
            imu_cov_z > 0.0)
        {
            R_imu_(0, 0) = imu_cov_z;
        }

        diff_imu_ = z - H_imu_ * x_;

        S_imu_ =
            H_imu_ *
            P_ *
            H_imu_.transpose()
            +
            R_imu_;

        const double maha =
            diff_imu_(0, 0) *
            (1.0 / S_imu_(0, 0)) *
            diff_imu_(0, 0);

        if (maha > imu_gate_) {
            RCLCPP_WARN_THROTTLE(
                verbal_logger_,
                *this->get_clock(),
                1000,
                "IMU rejected by Mahalanobis gate: %.3f",
                maha
            );
            publish_odometry(stamp);
            return;
        }

        K_imu_ =
            P_ *
            H_imu_.transpose() *
            S_imu_.inverse();

        x_ = x_ + K_imu_ * diff_imu_(0, 0);

        normalize_angle(x_(2));

        const Matrix6d I = Matrix6d::Identity();

        P_ =
            (I - K_imu_ * H_imu_) *
            P_ *
            (I - K_imu_ * H_imu_).transpose()
            +
            K_imu_ *
            R_imu_ *
            K_imu_.transpose();

        symmetrize_covariance();

        publish_odometry(stamp);
    }

    void encoder_twist_reader(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        apply_encoder_measurement(
            msg->twist.linear.x,
            msg->twist.angular.z,
            get_valid_stamp(msg->header.stamp));
    }

    void joint_states_reader(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const auto left_it = std::find(
            msg->name.begin(), msg->name.end(), left_wheel_joint_);
        const auto right_it = std::find(
            msg->name.begin(), msg->name.end(), right_wheel_joint_);

        if (left_it == msg->name.end() || right_it == msg->name.end()) {
            RCLCPP_WARN_THROTTLE(
                verbal_logger_,
                *this->get_clock(),
                1000,
                "JointState on '%s' does not contain wheel joints '%s' and '%s'",
                joint_states_topic_.c_str(),
                left_wheel_joint_.c_str(),
                right_wheel_joint_.c_str());
            return;
        }

        const auto left_index = static_cast<std::size_t>(
            std::distance(msg->name.begin(), left_it));
        const auto right_index = static_cast<std::size_t>(
            std::distance(msg->name.begin(), right_it));

        if (left_index >= msg->velocity.size() ||
            right_index >= msg->velocity.size())
        {
            RCLCPP_WARN_THROTTLE(
                verbal_logger_,
                *this->get_clock(),
                1000,
                "JointState on '%s' has no velocity for both configured wheel joints",
                joint_states_topic_.c_str());
            return;
        }

        const double left_angular_velocity =
            left_wheel_velocity_multiplier_ * msg->velocity[left_index];
        const double right_angular_velocity =
            right_wheel_velocity_multiplier_ * msg->velocity[right_index];

        if (!std::isfinite(left_angular_velocity) ||
            !std::isfinite(right_angular_velocity))
        {
            RCLCPP_WARN_THROTTLE(
                verbal_logger_,
                *this->get_clock(),
                1000,
                "JointState on '%s' contains a non-finite wheel velocity",
                joint_states_topic_.c_str());
            return;
        }

        const double left_linear_velocity = radius_ * left_angular_velocity;
        const double right_linear_velocity = radius_ * right_angular_velocity;
        const double linear_velocity =
            0.5 * (right_linear_velocity + left_linear_velocity);
        const double angular_velocity =
            (right_linear_velocity - left_linear_velocity) / wheel_base_;

        apply_encoder_measurement(
            linear_velocity,
            angular_velocity,
            get_valid_stamp(msg->header.stamp));
    }

    void apply_encoder_measurement(
        double linear_velocity,
        double angular_velocity,
        const rclcpp::Time& stamp)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);

        predict_to_time(stamp);

        Eigen::Matrix<double, 2, 1> z;

        z <<
            linear_velocity,
            angular_velocity;

        diff_enc_ = z - H_enc_ * x_;

        S_enc_ =
            H_enc_ *
            P_ *
            H_enc_.transpose()
            +
            R_enc_;

        const double maha =
            (diff_enc_.transpose() *
             S_enc_.inverse() *
             diff_enc_)(0, 0);

        if (maha > enc_gate_) {
            RCLCPP_WARN_THROTTLE(
                verbal_logger_,
                *this->get_clock(),
                1000,
                "Encoder rejected by Mahalanobis gate: %.3f",
                maha
            );
            publish_odometry(stamp);
            return;
        }

        K_enc_ =
            P_ *
            H_enc_.transpose() *
            S_enc_.inverse();

        x_ = x_ + K_enc_ * diff_enc_;

        normalize_angle(x_(2));

        const Matrix6d I = Matrix6d::Identity();

        P_ =
            (I - K_enc_ * H_enc_) *
            P_ *
            (I - K_enc_ * H_enc_).transpose()
            +
            K_enc_ *
            R_enc_ *
            K_enc_.transpose();

        symmetrize_covariance();

        publish_odometry(stamp);
    }

    void publish_odometry(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = x_(0);
        odom.pose.pose.position.y = x_(1);
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x_(2));
        q.normalize();

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = x_(3);
        odom.twist.twist.angular.z = x_(4);

        fill_odometry_covariance(odom);

        odom_pub_->publish(odom);

        if (!publish_tf_) {
            return;
        }

        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = odom_frame_id_;
        tf_msg.child_frame_id = base_frame_id_;

        tf_msg.transform.translation.x = x_(0);
        tf_msg.transform.translation.y = x_(1);
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void fill_odometry_covariance(nav_msgs::msg::Odometry& odom)
    {
        odom.pose.covariance.fill(0.0);
        odom.twist.covariance.fill(0.0);

        odom.pose.covariance[0]  = P_(0, 0);
        odom.pose.covariance[1]  = P_(0, 1);
        odom.pose.covariance[5]  = P_(0, 2);

        odom.pose.covariance[6]  = P_(1, 0);
        odom.pose.covariance[7]  = P_(1, 1);
        odom.pose.covariance[11] = P_(1, 2);

        odom.pose.covariance[30] = P_(2, 0);
        odom.pose.covariance[31] = P_(2, 1);
        odom.pose.covariance[35] = P_(2, 2);

        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;

        odom.twist.covariance[0]  = P_(3, 3);
        odom.twist.covariance[5]  = P_(3, 4);

        odom.twist.covariance[30] = P_(4, 3);
        odom.twist.covariance[35] = P_(4, 4);

        odom.twist.covariance[7]  = 1e6;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
    }

    rclcpp::Time get_valid_stamp(const builtin_interfaces::msg::Time& stamp_msg)
    {
        rclcpp::Time stamp(stamp_msg);

        if (stamp.nanoseconds() == 0) {
            return this->now();
        }

        return stamp;
    }

    void symmetrize_covariance()
    {
        P_ = 0.5 * (P_ + P_.transpose());
    }

    void normalize_angle(double& angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<Ekf>();

        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("ekf"),
            "Failed to start EKF node: %s",
            error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
