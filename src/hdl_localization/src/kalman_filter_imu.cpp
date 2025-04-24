#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>
#include <cmath>

class SimpleImuKalmanFilter {
private:
    // State vector: [orientation(quaternion), angular_velocity, acceleration]
    Eigen::Matrix<double, 10, 1> x_;
    
    // The 3 main parameters of Kalman Filter
    Eigen::Matrix<double, 10, 10> P_;  // State covariance
    Eigen::Matrix<double, 10, 10> Q_;  // Process noise covariance
    Eigen::Matrix<double, 6, 6> R_;    // Measurement noise covariance
    
    ros::Time last_time_;
    ros::Subscriber imu_sub_;
    ros::Publisher filtered_imu_pub_;
    ros::ServiceServer reset_service_;
    bool initialized_ = false;
    
    // Filter parameters (simplified to 3 main scalars)
    double process_noise_;     // Controls how much we trust the process model
    double measurement_noise_; // Controls how much we trust the measurements
    double initial_covariance_; // Initial uncertainty in our state estimate

public:
    SimpleImuKalmanFilter(ros::NodeHandle& nh) {
        // Load the 3 main parameters
        nh.param("process_noise", process_noise_, 0.01);
        nh.param("measurement_noise", measurement_noise_, 0.1);
        nh.param("initial_covariance", initial_covariance_, 0.01);
        
        // Initialize state vector (quaternion is identity, others are zero)
        x_.setZero();
        x_(0) = 1.0;  // w component of quaternion
        
        // Initialize state covariance (P) - our confidence in initial state
        P_.setIdentity();
        P_ *= initial_covariance_;
        
        // Initialize process noise covariance (Q) - how much we expect state to change randomly
        Q_.setIdentity();
        Q_ *= process_noise_;
        
        // Initialize measurement noise covariance (R) - how much we trust sensor readings
        R_.setIdentity();
        R_ *= measurement_noise_;
        
        // Set up ROS communication
        imu_sub_ = nh.subscribe("/livox/imu", 10, &SimpleImuKalmanFilter::imuCallback, this);
        filtered_imu_pub_ = nh.advertise<sensor_msgs::Imu>("/filtered_imu", 10);
        reset_service_ = nh.advertiseService("reset_imu_filter", &SimpleImuKalmanFilter::resetFilter, this);
        
        ROS_INFO("Simple IMU Kalman Filter initialized with process_noise=%.3f, measurement_noise=%.3f", 
                 process_noise_, measurement_noise_);
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!initialized_) {
            // Initialize state with first measurement
            x_(0) = msg->orientation.w;
            x_(1) = msg->orientation.x;
            x_(2) = msg->orientation.y;
            x_(3) = msg->orientation.z;
            x_(4) = msg->angular_velocity.x;
            x_(5) = msg->angular_velocity.y;
            x_(6) = msg->angular_velocity.z;
            x_(7) = msg->linear_acceleration.x;
            x_(8) = msg->linear_acceleration.y;
            x_(9) = msg->linear_acceleration.z;
            
            normalizeQuaternion();
            last_time_ = msg->header.stamp;
            initialized_ = true;
            return;
        }
        
        // Calculate time delta
        double dt = (msg->header.stamp - last_time_).toSec();
        if (dt <= 0.0 || dt > 0.1) {  // Sanity check
            last_time_ = msg->header.stamp;
            return;
        }
        
        // Prediction step
        predict(dt);
        
        // Measurement vector: [angular_velocity, acceleration]
        Eigen::Matrix<double, 6, 1> z;
        z << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        
        // Update step
        update(z);
        
        // Normalize quaternion
        normalizeQuaternion();
        
        // Publish filtered IMU message
        publishFiltered(msg);
        
        // Update timestamp
        last_time_ = msg->header.stamp;
    }
    
    void predict(double dt) {
        // Extract quaternion and angular velocity components
        double qw = x_(0), qx = x_(1), qy = x_(2), qz = x_(3);
        double wx = x_(4), wy = x_(5), wz = x_(6);
        
        // Update quaternion with current angular velocity
        // This is the quaternion differential equation: q_dot = 0.5 * q ⊗ [0, ω]
        x_(0) += 0.5 * dt * (-qx*wx - qy*wy - qz*wz);
        x_(1) += 0.5 * dt * (qw*wx + qy*wz - qz*wy);
        x_(2) += 0.5 * dt * (qw*wy - qx*wz + qz*wx);
        x_(3) += 0.5 * dt * (qw*wz + qx*wy - qy*wx);
        
        // Angular velocity and acceleration remain constant in the prediction model
        
        // Create simplified state transition matrix
        Eigen::Matrix<double, 10, 10> F = Eigen::Matrix<double, 10, 10>::Identity();
        
        // Update state covariance
        P_ = F * P_ * F.transpose() + Q_ * dt;
    }
    
    void update(const Eigen::Matrix<double, 6, 1>& z) {
        // Measurement model matrix H maps state to measurement (angular_vel and accel)
        Eigen::Matrix<double, 6, 10> H = Eigen::Matrix<double, 6, 10>::Zero();
        
        // Angular velocity measurement
        H(0, 4) = 1.0;  // wx
        H(1, 5) = 1.0;  // wy
        H(2, 6) = 1.0;  // wz
        
        // Acceleration measurement
        H(3, 7) = 1.0;  // ax
        H(4, 8) = 1.0;  // ay
        H(5, 9) = 1.0;  // az
        
        // Expected measurement from current state
        Eigen::Matrix<double, 6, 1> z_pred;
        z_pred(0) = x_(4);  // wx
        z_pred(1) = x_(5);  // wy
        z_pred(2) = x_(6);  // wz
        z_pred(3) = x_(7);  // ax
        z_pred(4) = x_(8);  // ay
        z_pred(5) = x_(9);  // az
        
        // Innovation (measurement residual)
        Eigen::Matrix<double, 6, 1> y = z - z_pred;
        
        // Innovation covariance
        Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_;
        
        // Kalman gain
        Eigen::Matrix<double, 10, 6> K = P_ * H.transpose() * S.inverse();
        
        // Update state
        x_ = x_ + K * y;
        
        // Update covariance
        Eigen::Matrix<double, 10, 10> I = Eigen::Matrix<double, 10, 10>::Identity();
        P_ = (I - K * H) * P_;
        
        // Print debug info occasionally
        ROS_DEBUG_THROTTLE(1.0, "Raw angular_x: %.4f, Filtered: %.4f", z(0), x_(4));
    }
    
    void normalizeQuaternion() {
        double norm = std::sqrt(x_.segment<4>(0).squaredNorm());
        if (norm < 1e-10) {
            x_(0) = 1.0; x_(1) = x_(2) = x_(3) = 0.0;
        } else {
            x_.segment<4>(0) /= norm;
        }
    }
    
    void publishFiltered(const sensor_msgs::Imu::ConstPtr& original_msg) {
        sensor_msgs::Imu filtered_msg;
        
        // Copy header
        filtered_msg.header = original_msg->header;
        
        // Set orientation from state
        filtered_msg.orientation.w = x_(0);
        filtered_msg.orientation.x = x_(1);
        filtered_msg.orientation.y = x_(2);
        filtered_msg.orientation.z = x_(3);
        
        // Set angular velocity from state
        filtered_msg.angular_velocity.x = x_(4);
        filtered_msg.angular_velocity.y = x_(5);
        filtered_msg.angular_velocity.z = x_(6);
        
        // Set acceleration from state
        filtered_msg.linear_acceleration.x = x_(7);
        filtered_msg.linear_acceleration.y = x_(8);
        filtered_msg.linear_acceleration.z = x_(9);
        
        // Set covariances (optional)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                filtered_msg.orientation_covariance[3*i+j] = P_(i+1,j+1);
                filtered_msg.angular_velocity_covariance[3*i+j] = P_(i+4,j+4);
                filtered_msg.linear_acceleration_covariance[3*i+j] = P_(i+7,j+7);
            }
        }
        
        // Publish filtered message
        filtered_imu_pub_.publish(filtered_msg);
    }
    
    bool resetFilter(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        // Reset state and covariance
        x_.setZero();
        x_(0) = 1.0;  // Identity quaternion
        P_.setIdentity();
        P_ *= initial_covariance_;
        
        initialized_ = false;
        ROS_INFO("IMU filter has been reset");
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_imu_filter_node");
    ros::NodeHandle nh("~");  // Private namespace for parameters
    
    SimpleImuKalmanFilter filter(nh);
    
    ros::spin();
    return 0;
}