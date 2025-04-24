#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class SensorDataRepublisher
{
private:
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher tf_static_pub;
    ros::Publisher diagnostics_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher imu_data_pub;
    ros::Publisher imu_livox_pub;
    ros::Publisher ekf_imu_pub;
    ros::Publisher livox_pub;
    ros::Publisher ekf_odom_pub;
    ros::Publisher odom_pub;
  
    // Subscribers
    ros::Subscriber tf_static_sub;
    ros::Subscriber imu_data_sub;
    ros::Subscriber livox_sub;
    ros::Subscriber ekf_odom_sub;
    ros::Subscriber ekf_imu_sub;
    ros::Subscriber diagnostics_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber imu_livox_sub;

    ros::Rate odom_rate;
    ros::Rate livox_rate;
    ros::Rate imu_rate;
    ros::Rate cmd_vel_rate;
    ros::Rate diagnostics_rate;
    ros::Rate livox_imu_rate;
    ros::Publisher gnss_pose_pub;
     ros::Subscriber gnss_pose_sub;
public:
    SensorDataRepublisher()  : odom_rate(50), livox_rate(10), imu_rate(50), cmd_vel_rate(50),diagnostics_rate(4)
    ,livox_imu_rate(200)

    {
        // Initialize publishers
        tf_static_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 10);
        diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        imu_data_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
        imu_livox_pub = nh.advertise<sensor_msgs::Imu>("/livox/imu", 10);
        ekf_imu_pub = nh.advertise<sensor_msgs::Imu>("/ekf/imu/filtered", 10);
        livox_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 10);
        ekf_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf/odometry/filtered", 10);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
        
        // Initialize subscribers
        tf_static_sub = nh.subscribe("/tf_static_bag", 10, &SensorDataRepublisher::tfStaticCallback, this);
        imu_data_sub = nh.subscribe("/imu/data_bag", 10, &SensorDataRepublisher::imuCallback, this);
        livox_sub = nh.subscribe("/sensing/lidar/top/lslidar_point_cloud_bag", 10, &SensorDataRepublisher::livoxlidarCallback, this);
        ekf_odom_sub = nh.subscribe("/ekf/odometry/filtered_bag", 10, &SensorDataRepublisher::ekfOdomCallback, this);
        ekf_imu_sub = nh.subscribe("/ekf/imu/filtered_bag", 10, &SensorDataRepublisher::ekfIMUCallback, this);
        diagnostics_sub = nh.subscribe("/diagnostics_bag", 10, &SensorDataRepublisher::diagnosticsCallback, this);
        odom_sub = nh.subscribe("/odom_bag", 10, &SensorDataRepublisher::odomCallback, this);
        cmd_vel_sub = nh.subscribe("/cmd_vel_bag", 10, &SensorDataRepublisher::cmdCallback, this);
        imu_livox_sub = nh.subscribe("/livox/imu_bag", 10, &SensorDataRepublisher::livoxIMUCallback, this);
       
         // GNSS
         gnss_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 10);
         gnss_pose_sub = nh.subscribe("/sensing/gnss/pose_with_covariance_bag", 10, &SensorDataRepublisher::gnssCallback, this);
 

        ROS_INFO("Sensor Data Republisher Started...");
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        nav_msgs::Odometry new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); 
        odom_pub.publish(new_msg);
        
        
    }
    void gnssCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        geometry_msgs::PoseWithCovarianceStamped new_msg = *msg;
        new_msg.header.stamp = ros::Time::now();
        gnss_pose_pub.publish(new_msg);
    }
    void livoxIMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        sensor_msgs::Imu new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); // Update timestamp to current time
        imu_livox_pub.publish(new_msg);
        
    }
    // Callback for static TF data
    void tfStaticCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
    {
        tf2_msgs::TFMessage new_msg = *msg;
        ros::Time current_time = ros::Time::now();
        for (auto &transform : new_msg.transforms)
        {
            transform.header.stamp = current_time; 
        }
        tf_static_pub.publish(new_msg);
    }

    // Callback for IMU data
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        sensor_msgs::Imu new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); // Update timestamp to current time
        imu_data_pub.publish(new_msg);
        
    }

    // Callback for LiDAR data
    void livoxlidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        sensor_msgs::PointCloud2 new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); // Set timestamp to current time

        livox_pub.publish(new_msg);
       
    }

    // Callback for EKF odometry data
    void ekfOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        nav_msgs::Odometry new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); // Set timestamp to current time
        ekf_odom_pub.publish(new_msg);
        odom_rate.sleep();
    }

    // Callback for EKF IMU data
    void ekfIMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        sensor_msgs::Imu new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); // Set timestamp to current time
        ekf_imu_pub.publish(new_msg);
        
    }

    // Callback for diagnostics data
    void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr &msg)
    {
        diagnostic_msgs::DiagnosticArray new_msg = *msg;
        new_msg.header.stamp = ros::Time::now(); // Set timestamp to current time
        diagnostics_pub.publish(new_msg);

    }

    // Callback for command velocity data
    void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        geometry_msgs::Twist new_msg = *msg;
        new_msg.linear.x = 0.5; // Example: modify cmd_vel if needed
        cmd_vel_pub.publish(new_msg);
        
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "real_topic");
    SensorDataRepublisher republisher;
    ros::spin(); // Keeps the program running and handling callbacks
    return 0;
}
