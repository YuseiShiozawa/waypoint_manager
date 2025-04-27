#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel_pub;

// 真正面の1本だけチェック（1m以内なら止まる）
bool checkFrontObstacle(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int center_index = (0.0 - scan->angle_min) / scan->angle_increment;  // 0度（真正面）のindex

    // 範囲外を防ぐためにチェック
    if (center_index < 0 || center_index >= scan->ranges.size())
    {
        ROS_WARN("Center index out of range!");
        return false;
    }

    double front_distance = scan->ranges[center_index];
    ROS_INFO("Center range [index %d]: %f", center_index, front_distance);  // デバッグ表示

    // 1.0m以内に障害物があるか？
    if (front_distance < 1.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    geometry_msgs::Twist cmd_vel;

    if (checkFrontObstacle(scan))
    {
        ROS_INFO("Obstacle detected within 1m in front! Stopping.");
        cmd_vel.linear.x = 0.0;  // 止まる
    }
    else
    {
        ROS_INFO("No obstacle within 1m. Moving forward.");
        cmd_vel.linear.x = 0.5;  // 通常の速度で前進
    }

    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_front_obstacle_check");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
