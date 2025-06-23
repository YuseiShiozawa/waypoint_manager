#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>

ros::ServiceClient dynamic_client;
bool obstacle_detected = false;  // 障害物が検出されたかどうか
ros::Time last_obstacle_time;   // 障害物が検出された／消えた時刻
double current_vel_x = -1.0;    // 現在の max_vel_x の設定値（初期化されていないことを示す）

// 正面1点のみチェック（近距離なら止まる）
bool checkFrontObstacle(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int center_index = (0.0 - scan->angle_min) / scan->angle_increment;

    if (center_index < 0 || center_index >= scan->ranges.size())
    {
        ROS_WARN("Center index out of range!");
        return false;
    }

    double front_distance = scan->ranges[center_index];
  //  ROS_INFO("Center range [index %d]: %f", center_index, front_distance);

    return (front_distance < 8.0);  // 1m以内なら障害物と判定
}

// max_vel_x を変更（不要な再設定は避ける）
void setMaxVelX(double value)
{
    if (value == current_vel_x)
        return;  // 同じ値なら再設定しない

    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    double_param.name = "max_vel_x";
    double_param.value = value;
    config.doubles.push_back(double_param);
    req.config = config;

    if (dynamic_client.call(req, res))
    {
 //       ROS_INFO("Max velocity updated successfully to %f", value);
        current_vel_x = value;
    }
    else
    {
        ROS_ERROR("Failed to call service to update velocity.");
    }
}

// LaserScan のコールバック
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool detected = checkFrontObstacle(scan);

    if (detected)
    {
        if (!obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();  // 障害物検出時刻を記録
            obstacle_detected = true;
   //         ROS_INFO("Obstacle detected, stopping!");
            setMaxVelX(0.5);  // 速度制限
        }
    }
    else
    {
        if (obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();  // 障害物が消えた時刻を記録
            obstacle_detected = false;
        }

        if ((ros::Time::now() - last_obstacle_time).toSec() > 7.0)
        {
          //  ROS_INFO("5 seconds passed without obstacle, restoring max_vel_x to 1.0");
            setMaxVelX(1.0);  // 元の速度に戻す
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_front_obstacle_check");
    ros::NodeHandle nh;

    dynamic_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/move_base/TrajectoryPlannerROS/set_parameters");

    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
