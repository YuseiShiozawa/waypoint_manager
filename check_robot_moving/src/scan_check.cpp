#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>

ros::ServiceClient dynamic_client;
bool obstacle_detected = false;  // 障害物が検出されたかどうか
ros::Time last_obstacle_time;  // 障害物検出時間

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
    if (front_distance < 30.0)
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
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    obstacle_detected = checkFrontObstacle(scan);

    if (obstacle_detected)
    {
        // 障害物を検出した時間を記録
        if (!obstacle_detected) {
            last_obstacle_time = ros::Time::now();
        }

        ROS_INFO("Obstacle detected, stopping!");
        // 停止のために速度を0に設定
        double_param.name = "max_vel_x";
        double_param.value = 0.3;  // 停止
        config.doubles.push_back(double_param);
        req.config = config;

        if (dynamic_client.call(req, res))
        {
            ROS_INFO("Max velocity updated successfully to %f", double_param.value);
        }
        else
        {
            ROS_ERROR("Failed to call service to update velocity.");
        }
    }
    else
    {
        // 障害物がない場合、5秒経過していたら速度を1.0に戻す
        if ((ros::Time::now() - last_obstacle_time).toSec() > 5.0)
        {
            ROS_INFO("5 seconds passed, restoring max_vel_x to 1.0");
            double_param.name = "max_vel_x";
            double_param.value = 1.0;  // 通常の速度
            config.doubles.push_back(double_param);
            req.config = config;

            if (dynamic_client.call(req, res))
            {
                ROS_INFO("Max velocity updated successfully to %f", double_param.value);
            }
            else
            {
                ROS_ERROR("Failed to call service to update velocity.");
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_front_obstacle_check");
    ros::NodeHandle nh;

    // 動的パラメータのサービスクライアントを作成
    dynamic_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TrajectoryPlannerROS/set_parameters");

    // センサーデータの購読
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
