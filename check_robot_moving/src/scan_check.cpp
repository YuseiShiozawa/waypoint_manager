#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>

ros::ServiceClient dynamic_client;
bool obstacle_detected = false;
ros::Time last_obstacle_time;
double current_vel_x = -1.0;

// 強制停止フラグ
bool force_stop = false;

// === front obstacle check ===
bool checkFrontObstacle(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int center_index = (0.0 - scan->angle_min) / scan->angle_increment;
    if (center_index < 0 || center_index >= scan->ranges.size())
    {
        ROS_WARN("Center index out of range!");
        return false;
    }

    double front_distance = scan->ranges[center_index];
    return (front_distance < 8.0);
}

// === max_vel_x setter ===
void setMaxVelX(double value)
{
    if (value == current_vel_x)
        return;

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
        current_vel_x = value;
        ROS_INFO("Set max_vel_x to %f", value);
    }
    else
    {
        ROS_ERROR("Failed to set max_vel_x");
    }
}

// === LaserScan callback ===
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool detected = checkFrontObstacle(scan);

    if (force_stop)
    {
        if (detected)
        {
            // 強制停止中で、前に障害物が近い → 完全停止
            setMaxVelX(0.0);
        }
        else
        {
            // 強制停止中でも、まだ障害物が遠い → 減速だけ維持 or 無視
            setMaxVelX(1.0);  // ← ここで止まりたくなければコメントアウトして維持も可
        }
        return;
    }

    if (detected)
    {
        if (!obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();
            obstacle_detected = true;
            setMaxVelX(0.5);
        }
    }
    else
    {
        if (obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();
            obstacle_detected = false;
        }

        if ((ros::Time::now() - last_obstacle_time).toSec() > 7.0)
        {
            setMaxVelX(1.0);
        }
    }
}


// === /force_stop service callback ===
bool forceStopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    force_stop = true;
    setMaxVelX(0.0);
    res.success = true;
    res.message = "Force stop activated.";
    return true;
}

// === /force_resume service callback ===
bool forceResumeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    force_stop = false;
    res.success = true;
    res.message = "Force stop released. Resuming normal control.";
    return true;
}

// === main ===
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_front_obstacle_check");
    ros::NodeHandle nh;

    dynamic_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/move_base/TrajectoryPlannerROS/set_parameters");

    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::ServiceServer stop_srv = nh.advertiseService("/force_stop", forceStopCallback);
    ros::ServiceServer resume_srv = nh.advertiseService("/force_resume", forceResumeCallback);

    ros::spin();
    return 0;
}
