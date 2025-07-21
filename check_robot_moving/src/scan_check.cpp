#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>

// === グローバル変数 ===
ros::ServiceClient dynamic_client;
bool obstacle_detected = false;
ros::Time last_obstacle_time;
double current_vel_x = -1.0;

bool force_stop = false;

// しきい値（パラメータで読み込む）
double slow_threshold = 8.0;
double stop_threshold = 5.0;
int front_check_angle_deg = 5;

// === front obstacle check ===
// 与えられたしきい値以下なら true（障害物あり）
bool checkFrontObstacle(const sensor_msgs::LaserScan::ConstPtr& scan, double threshold, int angle_deg)
{
    int center_index = (0.0 - scan->angle_min) / scan->angle_increment;
    int width = angle_deg / (scan->angle_increment * 180.0 / M_PI);

    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = center_index - width; i <= center_index + width; ++i)
    {
        if (i >= 0 && i < scan->ranges.size())
        {
            double d = scan->ranges[i];
            if (std::isfinite(d))
            {
                min_distance = std::min(min_distance, d);
            }
        }
    }

    return (min_distance < threshold);
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
    if (force_stop)
    {
        // 停止モード：しきい値は stop_threshold
        bool detected = checkFrontObstacle(scan, stop_threshold, front_check_angle_deg);

        if (detected)
        {
            setMaxVelX(0.0);  // 完全停止
        }
        else
        {
            setMaxVelX(0.7);  // 必要に応じてこの動作はコメントアウトしてもよい
        }
        return;
    }

    // 通常（減速）モード：しきい値は slow_threshold
    bool detected = checkFrontObstacle(scan, slow_threshold, front_check_angle_deg);

    if (detected)
    {
        if (!obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();
            obstacle_detected = true;
            setMaxVelX(0.5);  // 減速
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
            setMaxVelX(1.0);  // 通常速度へ戻す
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
    ros::NodeHandle pnh("~");
    // パラメータの読み込み
    pnh.param("slow_threshold", slow_threshold, 8.0);
    pnh.param("stop_threshold", stop_threshold, 5.0);
    pnh.param("front_check_angle_deg", front_check_angle_deg, 5);


    dynamic_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/move_base/TrajectoryPlannerROS/set_parameters");

    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::ServiceServer stop_srv = nh.advertiseService("/force_stop", forceStopCallback);
    ros::ServiceServer resume_srv = nh.advertiseService("/force_resume", forceResumeCallback);

    ros::spin();
    return 0;
}
