#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h> // Emptyサービス用

// 動的パラメータの設定用
dynamic_reconfigure::ReconfigureRequest req;
dynamic_reconfigure::ReconfigureResponse res;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config config;

ros::ServiceClient dynamic_client;
costmap_2d::Costmap2DROS* costmap_ros_;  // コストマップ

// 障害物を検出した場合、ロボットの最大速度を減少させる
void reduceSpeed(double new_speed)
{
    // 動的パラメータの設定
    double_param.name = "max_vel_x";
    double_param.value = 0.0;  // 速度を新しい値に設定

    // 設定を構成に追加
    config.doubles.clear();  // 既存の設定をクリア
    config.doubles.push_back(double_param);
    req.config = config;

    // サービスを呼び出して、パラメータを変更
    dynamic_client.call(req, res); // 正しくリクエストとレスポンスを指定
    ROS_INFO("Obstacle detected, reducing speed.");
}

// 障害物がない場合、速度制限を元に戻す
void restoreSpeed(double original_speed)
{
    double_param.value = original_speed;

    config.doubles.clear();
    config.doubles.push_back(double_param);
    req.config = config;

    dynamic_client.call(req, res); // 正しくリクエストとレスポンスを指定
    ROS_INFO("No obstacle detected, restoring speed.");
}

// コストマップを監視し、障害物が近くにあるか確認
void monitorCostmap()
{
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();  // コストマップの取得

    bool obstacle_detected = false;
    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();

    // コストマップを走査して、近くに障害物があるか確認
    for (unsigned int x = 0; x < size_x; ++x)
    {
        for (unsigned int y = 0; y < size_y; ++y)
        {
            unsigned char cost = costmap->getCost(x, y);
            // 近くに致命的な障害物（LETHAL_OBSTACLE）がある場合
            if (cost >= costmap_2d::LETHAL_OBSTACLE)
            {
                obstacle_detected = true;
                break;
            }
        }
        if (obstacle_detected)
        {
            break;
        }
    }

    // 障害物を検出した場合、速度制限を減少
    if (obstacle_detected)
    {
        reduceSpeed(0.0);  // 障害物があれば停止
    }
    else
    {
        restoreSpeed(0.5);  // 障害物がなければ通常速度に戻す
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_robot_moving_node");

    ros::NodeHandle nh;

    // サービスクライアントの作成
    dynamic_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TrajectoryPlannerROS/set_parameters");

    // tf2_ros::Bufferを作成
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // コストマップROSの初期化
    costmap_ros_ = new costmap_2d::Costmap2DROS("move_base", tf_buffer);  // tf2_ros::Bufferを使用

    ros::Rate rate(10);  // 10 Hzでコストマップを監視
    while (ros::ok())
    {
        monitorCostmap();  // コストマップを監視し、障害物があれば速度制限を変更
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
