#include "rclcpp/rclcpp.hpp"
#include "afv_dwa.hpp"
using std::placeholders::_1;

namespace afv_dwa
{

    DWA::DWA(const rclcpp::NodeOptions &options) : Node("afv_dwa", options)
    {
        declare_parameter("occupancy_grid_topic","occupancy_grid");
        declare_parameter("odom_topic","odom");
        declare_parameter("cmd_vel_topic","cmd_vel");
        declare_parameter("current_pose_topic","current_pose");
        declare_parameter("predicted_path_topic","predicted_path");
        declare_parameter("goal_pose_topic","goal_pose");
        const auto occupancy_grid_topic = get_parameter("occupancy_grid_topic").as_string();
        const auto odom_topic = get_parameter("odom_topic").as_string();
        const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
        const auto current_pose_topic = get_parameter("current_pose_topic").as_string();
        const auto predicted_path_topic = get_parameter("predicted_path_topic").as_string();
        const auto goal_pose_topic = get_parameter("goal_pose_topic").as_string();

        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(occupancy_grid_topic, 10, std::bind(&DWA::obstacleCallback, this, _1));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&DWA::odomCallback, this, _1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_topic, 10);
        predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(predicted_path_topic, 10);
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10, std::bind(&DWA::goalCallback, this, _1));
        // Initialize params
        initParameter();
        // Initialize current state
        current_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};

        goal_ = std::make_pair(-2.0, -0.5);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DWA::dwaControl, this));
    }

    void DWA::obstacleCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        obstacles_.clear();
        double resolution = msg->info.resolution;      //マップの解像度
        double origin_x = msg->info.origin.position.x; //マップの原点x
        double origin_y = msg->info.origin.position.y; //マップの原点y
        for (int i = 0; i < msg->data.size(); i++)
        {
            // Get the cell value
            int cell_value = msg->data[i];
            // Check if the cell is occupied
            if (cell_value == 100)
            {
                int x = i % msg->info.width;
                int y = i / msg->info.width;
                double world_x = (x * resolution) + origin_x;
                double world_y = (y * resolution) + origin_y;
                obstacles_.push_back(std::make_pair(world_x, world_y));
            }
        }
    }

    void DWA::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update position
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        // Update velocity
        current_state_.v = msg->twist.twist.linear.x;
        current_state_.omega = msg->twist.twist.angular.z;
        // Convert quaternion to euler
        geometry_msgs::msg::Quaternion quat = msg->pose.pose.orientation;
        double roll, pitch, yaw = 0.0;
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_state_.theta = yaw;
    }

    void DWA::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Update goal position
        goal_.first = msg->pose.position.x;
        goal_.second = msg->pose.position.y;
        //RCLCPP_INFO(this->get_logger(), "Updated goal position: x = %f, y = %f", goal_.first, goal_.second);
    }

    void DWA::initParameter(void)
    {
        param_.max_speed = this->declare_parameter("max_speed", 0.5);
        param_.min_speed = this->declare_parameter("min_speed", -0.0);
        param_.max_omega = this->declare_parameter("max_omega", 1.0);
        param_.max_accel = this->declare_parameter("max_accel", 3.0);
        param_.max_accel_omega = this->declare_parameter("max_accel_omega", 30.0 * M_PI / 180.0);
        param_.v_resolution = this->declare_parameter("v_resolution", 0.01);
        param_.omega_resolution = this->declare_parameter("omega_resolution", 0.1 * M_PI / 180.0);
        param_.predict_time = this->declare_parameter("predict_time", 4.0);
        param_.dt = this->declare_parameter("dt", 0.01);
        param_.goal_cost_gain = this->declare_parameter("goal_cost_gain", 0.3);
        param_.speed_cost_gain = this->declare_parameter("speed_cost_gain", 1.5);
        param_.obstacle_cost_gain = this->declare_parameter("obstacle_cost_gain", 0.1);
        param_.robot_radius = this->declare_parameter("robot_radius", 0.11);
        param_.goal_tolerance = this->declare_parameter("goal_tolerance", 0.2);
        frame_id_ = this->declare_parameter("frame_id_", "odom");
    }

    void DWA::updateParameter(void)
    {
        auto update_param = [&](const std::string &name)
        {
            return this->get_parameter(name).get_value<double>();
        };
        auto update_param_log = [&](const std::string &name, double &variable)
        {
            variable = update_param(name);
            // RCLCPP_INFO(this->get_logger(), "Updated %s: %f", name.c_str(), variable);
        };
        update_param_log("max_speed", param_.max_speed);
        update_param_log("min_speed", param_.min_speed);
        update_param_log("max_omega", param_.max_omega);
        update_param_log("max_accel", param_.max_accel);
        update_param_log("max_accel_omega", param_.max_accel_omega);
        update_param_log("v_resolution", param_.v_resolution);            
        update_param_log("omega_resolution", param_.omega_resolution);
        update_param_log("dt", param_.dt);
        update_param_log("goal_cost_gain", param_.goal_cost_gain);
        update_param_log("speed_cost_gain", param_.speed_cost_gain);
        update_param_log("obstacle_cost_gain", param_.obstacle_cost_gain);
    }



    std::vector<State> DWA::predictTrajectory(double v, double omega)
    {
        std::vector<State> predicted_states;
        State current_state = current_state_;
        predicted_states.push_back(current_state);
        double predict_time = param_.predict_time;
        double dt = param_.dt;
        double time = dt;
        while (time < predict_time)
        {
            //作動2輪ロボットの運動方程式
            //ロボットによってここを書き換える
            current_state.x += current_state.v * cos(current_state.theta) * dt;
            current_state.y += current_state.v * sin(current_state.theta) * dt;
            current_state.theta += current_state.omega * dt;
            current_state.v = v;
            current_state.omega = omega;
            predicted_states.push_back(current_state);
            time += dt;
        }
        return predicted_states;
    }

    //#############################################################################################
    // 評価関数 : G(v,ω)=σ(α⋅heading(v,ω)+β⋅dist(v,ω)+γ⋅velocity(v,ω))
    // heading  : ロボットがゴール方向を向いているかを評価(どれだけゴールの姿勢に近づけているか)
    // dist     : (v,ω)で移動したときの最近傍の障害物までの距離(障害物からどれだけ離れているか)
    // velocity : 速度の評価(どれだけ早いスピードで動けるか)
    // α,β,γ は重み付けパラメーター
    //#############################################################################################
    double DWA::calcGoalCost(std::vector<State> trajectory)
    {
        // Get the final state
        State final_state = trajectory.back();
        // Calculate the angle between the robot's position and the goal
        double dx = goal_.first - final_state.x;
        double dy = goal_.second - final_state.y;
        double target_angle = std::atan2(dy, dx);
        // Calculate the angle difference between the robot's heading and the target angle
        double angle_diff = target_angle - final_state.theta;
        double cost = std::abs(atan2(sin(angle_diff), cos(angle_diff)));
        return cost;
    }

    double DWA::calcObstacleCost(std::vector<State> trajectory)
    {
        double min_distance = std::numeric_limits<double>::max();
        if (obstacles_.empty())
        {
            return 1.0;
        }
        for (auto &state : trajectory)
        {
            for (auto &obstacle : obstacles_)
            {
                double distance = std::sqrt(std::pow(state.x - obstacle.first, 2) + std::pow(state.y - obstacle.second, 2));
                if (distance < min_distance) min_distance = distance;
            }
        }
        double cost = (min_distance > param_.robot_radius) ? 1.0 / min_distance : std::numeric_limits<double>::max();
        RCLCPP_INFO(this->get_logger(), "Min Distance from Obstacle: %lf", min_distance);
        RCLCPP_INFO(this->get_logger(), "Obstacle Cost: %lf", cost);
        return cost;
    }
    double DWA::calcSpeedCost(std::vector<State> trajectory)
    {
        double cost = param_.max_speed - trajectory.back().v;
        return cost;
    }


    void DWA::publishTwist(double v, double omega)
    {
        // If you reach the goal, stop.
        if (isArrivedAtGoal())
        {
            v = 0.0;
            omega = 0.0;
        }
        auto twist = std::make_unique<geometry_msgs::msg::Twist>();
        twist->linear.x = v;
        twist->angular.z = omega;
        twist_pub_->publish(std::move(twist));
    }

    void DWA::publishCurrentPose(void)
    {
        auto current_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
        current_pose->header.stamp = this->now();
        current_pose->header.frame_id = frame_id_;
        current_pose->pose.position.x = current_state_.x;
        current_pose->pose.position.y = current_state_.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_state_.theta);
        current_pose->pose.orientation = tf2::toMsg(q);
        current_pose_pub_->publish(std::move(current_pose));
    }

    void DWA::publishPath(std::vector<State> &trajectory)
    {
        auto path = std::make_unique<nav_msgs::msg::Path>();
        for (const auto &state : trajectory)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = state.x;
            pose.pose.position.y = state.y;
            tf2::Quaternion q;
            q.setRPY(0, 0, state.theta);
            pose.pose.orientation = tf2::toMsg(q);
            pose.header.stamp = this->now();
            pose.header.frame_id = frame_id_;
            path->poses.push_back(pose);
        }
        path->header.frame_id = frame_id_;
        path->header.stamp = this->now();
        predicted_path_pub_->publish(std::move(path));
    }

    bool DWA::isArrivedAtGoal(void)
    {
        double distance = std::sqrt(std::pow(current_state_.x - goal_.first, 2) + std::pow(current_state_.y - goal_.second, 2));
        return distance <= param_.goal_tolerance ? true : false;
    }

    void DWA::dwaControl()
    {
        updateParameter();

        //直線速度と角速度の条件を変更し、複数の予測パスを作成する        
        std::vector<double> dw = calcDynamicWindow();
        double best_cost = std::numeric_limits<double>::infinity();
        std::vector<State> best_trajectory;
        for (double v = dw[0]; v <= dw[1]; v += param_.v_resolution)
        {
            for (double omega = dw[2]; omega <= dw[3]; omega += param_.omega_resolution)
            {
                std::vector<State> trajectory = predictTrajectory(v, omega);
                double goal_cost = calcGoalCost(trajectory);
                double speed_cost = calcSpeedCost(trajectory);
                double obstacle_cost = calcObstacleCost(trajectory);
                double cost = param_.goal_cost_gain * goal_cost + param_.speed_cost_gain * speed_cost + param_.obstacle_cost_gain * obstacle_cost;
                //最も良いコースを選択
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_trajectory = trajectory;
                }
            }
        }

        // Get the final linear and angular velocities
        double v = best_trajectory.back().v;
        double omega = best_trajectory.back().omega;

        // Publish the twist message
        publishTwist(v, omega);

        // Publish the current robot pose
        publishCurrentPose();

        // Publish the current best trajectory
        publishPath(best_trajectory);
    }

    /**/
    std::vector<double> DWA::calcDynamicWindow(void)
    {
        std::vector<double> dw;
        double v = current_state_.v;
        double omega = current_state_.omega;
        double max_v = param_.max_speed;
        double min_v = param_.min_speed;
        double max_omega = param_.max_omega;
        double min_omega = -1 * param_.max_omega;
        double max_a = param_.max_accel;
        double max_a_omega = param_.max_accel_omega;
        double v_min = std::max(min_v, v - max_a * param_.dt);
        double v_max = std::min(max_v, v + max_a * param_.dt);
        double omega_min = std::max(min_omega, omega - max_a_omega * param_.dt);
        double omega_max = std::min(max_omega, omega + max_a_omega * param_.dt);
        dw.push_back(v_min);
        dw.push_back(v_max);
        dw.push_back(omega_min);
        dw.push_back(omega_max);
        return dw;
    }
} // namespace afv_dwa

// Components
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(afv_dwa::DWA)
