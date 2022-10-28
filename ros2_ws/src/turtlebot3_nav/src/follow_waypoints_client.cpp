#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_follower_action_client_;

    explicit Nav2Client() : Node("follow_waypoints_client")
    {
        this->waypoint_follower_action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    }

    void sendGoal(void)
    {
        while (!this->waypoint_follower_action_client_->wait_for_action_server())
        {
            RCLCPP_INFO(get_logger(), "Waiting for action server...");
        }

        auto goal_msg = FollowWaypoints::Goal();
        std::vector<geometry_msgs::msg::PoseStamped> poses;

        for (int i = 0; i < 10; i++)
        {
            geometry_msgs::msg::PoseStamped pose;

            pose.header.stamp = this->now();
            pose.header.frame_id = "map";

            pose.pose.position.x = 120 + i;
            pose.pose.position.y = 7;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.pose.orientation.z = 0.0;

            poses.push_back(pose);
        }

        goal_msg.poses = poses;

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);

        waypoint_follower_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedbackCallback(GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Current waypoint = %d", feedback->current_waypoint);
    }

    void resultCallback(const GoalHandleFollowWaypoints::WrappedResult &wrapped_result)
    {
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        if (wrapped_result.result->missed_waypoints.size() > 0)
        {
            RCLCPP_WARN(get_logger(), "Missed %ld waypoints", (wrapped_result.result->missed_waypoints.size()));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Client>();
    node->sendGoal();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
