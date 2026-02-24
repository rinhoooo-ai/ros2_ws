#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include "table_tennis_gazebo/action/spawn_ball.hpp"

#include <chrono>
#include <sstream>
#include <fstream>
#include <cmath>
#include <gz/transport.hh>
#include <gz/msgs/pose_v.pb.h>
#include <string>
#include <memory>

using namespace std::placeholders;
using SpawnBall = table_tennis_gazebo::action::SpawnBall;
using GoalHandleSpawnBall = rclcpp_action::ServerGoalHandle<SpawnBall>;

class BallSpawnerActionServer : public rclcpp::Node
{
public:
  BallSpawnerActionServer() : Node("ball_spawner_action_server")
  {
    // Create action server
    action_server_ = rclcpp_action::create_server<SpawnBall>(
      this,
      "spawn_ball",
      std::bind(&BallSpawnerActionServer::handle_goal, this, _1, _2),
      std::bind(&BallSpawnerActionServer::handle_cancel, this, _1),
      std::bind(&BallSpawnerActionServer::handle_accepted, this, _1));

    // Create Gazebo service clients
    spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(
      "/world/arena/create");
    delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(
      "/world/arena/remove");

    // Publisher for ball status
    ball_status_pub_ = this->create_publisher<std_msgs::msg::String>("/ball/status", 10);
    ball_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ball/pose", 10);

    // Initialize velocity tracking
    current_velocity_magnitude_ = 0.0;
    last_pose_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Ball spawner action server initialized");
  }

private:
  rclcpp_action::Server<SpawnBall>::SharedPtr action_server_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ball_pose_pub_;

  // Gazebo Transport for direct pose subscription
  gz::transport::Node gz_node_;
  
  geometry_msgs::msg::Pose current_ball_pose_;
  geometry_msgs::msg::Pose previous_ball_pose_;
  rclcpp::Time last_pose_update_time_;
  double current_velocity_magnitude_;
  bool ball_alive_ = false;
  bool pose_received_ = false;
  std::string current_ball_name_ = "table_tennis_ball";
  rclcpp::Time spawn_time_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SpawnBall::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received spawn ball goal at [%.3f, %.3f, %.3f]",
                goal->x, goal->y, goal->z);

    // Check if ball is currently alive
    if (ball_alive_) {
      RCLCPP_WARN(this->get_logger(), "Ball is still alive, rejecting new spawn request");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSpawnBall> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSpawnBall> goal_handle)
  {
    // Spawn the ball in a separate thread
    std::thread{std::bind(&BallSpawnerActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSpawnBall> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing spawn ball action");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SpawnBall::Feedback>();
    auto result = std::make_shared<SpawnBall::Result>();

    // Delete existing ball if any
    if (ball_exists()) {
      delete_ball();
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Reset pose state
    current_ball_pose_.position.x = goal->x;
    current_ball_pose_.position.y = goal->y;
    current_ball_pose_.position.z = goal->z;
    pose_received_ = false;

    // Spawn new ball
    if (!spawn_ball(goal->x, goal->y, goal->z)) {
      result->final_status = "failed_to_spawn";
      result->final_x = goal->x;
      result->final_y = goal->y;
      result->final_z = goal->z;
      result->time_alive = 0.0;
      goal_handle->abort(result);
      return;
    }

    ball_alive_ = true;
    spawn_time_ = this->now();

    // Initialize pose
    current_ball_pose_.position.x = goal->x;
    current_ball_pose_.position.y = goal->y;
    current_ball_pose_.position.z = goal->z;
    previous_ball_pose_ = current_ball_pose_;
    current_velocity_magnitude_ = 0.0;
    pose_received_ = false;
    
    // Subscribe to Gazebo world poses using Gazebo Transport
    if (!gz_node_.Subscribe("/world/arena/dynamic_pose/info",
                           &BallSpawnerActionServer::on_pose_msg, this)) {
      RCLCPP_WARN(this->get_logger(), "Failed to subscribe to Gazebo pose topic");
    } else {
      RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo pose updates");
    }
    
    // Wait briefly for first pose update
    rclcpp::Rate init_rate(20);
    auto wait_start = this->now();
    while (!pose_received_ && (this->now() - wait_start).seconds() < 1.0) {
      init_rate.sleep();
    }
    
    if (pose_received_) {
      RCLCPP_INFO(this->get_logger(), "Ball initial pose: [%.3f, %.3f, %.3f]",
                  current_ball_pose_.position.x,
                  current_ball_pose_.position.y,
                  current_ball_pose_.position.z);
    }

    // Monitor ball and send feedback
    rclcpp::Rate rate(50); // 50 Hz feedback
    while (rclcpp::ok() && ball_alive_) {
      // Check if goal was canceled
      if (goal_handle->is_canceling()) {
        result->final_status = "canceled";
        result->final_x = current_ball_pose_.position.x;
        result->final_y = current_ball_pose_.position.y;
        result->final_z = current_ball_pose_.position.z;
        result->time_alive = (this->now() - spawn_time_).seconds();
        goal_handle->canceled(result);
        delete_ball();
        ball_alive_ = false;
        return;
      }

      // Update feedback
      feedback->status = "alive";
      feedback->current_x = current_ball_pose_.position.x;
      feedback->current_y = current_ball_pose_.position.y;
      feedback->current_z = current_ball_pose_.position.z;
      goal_handle->publish_feedback(feedback);

      // Publish status
      std_msgs::msg::String status_msg;
      status_msg.data = "alive";
      ball_status_pub_->publish(status_msg);
      ball_pose_pub_->publish(current_ball_pose_);

      // Check deletion conditions
      double time_alive = (this->now() - spawn_time_).seconds();
      // Ball radius is 0.02m, ground is at z=0
      // When ball is on ground, center should be at z=0.02, but may sink slightly
      // Also check if ball is below table (table top is at z=0.76)
      bool hit_ground = current_ball_pose_.position.z < 0.05;  // Increased threshold to catch ball on ground
      bool below_table = current_ball_pose_.position.z < 0.65;  // If below table, it's on the ground
      bool is_stationary = (time_alive > 1.0) && (current_velocity_magnitude_ < 0.005);  // After 1s, check if velocity < 0.005 m/s
      
      if (hit_ground || below_table || is_stationary) {
        if (hit_ground || below_table) {
          RCLCPP_INFO(this->get_logger(), "Ball hit the ground! z=%.4f", current_ball_pose_.position.z);
        } else {
          RCLCPP_INFO(this->get_logger(), "Ball became stationary! velocity=%.4f m/s", current_velocity_magnitude_);
        }
        ball_alive_ = false;

        // Publish dead status
        status_msg.data = "dead";
        ball_status_pub_->publish(status_msg);

        // Set result with proper status
        result->final_status = (hit_ground || below_table) ? "hit_ground" : "stationary";
        result->final_x = current_ball_pose_.position.x;
        result->final_y = current_ball_pose_.position.y;
        result->final_z = current_ball_pose_.position.z;
        result->time_alive = (this->now() - spawn_time_).seconds();

        // Delete ball entity
        delete_ball();

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Ball action completed, time alive: %.2f seconds",
                    result->time_alive);
        return;
      }

      rate.sleep();
    }
  }

  void on_pose_msg(const gz::msgs::Pose_V &_msg)
  {
    // Find our ball in the pose message
    for (int i = 0; i < _msg.pose_size(); ++i) {
      if (_msg.pose(i).name() == "table_tennis_ball") {
        // Store previous pose for velocity calculation
        previous_ball_pose_ = current_ball_pose_;
        auto prev_time = last_pose_update_time_;
        last_pose_update_time_ = this->now();
        
        // Update current pose
        const auto& gz_pose = _msg.pose(i);
        current_ball_pose_.position.x = gz_pose.position().x();
        current_ball_pose_.position.y = gz_pose.position().y();
        current_ball_pose_.position.z = gz_pose.position().z();
        current_ball_pose_.orientation.x = gz_pose.orientation().x();
        current_ball_pose_.orientation.y = gz_pose.orientation().y();
        current_ball_pose_.orientation.z = gz_pose.orientation().z();
        current_ball_pose_.orientation.w = gz_pose.orientation().w();
        
        // Calculate velocity magnitude from pose changes
        double dt = (last_pose_update_time_ - prev_time).seconds();
        if (dt > 0.001 && pose_received_) {  // Avoid division by zero and skip first update
          double dx = current_ball_pose_.position.x - previous_ball_pose_.position.x;
          double dy = current_ball_pose_.position.y - previous_ball_pose_.position.y;
          double dz = current_ball_pose_.position.z - previous_ball_pose_.position.z;
          double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
          current_velocity_magnitude_ = distance / dt;
        }
        
        pose_received_ = true;
        break;
      }
    }
  }

  bool ball_exists()
  {
    // For now, assume ball exists if ball_alive_ is true
    return ball_alive_;
  }

  bool spawn_ball(double x, double y, double z)
  {
    // Create SDF for ball with realistic physics
    std::stringstream sdf_stream;
    sdf_stream << "<?xml version='1.0'?>"
               << "<sdf version='1.9'>"
               << "  <model name='" << current_ball_name_ << "'>"
               << "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
               << "    <link name='ball_link'>"
               << "      <inertial>"
               << "        <mass>0.0027</mass>"  // 2.7 grams
               << "        <inertia>"
               << "          <ixx>4.32e-7</ixx>"
               << "          <ixy>0.0</ixy>"
               << "          <ixz>0.0</ixz>"
               << "          <iyy>4.32e-7</iyy>"
               << "          <iyz>0.0</iyz>"
               << "          <izz>4.32e-7</izz>"
               << "        </inertia>"
               << "      </inertial>"
               << "      <collision name='collision'>"
               << "        <geometry>"
               << "          <sphere><radius>0.020</radius></sphere>"
               << "        </geometry>"
               << "        <surface>"
               << "          <contact>"
               << "            <ode>"
               << "              <kp>100000.0</kp>"
               << "              <kd>10.0</kd>"
               << "              <max_vel>100.0</max_vel>"
               << "              <min_depth>0.0001</min_depth>"
               << "            </ode>"
               << "          </contact>"
               << "          <friction>"
               << "            <ode>"
               << "              <mu>0.6</mu>"   // Medium friction
               << "              <mu2>0.6</mu2>"
               << "            </ode>"
               << "          </friction>"
               << "          <bounce>"
               << "            <restitution_coefficient>0.85</restitution_coefficient>"  // ~85% bounce
               << "            <threshold>0.01</threshold>"
               << "          </bounce>"
               << "        </surface>"
               << "      </collision>"
               << "      <visual name='visual'>"
               << "        <geometry>"
               << "          <sphere><radius>0.020</radius></sphere>"
               << "        </geometry>"
               << "        <material>"
               << "          <ambient>1.0 0.6 0.0 1.0</ambient>"
               << "          <diffuse>1.0 0.7 0.1 1.0</diffuse>"
               << "          <specular>0.8 0.8 0.8 1.0</specular>"
               << "        </material>"
               << "      </visual>"
               << "    </link>"
               << "  </model>"
               << "</sdf>";

    // Write SDF to temporary file
    std::string sdf_file = "/tmp/ball_temp.sdf";
    std::ofstream file(sdf_file);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create temporary SDF file");
      return false;
    }
    file << sdf_stream.str();
    file.close();

    // Use gz service command to spawn the ball
    std::string command = "gz service -s /world/arena/create --reqtype gz.msgs.EntityFactory "
                          "--reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"" + 
                          sdf_file + "\"' 2>&1";
    
    RCLCPP_INFO(this->get_logger(), "Spawning ball at [%.3f, %.3f, %.3f]", x, y, z);
    int result = system(command.c_str());
    
    // Wait a bit for Gazebo to process the spawn
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "Ball spawn command executed successfully");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn ball, command exit code: %d", result);
      return false;
    }
  }

  void delete_ball()
  {
    // Use gz service command to delete the ball
    std::string command = "gz service -s /world/arena/remove --reqtype gz.msgs.Entity "
                          "--reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"" + 
                          current_ball_name_ + "\" type: MODEL'";
    
    int result = system(command.c_str());
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "Ball deleted");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to delete ball");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BallSpawnerActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
