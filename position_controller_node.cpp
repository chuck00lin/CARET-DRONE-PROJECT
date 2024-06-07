// position_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp> //for odom class
#include <std_msgs/msg/bool.hpp>
//only for box mode and emergency stop
#include <chrono>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
// For the M_PI and std::atan2 function
#include <cmath> 
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <mutex>

class PositionController : public rclcpp::Node
{
private:
  nav_msgs::msg::Odometry current_pose_;
  nav_msgs::msg::Odometry uav_odom_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped home_pose_;
  bool home_set_ = false;
  bool source_received_ = true;
  std::chrono::time_point<std::chrono::steady_clock> last_received_time_;
  
  geometry_msgs::msg::Vector3 integral_error_;
  geometry_msgs::msg::Vector3 last_error_;
  double integral_yaw_error_ = 0.0;
  double last_yaw_error_ = 0.0;
  std::mutex state_mutex_;

  // --Parameters--
  double Kp_, Ki_, Kd_, Ki_yaw_, Kd_yaw_, Kp_yaw_;
  double max_integral_ = 10.0; 
  double max_yaw_integral_ = 10.0;
  bool box_mode_ = false;

  // --Timer to check if source is received--
  rclcpp::TimerBase::SharedPtr source_check_timer_;
  // --Safety clients--
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarm_client_;

  // --Logger--
  rclcpp::Logger logger_;
  // --Logging--
  std::ofstream log_file_;
  std::string log_file_path_;
  bool logging_enabled_ = false;
  std::mutex log_mutex_;

  // --mode--  
  std::string source_mode_ = "vision";  // or "vision-calibrated" or any other source
  std::string pose_source_;

  // --Subscribers-- 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uav_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
  // --Publishers--
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

public:
  PositionController() : Node("position_controller"),logger_(rclcpp::get_logger("position_controller"))
  {

    
    
    this->declare_parameters();
    this->get_parameters();
    this->initialize_subscribers_and_publishers();

    // Timer to check if source is received
    source_check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PositionController::check_source_received, this));

    // Initialize safety clients
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    disarm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  }

  void declare_parameters()
  {
    // Declare your parameters here
    this->declare_parameter<double>("Kp", 1.0);
    this->declare_parameter<double>("Ki", 0.1);
    this->declare_parameter<double>("Kd", 0.1);
    this->declare_parameter<double>("Ki_yaw", 0.1);
    this->declare_parameter<double>("Kd_yaw", 0.1);
    this->declare_parameter<double>("Kp_yaw", 1.0);
    this->declare_parameter<bool>("box_mode", false);
    this->declare_parameter<std::string>("log_file_path", "");
    this->declare_parameter<std::string>("source_mode", "vision");
    this->declare_parameter<bool>("logging_enabled", false);
  }

  void get_parameters()
  {
    // Get parameters from YAML or CLI
    this->get_parameter("Kp", Kp_);
    this->get_parameter("Ki", Ki_);
    this->get_parameter("Kd", Kd_);
    this->get_parameter("Ki_yaw", Ki_yaw_);
    this->get_parameter("Kd_yaw", Kd_yaw_);
    this->get_parameter("Kp_yaw", Kp_yaw_);
    this->get_parameter("box_mode", box_mode_);
    this->get_parameter("source_mode", source_mode_);
    this->get_parameter("logging_enabled", logging_enabled_);
    this->get_parameter("log_file_path", log_file_path_);
    RCLCPP_INFO(rclcpp::get_logger("position_controller"), "Using box mode: %d", box_mode_);
    RCLCPP_INFO(rclcpp::get_logger("position_controller"), "Kp: %f, Ki: %f, Kd: %f", Kp_, Ki_, Kd_);

    std::string log_file_path;
    this->get_parameter("log_file_path", log_file_path);
    this->get_parameter("logging_enabled", logging_enabled_);

    // Set Odometry source
    if (source_mode_ == "vision") {
        pose_source_ = "/stella_ros/camera_pose";
    } 
    else if (source_mode_ == "vision-calibrated") {
        pose_source_ = "/calibrated/camera_pose";
    }
    else {
        pose_source_ = "/mavros/local_position/odom";
    }

    RCLCPP_INFO(this->get_logger(), "Using pose source: %s", pose_source_.c_str());

    if (logging_enabled_ && !log_file_path.empty()) {
      // Add timestamp to the log file name
      std::stringstream log_filename;
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);

      log_filename << log_file_path;
      log_filename << std::put_time(std::localtime(&in_time_t), "_%Y-%m-%d_%H-%M-%S");
      log_filename << ".log";

      log_file_.open(log_filename.str(), std::ios::out);
      if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open log file: %s", log_filename.str().c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Logging to file: %s", log_filename.str().c_str());
      }
    }
  }

  void initialize_subscribers_and_publishers()
  {
    // Initialize subscribers and publishers here
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    local_pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_source_, qos, 
                                std::bind(&PositionController::local_position_cb, this, std::placeholders::_1));

    // for velocity loop (not used for now, only for logging)
    uav_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("mavros/local_position/odom", qos, 
                                        std::bind(&PositionController::uav_odom_cb, this, std::placeholders::_1));

    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("waypoint", 10, 
                                        std::bind(&PositionController::waypoint_cb, this, std::placeholders::_1));
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  }

  void log_to_file(const std::string &message) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (logging_enabled_ && log_file_.is_open()) {
      log_file_ << message << std::endl;
    }
  }

  void check_source_received() {
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_received_time_).count();

    if (!home_set_ && time_diff > 3000) {
        RCLCPP_WARN(rclcpp::get_logger("position_controller"), "No pose source received, pllease check the connection");
        source_received_ = false;
    }
    if (home_set_ && time_diff > 500) {
        // If we haven't received a message in the last 500 milliseconds, consider the source lost
        RCLCPP_WARN(rclcpp::get_logger("position_controller"), "Time since last message: %ld ms", time_diff);
        source_received_ = false;
        trigger_safety_procedures();
    }
  }
  void trigger_safety_procedures() {
    // Set flight mode to LAND 
    // Ned Fix: (when flying in OFFBOARD mode, disarm will not triggered sucessfully)
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "AUTO.LAND";
    set_mode_client_->async_send_request(set_mode_request,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        if (future.get()->mode_sent) {
            RCLCPP_INFO(logger_, "Set mode to LAND");
            // After setting mode to LAND, disarm the drone
            auto disarm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            disarm_request->value = false;
            disarm_client_->async_send_request(disarm_request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                    if (future.get()->success) {
                        RCLCPP_INFO(logger_, "Disarmed drone");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to disarm drone");
                    }
                    // Shut down after disarming
                    rclcpp::shutdown();
                });
        } else { // Lost source when in Land mode
            RCLCPP_ERROR(logger_, "Failed to set mode to LAND");
            // Still try to disarm even if setting mode to LAND failed
            auto disarm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            disarm_request->value = false;
            disarm_client_->async_send_request(disarm_request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                    if (future.get()->success) {
                        RCLCPP_INFO(logger_, "Disarmed drone");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to disarm drone");
                    }
                    // Shut down after disarming attempt
                    rclcpp::shutdown();
                });
        }
      });
  }


  void check_safety_box() {
    // Assuming the box center bottom is at (0, 0, 0) and dimensions are 2*2*2
    if (fabs(current_pose_.pose.pose.position.x) > 1.0 ||
        fabs(current_pose_.pose.pose.position.y) > 1.0 ||
        fabs(current_pose_.pose.pose.position.z) > 2.0) {
        RCLCPP_WARN(rclcpp::get_logger("position_controller"), "Drone is outside the box");

        // Set the target pose to home_pose to return the drone to home
        target_pose_.pose.position.x = home_pose_.pose.position.x;
        target_pose_.pose.position.y = home_pose_.pose.position.y;
        target_pose_.pose.position.z = home_pose_.pose.position.z;        
    }
  }

  void waypoint_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_ = *msg;
  }
  void uav_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    uav_odom_ = *msg;
  }

  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void apply_pid(geometry_msgs::msg::Vector3& velocity, double& yaw_velocity, geometry_msgs::msg::Vector3& error, double yaw_error) 
  {
      // Update integral error with clamping
      integral_error_.x += std::clamp(error.x, -max_integral_, max_integral_);
      integral_error_.y += std::clamp(error.y, -max_integral_, max_integral_);
      integral_error_.z += std::clamp(error.z, -max_integral_, max_integral_);
      integral_yaw_error_ += std::clamp(yaw_error, -max_yaw_integral_, max_yaw_integral_);

      // Compute derivative error
      geometry_msgs::msg::Vector3 derivative_error;
      derivative_error.x = error.x - last_error_.x;
      derivative_error.y = error.y - last_error_.y;
      derivative_error.z = error.z - last_error_.z;
      double derivative_yaw_error = yaw_error - last_yaw_error_;

      // PID controller formula
      velocity.x = Kp_ * error.x + Ki_ * integral_error_.x + Kd_ * derivative_error.x;
      velocity.y = Kp_ * error.y + Ki_ * integral_error_.y + Kd_ * derivative_error.y;
      velocity.z = Kp_ * error.z + Ki_ * integral_error_.z + Kd_ * derivative_error.z;
      yaw_velocity = Kp_yaw_ * yaw_error + Ki_yaw_ * integral_yaw_error_ + Kd_yaw_ * derivative_yaw_error;

      // Update last_error
      last_error_ = error;
      last_yaw_error_ = yaw_error;
  }

  void local_position_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = *msg;
    source_received_ = true;
    last_received_time_ = std::chrono::steady_clock::now();

    // Set home pose if not set
    if (!home_set_) {
        home_pose_.pose = msg->pose.pose;
        home_set_ = true;
        double home_yaw_rad = get_yaw_from_quaternion(home_pose_.pose.orientation);
        // Convert to degrees
        double home_yaw_deg = home_yaw_rad * (180.0 / M_PI);
        
        RCLCPP_INFO(rclcpp::get_logger("position_controller"),
        "Home pose set to: x=%f, y=%f, z=%f, yaw=%f degrees", 
        home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z, home_yaw_deg);
    }
    // Check if drone is outside the box when in box mode
    if (box_mode_) {
        check_safety_box();
    }
    // Calculate error here (assumes target_pose_ has been set)
    geometry_msgs::msg::Vector3 error;
    error.x = target_pose_.pose.position.x - current_pose_.pose.pose.position.x;
    error.y = target_pose_.pose.position.y - current_pose_.pose.pose.position.y;
    error.z = target_pose_.pose.position.z - current_pose_.pose.pose.position.z;

    double current_yaw = get_yaw_from_quaternion(current_pose_.pose.pose.orientation);
    double target_yaw = get_yaw_from_quaternion(target_pose_.pose.orientation);
    double yaw_error = target_yaw - current_yaw;

    //Handle angle wrapping
    if (yaw_error > M_PI) {
      yaw_error -= 2 * M_PI;
    } else if (yaw_error < -M_PI) {
      yaw_error += 2 * M_PI;
    }
    // Apply PID controller
    geometry_msgs::msg::Vector3 velocity_cmd;
    double yaw_velocity_cmd;
    apply_pid(velocity_cmd, yaw_velocity_cmd, error, yaw_error);
    
    // Logging (if enabled)
    if (logging_enabled_) {
      std::stringstream log_stream;
      log_stream << "Current Pose: " << "X: " << current_pose_.pose.pose.position.x << ", "
                 << "Y: " << current_pose_.pose.pose.position.y << ", "
                 << "Z: " << current_pose_.pose.pose.position.z << ", "
                 << "Yaw: " << get_yaw_from_quaternion(current_pose_.pose.pose.orientation) << ", "
                 << "Target Pose: " << "X: " << target_pose_.pose.position.x << ", "
                 << "Y: " << target_pose_.pose.position.y << ", "
                 << "Z: " << target_pose_.pose.position.z << ", "
                 << "Yaw: " << get_yaw_from_quaternion(target_pose_.pose.orientation) << ", "
                 << "Velocity_out: " << "X: " << velocity_cmd.x << ", "
                 << "Y: " << velocity_cmd.y << ", "
                 << "Z: " << velocity_cmd.z << ", "
                 << "Velocity_in: " << "X: " << uav_odom_.twist.twist.linear.x << ", "
                 << "Y: " << uav_odom_.twist.twist.linear.y << ", "
                 << "Z: " << uav_odom_.twist.twist.linear.z << ", "
                 << "Error: " << "X: " << error.x << ", "
                 << "Y: " << error.y << ", "
                 << "Z: " << error.z << ", ";
      log_to_file(log_stream.str());
    }

    // Create and publish the TwistStamped message
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.twist.linear = velocity_cmd;
    twist_msg.twist.angular.z = yaw_velocity_cmd; // Add yaw velocity
    velocity_pub_->publish(twist_msg);
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionController>());
  rclcpp::shutdown();
  return 0;
}
