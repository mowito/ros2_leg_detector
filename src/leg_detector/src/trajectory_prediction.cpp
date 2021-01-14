#include "leg_detector/trajectory_prediction.hpp"

TrajectoryPredictionNode::TrajectoryPredictionNode():
  Node("trajectory_prediction_node")
{
  this->declare_parameter("trajectory_visualization_topic");
  this->declare_parameter("trajectory_array_topic");

  this->get_parameter_or("trajectory_visualization_topic", trajectory_visualization_topic_, std::string("/predicted_trajectories/marker"));
  this->get_parameter_or("trajectory_array_topic", trajectory_array_topic_, std::string("/trajectories"));

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "trajectory_visualization_topic: %s", trajectory_visualization_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "trajectory_array_topic: %s", trajectory_array_topic_.c_str());

  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

  this->predictedTrajectoryPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>(trajectory_visualization_topic_, 20);
  this->trajectoryArraySubscriber_ = this->create_subscription<leg_detector_msgs::msg::TrajectoryArray>(trajectory_array_topic_, default_qos, std::bind(&TrajectoryPredictionNode::trajectoryArrayCallback, this, std::placeholders::_1));
}

void TrajectoryPredictionNode::pairSort(std::vector<double> &a, std::vector<double> &b)
{
  unsigned long int n = static_cast<int>(a.size());
  std::pair<float, float> pairt[n]; 
  
  // Storing the respective array 
  // elements in pairs. 
  for (unsigned long int i = 0; i < n; i++) { 
    pairt[i].first = a[i]; 
    pairt[i].second = b[i]; 
  } 
  
  // Sorting the pair array. 
  std::sort(pairt, pairt + n);
      
  // Modifying original arrays 
  for (unsigned long int i = 0; i < n; i++) { 
    a[i] = pairt[i].first; 
    b[i] = pairt[i].second; 
  }
}

void TrajectoryPredictionNode::trajectoryArrayCallback(const leg_detector_msgs::msg::TrajectoryArray::SharedPtr msg)
{
  for (long unsigned int i = 0; i < msg->trajectories.size(); i++) {
    tk::spline s;
    std::vector<double> x, y;
    // RCLCPP_INFO(this->get_logger(), "Message arrived! size = %d, %d", msg->trajectories[i].x.size(), msg->trajectories[i].y.size());
    for (unsigned long int j = std::max(0, static_cast<int>(msg->trajectories[i].x.size()) - 3); j < msg->trajectories[i].x.size(); j++) {
      x.push_back(msg->trajectories[i].x[j]);
      y.push_back(msg->trajectories[i].y[j]);
      // RCLCPP_INFO(this->get_logger(), "Added point %d.", j+1);  
    }
    // RCLCPP_INFO(this->get_logger(), "Taking %d points", x.size());

    double d = x[x.size() - 1] - x[x.size() - 2];
    double x_ = x[x.size() - 1] + d;

    pairSort(x, y);
    s.set_points(x, y, true);
    // RCLCPP_INFO(this->get_logger(), "Fit a spline!");

    // RCLCPP_INFO(this->get_logger(), "d = %f", d);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = msg->header.frame_id;
    m.header.stamp = msg->header.stamp;
    m.ns = "trajectory_predicted";
    m.id = 100;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.scale.x = 0.03;
    m.scale.y = 0.03;
    m.scale.z = 0.03;

    // RCLCPP_INFO(this->get_logger(), "Initialized m");
    
    for(int j = 0; j < 10; j++) {
      geometry_msgs::msg::Point p;
      // RCLCPP_INFO(this->get_logger(), "Sampling point");
      p.x = x_;
      p.y = s(x_);

      if (std::hypot(x[x.size() - 1] - x_, y[y.size() - 1] - p.y) > 5)
        continue;
      m.points.push_back(p);
      x_ += d;
      // RCLCPP_INFO(this->get_logger(), "Added point %d", j);
    }
    // RCLCPP_INFO(this->get_logger(), "Predicted trajectory");
    predictedTrajectoryPublisher_->publish(m);
    // RCLCPP_INFO(this->get_logger(), "Published");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPredictionNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
