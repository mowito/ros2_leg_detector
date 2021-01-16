#include "leg_detector/trajectory_prediction.hpp"

void Line::addPoints(std::vector<double> x, std::vector<double> y)
{
  double sum_x = 0;
  double sum_y = 0;
  double sum_xy = 0;
  double sum_x2 = 0;

  for (int i = 0; i < static_cast<int>(x.size()); i++) {
    sum_x += x[i];
    sum_y += y[i];
    sum_xy += x[i] * y[i];
    sum_x2 += x[i] * x[i];
  }
  
  int n = x.size();
  double x_mean = sum_x / n;
  double y_mean = sum_y / n;
  double den  = sum_x2 - sum_x * x_mean;

  if(std::fabs(den) < 1e-7) {
    this->a = 1;
    this->b = 0;
    this->c = -x_mean;
    return;
  }

  double slope = (sum_xy - sum_x * y_mean) / den;
  double y_int = y_mean - slope * x_mean;

  this->b = 1;
  this->a = -slope;
  this->c = -y_int;
}

double Line::getA()
{
  return this->a;
}

double Line::getB()
{
  return this->b;
}

double Line::getC()
{
  return this->c;
}

double Line::getY(double x)
{
  return (-this->c - this->a * x) / this->b;
}

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

void TrajectoryPredictionNode::trajectoryArrayCallback(const leg_detector_msgs::msg::TrajectoryArray::SharedPtr msg)
{
  for (long unsigned int i = 0; i < msg->trajectories.size(); i++) {
    Line s;
    std::vector<double> x, y;

    for (unsigned long int j = std::max(0, static_cast<int>(msg->trajectories[i].x.size()) - 5); j < msg->trajectories[i].x.size(); j++) {
      x.push_back(msg->trajectories[i].x[j]);
      y.push_back(msg->trajectories[i].y[j]);
    }

    double d = x[x.size() - 1] - x[x.size() - 2];
    double x_ = x[x.size() - 1] + d;

    s.addPoints(x, y);

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
    
    for(int j = 0; j < 15; j++) {
      geometry_msgs::msg::Point p;
      p.x = x_;
      p.y = s.getY(x_);

      m.points.push_back(p);
      x_ += d;
    }
    predictedTrajectoryPublisher_->publish(m);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPredictionNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
