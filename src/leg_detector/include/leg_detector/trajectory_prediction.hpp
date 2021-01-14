#include <rclcpp/rclcpp.hpp>
#include <leg_detector_msgs/msg/trajectory.hpp>
#include <leg_detector_msgs/msg/trajectory_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "leg_detector/spline.h"
#include <utility>
#include <vector>

class TrajectoryPredictionNode : public rclcpp::Node
{
public:
  TrajectoryPredictionNode();
  void pairSort(std::vector<double> &a, std::vector<double> &b);
private:
  void trajectoryArrayCallback(const leg_detector_msgs::msg::TrajectoryArray::SharedPtr msg);
  rclcpp::Subscription<leg_detector_msgs::msg::TrajectoryArray>::SharedPtr trajectoryArraySubscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predictedTrajectoryPublisher_;

  std::string trajectory_array_topic_;
  std::string trajectory_visualization_topic_;
};
