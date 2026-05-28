#ifndef WAYPOINT_TOOL_HPP_
#define WAYPOINT_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rviz_waypoint_tool
{
class WaypointTool : public rviz_common::Tool
{
public:
  WaypointTool();
  ~WaypointTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rviz_common::properties::StringProperty* topic_property_;
};
}  // namespace rviz_waypoint_tool

#endif  // WAYPOINT_TOOL_HPP_