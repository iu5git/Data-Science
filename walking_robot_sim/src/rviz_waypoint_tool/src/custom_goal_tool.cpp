#include "rviz_waypoint_tool/custom_goal_tool.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <OgreRay.h>
#include <OgrePlane.h>
#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>

namespace rviz_waypoint_tool
{
WaypointTool::WaypointTool()
{
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/custom_goal_pose",
    "The topic on which to publish custom goal poses.",
    getPropertyContainer(), nullptr, this);
}

void WaypointTool::onInitialize()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), 10);
}

void WaypointTool::activate()
{
}

void WaypointTool::deactivate()
{
}

int WaypointTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  if (event.leftDown())
  {
    if (!event.panel)
    {
      return 0;
    }

    rviz_common::ViewManager* view_manager = context_->getViewManager();
    if (!view_manager)
    {
      return 0;
    }

    rviz_common::ViewController* view_controller = view_manager->getCurrent();
    if (!view_controller)
    {
      return 0;
    }

    Ogre::Camera* camera = view_controller->getCamera();
    if (!camera)
    {
      return 0;
    }

    int width = event.panel->width();
    int height = event.panel->height();

    if (width <= 0 || height <= 0)
    {
      return 0;
    }

    Ogre::Ray mouse_ray = camera->getCameraToViewportRay(
      static_cast<Ogre::Real>(event.x) / static_cast<Ogre::Real>(width),
      static_cast<Ogre::Real>(event.y) / static_cast<Ogre::Real>(height));

    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);

    if (intersection.first)
    {
      Ogre::Vector3 point = mouse_ray.getPoint(intersection.second);

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.header.stamp = rclcpp::Clock().now();
      goal_pose.pose.position.x = point.x;
      goal_pose.pose.position.y = point.y;
      goal_pose.pose.position.z = 0.0;
      goal_pose.pose.orientation.w = 1.0;

      pose_publisher_->publish(goal_pose);
      RCLCPP_INFO(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                  "Published waypoint: x=%f, y=%f",
                  point.x, point.y);
    }
    return Finished;
  }
  return 0;
}
}  // namespace rviz_waypoint_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_waypoint_tool::WaypointTool, rviz_common::Tool)