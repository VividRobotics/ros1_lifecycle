/**
 * Copyright 2021 Lucas Walter
 * Lucas Walter
 * April 19th
 *
 * Demonstarte a lifecycle managed nodelet
 */
#include <lifecycle/managed_node.h>
#include <nodelet/nodelet.h>

namespace lifecycle_demos
{

class LifecycleDemo : public ros::lifecycle::ManagedNode, public nodelet::Nodelet
{
public:
  LifecycleDemo() {
    ROS_INFO_STREAM("constructor");
  }
  virtual void onInit() {
    ROS_INFO_STREAM("on init");
    setup(getPrivateNodeHandle(), "map");
  }

protected :
  bool onConfigure() {
    ROS_INFO_STREAM("Configuring");
    ROS_INFO_STREAM("Configued");
    return true;
  }
  bool onActivate() {
    ROS_INFO_STREAM("Activating");
    ROS_INFO_STREAM("Activated");
    return true;
  }
  bool onDeactivate() {
    ROS_INFO_STREAM("De-activating");
    ROS_INFO_STREAM("de-activated");
    return true;
  }
  bool onCleanup() {
    ROS_INFO_STREAM("Cleaning up");
    ROS_INFO_STREAM("cleaned up");
    return true;
  }
  bool onShutdown() {
    ROS_INFO_STREAM("Shutting down");
    ROS_INFO_STREAM("shut down");
    return true;
  }
};

}  // namespace lifecycle_demos

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lifecycle_demos::LifecycleDemo, nodelet::Nodelet)
