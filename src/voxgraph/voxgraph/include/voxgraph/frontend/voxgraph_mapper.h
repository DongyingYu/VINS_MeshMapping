#ifndef VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_
#define VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_

#include <future>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <voxblox_msgs/FilePath.h>
#include <voxgraph_msgs/LoopClosure.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/frame_names.h"
#include "voxgraph/frontend/map_tracker/map_tracker.h"
#include "voxgraph/frontend/measurement_processors/gps_processor.h"
#include "voxgraph/frontend/measurement_processors/pointcloud_integrator.h"
#include "voxgraph/frontend/pose_graph_interface/pose_graph_interface.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"
#include "voxgraph/tools/data_servers/loop_closure_edge_server.h"
#include "voxgraph/tools/data_servers/projected_map_server.h"
#include "voxgraph/tools/data_servers/submap_server.h"
#include "voxgraph/tools/rosbag_helper.h"
#include "voxgraph/tools/visualization/loop_closure_visuals.h"
#include "voxgraph/tools/visualization/submap_visuals.h"

namespace voxgraph {
class VoxgraphMapper {
 public:
  // Constructor & Destructor
  VoxgraphMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  VoxgraphMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 VoxgraphSubmap::Config submap_config,
                 voxblox::MeshIntegratorConfig mesh_config);
  ~VoxgraphMapper() = default;

  // ROS topic callbacks
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);
  void loopClosureCallback(const voxgraph_msgs::LoopClosure& loop_closure_msg);

  // ROS timer callbacks
  void publishActiveSubmapMeshCallback();

  // ROS service callbacks
  bool publishSeparatedMeshCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
  bool publishCombinedMeshCallback(
      std_srvs::Empty::Request& request,                            // NOLINT
      std_srvs::Empty::Response& response);                         // NOLINT
  bool optimizeGraphCallback(std_srvs::Empty::Request& request,     // NOLINT
                             std_srvs::Empty::Response& response);  // NOLINT
  bool finishMapCallback(std_srvs::Empty::Request& request,         // NOLINT
                         std_srvs::Empty::Response& response);      // NOLINT
  bool saveToFileCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool savePoseHistoryToFileCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool saveSeparatedMeshCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool saveCombinedMeshCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool saveOptimizationTimesCallback(
      voxblox_msgs::FilePath::Request& request,     // NOLINT
      voxblox_msgs::FilePath::Response& response);  // NOLINT

  const VoxgraphSubmapCollection& getSubmapCollection() {
    return *submap_collection_ptr_;
  }

  const PoseGraph::SolverSummaryList& getSolverSummaries() {
    return pose_graph_interface_.getSolverSummaries();
  }

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Verbosity and debug mode
  bool verbose_;

  // Flag and helper to automatically pause rosbags during graph optimization
  // NOTE: This is useful when playing bags faster than real-time or when
  //       experimenting with optimization settings
  bool auto_pause_rosbag_;
  RosbagHelper rosbag_helper_;

  // Interaction with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // New submap creation, pose graph optimization and map publishing
  void switchToNewSubmap(const ros::Time& current_timestamp);
  int optimizePoseGraph();
  void publishMaps(const ros::Time& current_timestamp);

  // Asynchronous handle for the pose graph optimization thread
  std::future<int> optimization_async_handle_;

  // ROS topic subscribers
  std::string odometry_input_topic;
  std::string pointcloud_topic_;
  int subscriber_queue_length_;
  ros::Subscriber pointcloud_subscriber_;
  std::string loop_closure_topic_;
  int loop_closure_subscriber_queue_length_;
  ros::Subscriber loop_closure_subscriber_;
  // TODO(victorr): Add support for absolute pose measurements

  // Timers.
  ros::Timer update_mesh_timer_;

  // ROS topic publishers
  ros::Publisher separated_mesh_pub_;
  ros::Publisher active_mesh_pub_;
  ros::Publisher combined_mesh_pub_;
  ros::Publisher pose_history_pub_;
  ros::Publisher loop_closure_links_pub_;
  ros::Publisher loop_closure_axes_pub_;

  // ROS service servers
  ros::ServiceServer publish_separated_mesh_srv_;
  ros::ServiceServer publish_combined_mesh_srv_;
  ros::ServiceServer optimize_graph_srv_;
  ros::ServiceServer finish_map_srv_;
  ros::ServiceServer save_to_file_srv_;
  ros::ServiceServer save_pose_history_to_file_srv_;
  ros::ServiceServer save_separated_mesh_srv_;
  ros::ServiceServer save_combined_mesh_srv_;
  ros::ServiceServer save_optimization_times_srv_;
  // TODO(victorr): Add srvs to receive absolute pose and loop closure updates

  // Constraints to be used
  bool registration_constraints_enabled_;
  bool odometry_constraints_enabled_;
  bool height_constraints_enabled_;

  // Instantiate the submap collection
  VoxgraphSubmap::Config submap_config_;
  VoxgraphSubmapCollection::Ptr submap_collection_ptr_;

  // Visualization tools
  SubmapVisuals submap_vis_;
  LoopClosureVisuals loop_closure_vis_;

  // Interface to ease interaction with the pose graph
  PoseGraphInterface pose_graph_interface_;

  // Measurement processors
  PointcloudIntegrator pointcloud_integrator_;

  // Map servers, used to share the projected map and submaps with ROS nodes
  ProjectedMapServer projected_map_server_;
  SubmapServer submap_server_;
  LoopClosureEdgeServer loop_closure_edge_server_;

  // Map tracker handles the odometry input and refines it using scan-to-map ICP
  MapTracker map_tracker_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_FRONTEND_VOXGRAPH_MAPPER_H_
