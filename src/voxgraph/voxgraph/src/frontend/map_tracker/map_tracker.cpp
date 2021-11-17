#include "voxgraph/frontend/map_tracker/map_tracker.h"

#include <limits>
#include <string>
#include <utility>

#include "voxgraph/tools/tf_helper.h"

//使用Tf信息 或是里程计信息，怎么设置的
namespace voxgraph {
MapTracker::MapTracker(VoxgraphSubmapCollection::ConstPtr submap_collection_ptr,
                       FrameNames frame_names, bool verbose)
    : verbose_(verbose),
      submap_collection_ptr_(submap_collection_ptr),
      frame_names_(std::move(frame_names)),
      tf_transformer_(),
      odom_transformer_() {}

void MapTracker::subscribeToTopics(ros::NodeHandle nh,
                                   const std::string& odometry_input_topic) {\
  // .empty()用来测试变量是否已经配置。若变量已存在、非空字符串或者非零，则返回 false 值；反之返回 true值。
  if (!odometry_input_topic.empty()) {
    ROS_INFO_STREAM("Using odometry from ROS topic: " << odometry_input_topic);
    //只是表示里程计信息不从tf中得出？
    //即使使用里程计信息，也只是用来获取转换关系，tf信息同样需要输入吗？
    //在launch文件中添加节点信息
    //如果设置了里程计输入，use_odom_from_tfs_自动设置为false，可不在map_tracker.h文件中设置
    use_odom_from_tfs_ = false;
    odom_transformer_.subscribeToTopic(nh, odometry_input_topic);
  }
}

void MapTracker::advertiseTopics(ros::NodeHandle nh_private,
                                 const std::string& odometry_output_topic) {}

bool MapTracker::updateToTime(const ros::Time& timestamp,
                              const std::string& sensor_frame_id) {
  // Keep track of the timestamp that the MapTracker is currently at
  current_timestamp_ = timestamp;

  // Update the odometry
  // 根据map_tracker.h中的参数决定执行哪些代码
  if (use_odom_from_tfs_) {
    // Update the odometry estimate
    //从TF中获得转换矩阵信息
    if (!tf_transformer_.lookupTransform(frame_names_.input_odom_frame,
                                         frame_names_.input_base_link_frame,
                                         timestamp, &T_O_B_)) {
      return false;
    }
  } else {
    //直接从里程计数据中获得,使用里程计信息就不需要odom_frame 或 base_link_frame吗?
    if (!odom_transformer_.lookupTransform(timestamp, &T_O_B_)) {
      return false;
    }
  }

  // Express the odometry pose in the frame of the current submap
  T_S_B_ = initial_T_S_O_ * T_O_B_;

  // Get the transformation from the pointcloud sensor to the robot's
  // base link from TFs, unless it was already provided through ROS params
  // 从TF信息中获取robot与base_link之间的转换关系，即标定数据
  if (use_sensor_calibration_from_tfs_) {
    // TODO(victorr): Implement option to provide a sensor_frame_id instead of
    //                taking the one from the message
    // Strip leading slashes if needed to avoid TF errors
    std::string tf_sensor_frame_id;
    if (sensor_frame_id[0] == '/') {
      tf_sensor_frame_id = sensor_frame_id.substr(1, sensor_frame_id.length());
    } else {
      tf_sensor_frame_id = sensor_frame_id;
    }

    // Lookup the transform
    if (!tf_transformer_.lookupTransform(frame_names_.input_base_link_frame,
                                         tf_sensor_frame_id, timestamp,
                                         &T_B_C_)) {
      return false;
    }
  }

  // Signal that all transforms were successfully updated
  return true;
}

void MapTracker::switchToNewSubmap(const Transformation& T_M_S_new) {
  // Store the initial submap pose for visualization purposes
  initial_T_M_S_ = T_M_S_new;

  // Get the pose of the new submap in odom frame
  Transformation T_O_S = VoxgraphSubmapCollection::gravityAlignPose(T_O_B_);

  // Store the transform used to convert the odometry input into submap frame
  initial_T_S_O_ = T_O_S.inverse();

  // Update the current robot pose
  // NOTE: This initial pose can differ from Identity, since the submap pose
  //       has zero pitch and roll whereas the robot pose is in 6DoF
  T_S_B_ = initial_T_S_O_ * T_O_B_;
}

//tf_is_static设置为flase
void MapTracker::publishTFs() {
  TfHelper::publishTransform(submap_collection_ptr_->getActiveSubmapPose(),
                             frame_names_.output_mission_frame,
                             frame_names_.output_active_submap_frame, false,
                             current_timestamp_);
  TfHelper::publishTransform(
      initial_T_S_O_, frame_names_.output_active_submap_frame,
      frame_names_.output_odom_frame, false, current_timestamp_);

  // 如果输入odom、base_link与输出的名字不一致，或者没有从tfs中获取里程计信息，执行以下tf发布代码
  if (frame_names_.input_odom_frame != frame_names_.output_odom_frame ||
      frame_names_.input_base_link_frame !=
          frame_names_.output_base_link_frame ||
      !use_odom_from_tfs_) {
    // Republish the odometry if the output frame names are different,
    // or if the odom input is coming from a ROS topic
    // (in which case it might not yet be in the TF tree)
    TfHelper::publishTransform(T_O_B_, frame_names_.output_odom_frame,
                               frame_names_.output_base_link_frame, false,
                               current_timestamp_);
  }
  //否则，发布如下信息
  TfHelper::publishTransform(T_B_C_, frame_names_.output_base_link_frame,
                             frame_names_.output_sensor_frame, true,
                             current_timestamp_);
}

Transformation MapTracker::get_T_M_B() {
  if (submap_collection_ptr_->empty()) {
    // If no submap has been created yet, return the odometry pose
    return T_O_B_;
  } else {
    return submap_collection_ptr_->getActiveSubmapPose() * T_S_B_;
  }
}

void MapTracker::set_T_B_C(const Transformation& T_B_C) {
  T_B_C_ = T_B_C;
  use_sensor_calibration_from_tfs_ = false;
 }
}  // namespace voxgraph
