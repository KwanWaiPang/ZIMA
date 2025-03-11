/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima_ros/zima_node.h"

// #include <cartographer_ros_msgs/StartTrajectory.h>
// #include <cartographer_ros_msgs/WriteState.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/algorithm/slam/simple_slam.h"
#include "zima/common/util.h"
#include "zima/event/event_manager.h"
#include "zima/grid_map/map_util.h"
#include "zima/hal/system/process.h"
#include "zima_ros/util.h"
#include "zima_ros/zima_ros_version.h"

namespace zima_ros {

using namespace zima;

float cleaning_info_z = 0;
float print_map_layer_z = 0;
float user_area_map_layer_z = -0.1;
float virtual_obs_map_layer_z = -0.2;
float room_map_layer_z = -0.3;
float slam_char_map_layer_z = -0.4;
float underlay_layer_z = -0.5;
float path_z = 0.2;
float plan_path_z = 0.1;
float robot_z = 0.3;

// class CartographerSlam : public SlamBase {
//  public:
//   CartographerSlam() = delete;
//   CartographerSlam(ros::ServiceClient *cartographer_start_trajectory_client,
//                    ros::ServiceClient *cartographer_stop_client,
//                    ros::ServiceClient *cartographer_pause_resume_client)
//       : SlamBase(SlamBase::Config()),
//         cartographer_start_trajectory_client_(
//             cartographer_start_trajectory_client),
//         cartographer_stop_client_(cartographer_stop_client),
//         cartographer_pause_resume_client_(cartographer_pause_resume_client) {
//     is_ready_.store(true);
//   }
//   ~CartographerSlam() = default;

//   bool StartSlam(const MapPoint &init_pose, const std::string &save_file_name,
//                  const std::string &load_file_name) override {
//     if (is_running_.load()) {
//       ZINFO << "Slam is already running.";
//       return false;
//     }

//     // Start cartographer.
//     cartographer_ros_msgs::StartTrajectory request;
//     request.request.use_initial_pose = !load_file_name.empty();
//     geometry_msgs::Pose cartographer_init_pose;
//     cartographer_init_pose.position.x = init_pose.X();
//     cartographer_init_pose.position.y = init_pose.Y();
//     auto init_pose_transform = zima_ros::MapPointToTFTransform(init_pose);
//     cartographer_init_pose.orientation.x =
//         init_pose_transform.getRotation().x();
//     cartographer_init_pose.orientation.y =
//         init_pose_transform.getRotation().y();
//     cartographer_init_pose.orientation.z =
//         init_pose_transform.getRotation().z();
//     cartographer_init_pose.orientation.w =
//         init_pose_transform.getRotation().w();
//     request.request.initial_pose = cartographer_init_pose;
//     request.request.configuration_basename = load_file_name;
//     request.request.configuration_directory = save_file_name;

//     while (ros::ok()) {
//       ZINFO << "Call for start slam.";
//       if (!cartographer_start_trajectory_client_->call(request)) {
//         ZERROR << "Call service failed.";
//         Time::SleepSec(1);
//         continue;
//       }
//       break;
//     }
//     ZINFO << "Call for start slam finish.";

//     {
//       WriteLocker lock(access_);
//       start_or_resume_timestamp_ = Time::Now();
//     }

//     is_running_.store(true);

//     return true;
//   }

//   bool StopSlam() override {
//     // if (!is_running_.load()) {
//     //   ZINFO << "Slam is not running.";
//     //   return false;
//     // }

//     cartographer_ros_msgs::WriteState request;
//     while (ros::ok()) {
//       ZINFO << "Call for stop slam.";
//       if (!cartographer_stop_client_->call(request)) {
//         ZERROR << "Call service failed.";
//         Time::SleepSec(1);
//         continue;
//       }
//       break;
//     }
//     ZINFO << "Call for stop slam finish.";

//     // Wait for slam file to be written.
//     Time::SleepMSec(300);

//     {
//       WriteLocker lock(access_);
//       stop_or_pause_timestamp_ = Time::Now();
//     }

//     is_running_.store(false);
//     return true;
//   }

//   bool PauseSlam() override {
//     if (!is_running_.load()) {
//       ZINFO << "Slam is not running.";
//       return false;
//     }

//     cartographer_ros_msgs::StartTrajectory request;
//     request.request.use_initial_pose = false;
//     while (ros::ok()) {
//       ZINFO << "Call for pause slam.";
//       if (!cartographer_pause_resume_client_->call(request)) {
//         ZERROR << "Call service failed.";
//         Time::SleepSec(1);
//         continue;
//       }
//       break;
//     }
//     ZINFO << "Call for pause slam finish.";

//     {
//       WriteLocker lock(access_);
//       stop_or_pause_timestamp_ = Time::Now();
//     }

//     is_running_.store(false);

//     return true;
//   }

//   bool ResumeSlam(const MapPoint &init_pose) override {
//     if (is_running_.load()) {
//       ZINFO << "Slam is already running.";
//       return false;
//     }

//     // Wait for a moment for relocation result to publish.
//     Time::SleepMSec(100);

//     cartographer_ros_msgs::StartTrajectory request;
//     request.request.use_initial_pose = true;
//     geometry_msgs::Pose cartographer_init_pose;
//     cartographer_init_pose.position.x = init_pose.X();
//     cartographer_init_pose.position.y = init_pose.Y();
//     auto init_pose_transform = zima_ros::MapPointToTFTransform(init_pose);
//     cartographer_init_pose.orientation.x =
//         init_pose_transform.getRotation().x();
//     cartographer_init_pose.orientation.y =
//         init_pose_transform.getRotation().y();
//     cartographer_init_pose.orientation.z =
//         init_pose_transform.getRotation().z();
//     cartographer_init_pose.orientation.w =
//         init_pose_transform.getRotation().w();
//     request.request.initial_pose = cartographer_init_pose;

//     while (ros::ok()) {
//       ZINFO << "Call for resume slam.";
//       if (!cartographer_pause_resume_client_->call(request)) {
//         ZERROR << "Call service failed.";
//         Time::SleepSec(1);
//         continue;
//       }
//       break;
//     }
//     ZINFO << "Call for resume slam finish.";

//     {
//       WriteLocker lock(access_);
//       start_or_resume_timestamp_ = Time::Now();
//     }

//     is_running_.store(true);

//     return true;
//   }

//  private:
//   ros::ServiceClient *cartographer_start_trajectory_client_;
//   ros::ServiceClient *cartographer_stop_client_;
//   ros::ServiceClient *cartographer_pause_resume_client_;
// };

ZimaNode::ZimaNode(const bool &use_simple_slam, const bool &use_sim_time)
    : use_sim_time_(use_sim_time),
      operation_data_access_(std::make_shared<ReadWriteLock>()),
      operation_data_(nullptr),
      cmd_lock_(std::make_shared<ReadWriteLock>()),
      use_simple_slam_(use_simple_slam),
      shutdown_request_(false) {
  ZINFO << GetROSVersionInfo();
}

ZimaNode::~ZimaNode(){};

void ZimaNode::InitForChassis() {
  chassis_.reset(new Chassis(Chassis::Config()));
  chassis_->Initialize();
}

void ZimaNode::InitForAutoCleaning() {
  operation_data_ = std::make_shared<OperationData>();
}

void ZimaNode::InitForRos() {
  odom_msg_access_ = std::make_shared<ReadWriteLock>();
  odom_msg_template_ = nullptr;
  publish_scan_msg_access_ = std::make_shared<ReadWriteLock>();
  publish_scan_msg_ = nullptr;

  cleaning_info_marker_publisher_ =
      node.advertise<visualization_msgs::Marker>("/zima_cleaning_info", 5);
  map_marker_publisher_ =
      node.advertise<visualization_msgs::Marker>("/zima_map", 5);
  plan_path_marker_publisher_ =
      node.advertise<visualization_msgs::Marker>("/zima_plan_path", 5);
  path_marker_publisher_ =
      node.advertise<visualization_msgs::Marker>("/zima_path", 5);
  robot_marker_publisher_ =
      node.advertise<visualization_msgs::MarkerArray>("/zima_robot_marker", 5);
  robot_sensor_marker_publisher_ =
      node.advertise<visualization_msgs::Marker>("/zima_robot_sensor_marker", 5);
  odom_publisher_ = node.advertise<nav_msgs::Odometry>("/zima_odom", 5);
  scan_publisher_ = node.advertise<sensor_msgs::LaserScan>("/zima_scan", 5);
  slam_value_map_publisher_ =
      node.advertise<nav_msgs::OccupancyGrid>("/zima_slam_map", 5);
  point_cloud_in_chassis_frame_publisher_ =
      node.advertise<visualization_msgs::Marker>(
          "/zima_point_cloud_in_chassis_frame", 5);
  point_cloud_in_world_frame_publisher_ =
      node.advertise<visualization_msgs::Marker>(
          "/zima_point_cloud_in_world_frame", 5);
  line_segments_publisher_ =
      node.advertise<visualization_msgs::Marker>("/zima_line_segments", 5);

  mode_switch_subscriber_ =
      node.subscribe("/zima/mode_switch", 1, &ZimaNode::ModeSwitchCb, this);
  virtual_wall_subscriber_ = node.subscribe("/zima/set_virtual_wall", 1,
                                            &ZimaNode::SetVirtualWallCb, this);
  block_area_subscriber_ = node.subscribe("/zima/set_block_area", 1,
                                          &ZimaNode::SetBlockAreaCb, this);

  cache_slam_occupancy_grid_map_access_ = std::make_shared<ReadWriteLock>();
  cache_slam_occupancy_grid_map_ = nullptr;
  if (!use_simple_slam_) {
    slam_map_subscriber_ =
        node.subscribe("/map", 1, &ZimaNode::SlamMapCb, this);
  }

  sim_time_received_.store(false);

  // Cartographer_ros
  // cartographer_start_trajectory_client_ =
  //     node.serviceClient<cartographer_ros_msgs::StartTrajectory>(
  //         "/cartographer_ros/start_trajectory");
  // cartographer_stop_client_ =
  //     node.serviceClient<cartographer_ros_msgs::WriteState>(
  //         "/cartographer_ros/write_state");
  // cartographer_pause_resume_client_ =
  //     node.serviceClient<cartographer_ros_msgs::StartTrajectory>(
  //         "/cartographer_ros/pause_resume");
  slam_map_received_.store(false);

  if (!use_simple_slam_) {
    ZERROR << "Slam other than simple slam is not support anymore.";
    slam_wrapper_ = std::make_shared<SimpleSlam>(
        SimpleSlam::Config(), chassis_->GetLidar(chassis_->kLidar_)->GetTf());
    // slam_wrapper_ = std::make_shared<CartographerSlam>(
    //     &cartographer_start_trajectory_client_, &cartographer_stop_client_,
    //     &cartographer_pause_resume_client_);
  } else {
    slam_wrapper_ = std::make_shared<SimpleSlam>(
        SimpleSlam::Config(), chassis_->GetLidar(chassis_->kLidar_)->GetTf());
  }
}

void ZimaNode::PublishCleaningInfo(SlamValueGridMap2D::SPtr map,
                                   const float &area, const float &min) {
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  marker.ns = "info";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = (data_bound.GetMax().X() - data_bound.GetMin().X()) *
                   map->GetResolution() / 9;
  marker.scale.y = marker.scale.x;
  marker.scale.z = marker.scale.x;

  // marker.scale.x = 1;
  // marker.scale.y = 1;
  // marker.scale.z = 1;

  marker.pose.position.x = (data_bound.GetMax().X() + data_bound.GetMin().X()) /
                           2 * map->GetResolution();
  marker.pose.position.y = data_bound.GetMax().Y() * map->GetResolution() + 2;
  marker.pose.position.z = cleaning_info_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.a = 1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  marker.text = FloatToString(area, 2) + "m2  " + FloatToString(min, 2) +
                "mins\n" + FloatToString(area / min, 2) + "m2/min";

  cleaning_info_marker_publisher_.publish(marker);
}

void ZimaNode::PublishMap(CharGridMap2D::SPtr map,
                          const ZimaNode::MapType &type,
                          const float &x_offset) {
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  auto resolution = map->GetResolution();
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  switch (type) {
    case kPrintMap: {
      marker.id = 1;
      marker.ns = "print map";
      break;
    }
    case kVirtualObsMap: {
      marker.id = 2;
      marker.ns = "virtual obs map";
      break;
    }
    case kRoomMap: {
      marker.id = 3;
      marker.ns = "room map";
      break;
    }
    case kUserAreaMap: {
      marker.id = 4;
      marker.ns = "user area map";
      break;
    }
    case kSlamCharMap:
    default: {
      marker.id = 5;
      marker.ns = "slam char map";
      break;
    }
  }
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x_offset;
  marker.pose.position.y = 0;
  switch (type) {
    case kPrintMap: {
      marker.pose.position.z = print_map_layer_z;
      break;
    }
    case kVirtualObsMap: {
      marker.pose.position.z = virtual_obs_map_layer_z;
      break;
    }
    case kRoomMap: {
      marker.pose.position.z = room_map_layer_z;
      break;
    }
    case kUserAreaMap: {
      marker.pose.position.z = user_area_map_layer_z;
      break;
    }
    case kSlamCharMap:
    default: {
      marker.pose.position.z = slam_char_map_layer_z;
      break;
    }
  }
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution * 0.1;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      CharGridMap2D::DataType data = map->GetDefaultValue();
      map->GetValue(x, y, data);
      geometry_msgs::Point point;
      point.x = x * resolution;
      point.y = y * resolution;
      point.z = 0.0;
      std_msgs::ColorRGBA color;
      if (data == NavMap::kFootStep_ || data == NavMap::kRoomA_) {
        color.a = 0.7;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
      } else if (data == NavMap::kBumper_ || data == NavMap::kRoomB_ ||
                 data == NavMap::kStrictBlockArea_) {
        color.a = 0.7;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
      } else if (data == NavMap::kRoomC_ || data == NavMap::kVirtualWall_) {
        color.a = 0.7;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
      } else if (data == NavMap::kPath_ || data == NavMap::kRoomD_) {
        color.a = 0.7;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
      } else if (data == NavMap::kWall_ || data == NavMap::kSlamWall_) {
        color.a = 0.7;
        color.r = 42.0 / 255;
        color.g = 42.0 / 255;
        color.b = 42.0 / 255;
      } else if (data == NavMap::kSlamFloor_) {
        color.a = 0.7;
        color.r = 0.0 / 255;
        color.g = 170.0 / 255;
        color.b = 255.0 / 255;
      } else if (type == kSlamCharMap && data == NavMap::kUnknown_) {
        color.a = 0.7;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
      } else {
        color.a = 0.01;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
      }
      marker.points.emplace_back(point);
      marker.colors.emplace_back(color);
    }
  }
  // ZWARN << "marker size " << marker.points.size();

  if (!marker.points.empty()) {
    map_marker_publisher_.publish(marker);
  }
}

void ZimaNode::PublishUnderLayerMap(SlamValueGridMap2D::SPtr map) {
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  auto resolution = map->GetResolution();
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.ns = "under layer map";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = (data_bound.GetMax().X() - data_bound.GetMin().X()) *
                   map->GetResolution();
  auto y_offset = marker.scale.x / 3;
  marker.scale.y = (data_bound.GetMax().Y() - data_bound.GetMin().Y()) *
                       map->GetResolution() +
                   y_offset;
  marker.scale.z = resolution * 0.1;

  marker.pose.position.x = (data_bound.GetMax().X() + data_bound.GetMin().X()) /
                           2 * map->GetResolution();
  marker.pose.position.y = (data_bound.GetMax().Y() + data_bound.GetMin().Y()) /
                               2 * map->GetResolution() +
                           y_offset / 2;
  marker.pose.position.z = underlay_layer_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  map_marker_publisher_.publish(marker);
}

void ZimaNode::PublishPath(CharGridMap2D::SPtr map, const Steps &steps,
                           const float &x_offset) {
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  auto resolution = map->GetResolution();
  visualization_msgs::Marker marker;
  bool first_cell = true;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  marker.ns = "clean_path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x_offset;
  marker.pose.position.y = 0;
  marker.pose.position.z = path_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = resolution * 0.8;
  marker.scale.y = resolution * 0.2;
  marker.scale.z = resolution * 0.1;
  geometry_msgs::Point point;
  point.z = 0.0;
  std_msgs::ColorRGBA color;
  color.a = 1;
  for (auto &&step : steps) {
    point.x = step.Pose().X();
    point.y = step.Pose().Y();
    if (first_cell) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      first_cell = false;
    } else {
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
    }
    marker.points.emplace_back(point);
    marker.colors.emplace_back(color);
  }
  path_marker_publisher_.publish(marker);
}

void ZimaNode::PublishPlanPath(const MapCellPath &path,
                               const double &resolution,
                               const float &x_offset) {
  visualization_msgs::Marker marker;
  bool first_cell = true;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  marker.ns = "plan_path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = plan_path_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = resolution * 0.8;
  marker.scale.y = resolution * 0.8;
  marker.scale.z = resolution * 0.1;
  geometry_msgs::Point point;
  point.z = 0.0;
  std_msgs::ColorRGBA color;
  color.a = 1;
  for (auto &&cell : path) {
    point.x = cell.X() * resolution;
    point.y = cell.Y() * resolution;
    if (first_cell) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      first_cell = false;
    } else {
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
    }
    marker.points.emplace_back(point);
    marker.colors.emplace_back(color);
  }
  plan_path_marker_publisher_.publish(marker);
}

void ZimaNode::PublishRobot(const MapPoint &robot_pose) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  marker.ns = "robot_track";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = robot_pose.X();
  marker.pose.position.y = robot_pose.Y();
  marker.pose.position.z = robot_z;
  auto orientation = tf::createQuaternionFromYaw(robot_pose.Radian());
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  marker.scale.x = chassis_->GetTrackLength();
  marker.scale.y = chassis_->GetTrackLength();
  marker.scale.z = 0.01;
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker_array.markers.emplace_back(marker);
  marker.ns = "robot_shape";
  marker.id = 1;
  marker.header.stamp = ros::Time();
  marker.scale.x = chassis_->GetRadius() * 2;
  marker.scale.y = chassis_->GetRadius() * 2;
  marker_array.markers.emplace_back(marker);
  marker.ns = "robot_direction";
  marker.id = 2;
  marker.header.stamp = ros::Time();
  marker.scale.y = chassis_->GetTrackLength() / 2;
  marker.type = visualization_msgs::Marker::ARROW;
  marker_array.markers.emplace_back(marker);
  robot_marker_publisher_.publish(marker_array);
}

void ZimaNode::PublishWallSensor(const MapPoint &robot_pose) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  marker.header.stamp = ros::Time();
  marker.ns = "left wall sensor";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = robot_pose.X();
  marker.pose.position.y = robot_pose.Y();
  marker.pose.position.z = robot_z;
  auto wall_sensor = chassis_->GetWallSensor(chassis_->kLeftWallSensor_);
  auto degree = robot_pose.Degree() + wall_sensor->GetTf().Degree();
  auto orientation =
      tf::createQuaternionFromYaw(DegreesToRadians(NormalizeDegree(degree)));
  marker.scale.x = wall_sensor->GetDistance();
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  robot_sensor_marker_publisher_.publish(marker);

  marker.ns = "right wall sensor";
  // marker.id = 1;
  wall_sensor = chassis_->GetWallSensor(chassis_->kRightWallSensor_);
  degree = robot_pose.Degree() + wall_sensor->GetTf().Degree();
  orientation =
      tf::createQuaternionFromYaw(DegreesToRadians(NormalizeDegree(degree)));
  marker.scale.x = wall_sensor->GetDistance();
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  robot_sensor_marker_publisher_.publish(marker);
}

void ZimaNode::PublishOdom() {
  auto merged_odom_data = chassis_->GetMergedOdomData();
  if (merged_odom_data == nullptr) {
    return;
  }

  // ZINFO << "Publish odom " << merged_odom_data->GetPose().DebugString();

  auto odom_ros_tf = MapPointToTFTransform(merged_odom_data->GetPose());
  auto now = ros::Time(merged_odom_data->GetTimeStamp());
  auto &tf_manager = *TransformManager::Instance();
  PublishTf(zima_ros::GeometryTransformToStampedTransform(
      now, tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
      zima_ros::TFToGeometryMsgTransform(odom_ros_tf)));
  // ZINFO << "Publish odom " << std::setprecision(13) << now.toSec() << " "
  //       << merged_odom_data->GetPose().DebugString();

  if (use_simple_slam_) {
    return;
  }
  WriteLocker lock(odom_msg_access_);
  if (odom_msg_template_ == nullptr) {
    return;
  }
  odom_msg_template_->header.stamp = now;
  odom_msg_template_->header.frame_id = tf_manager.kOdomFrame_.substr(1);
  odom_msg_template_->child_frame_id = tf_manager.kRobotFrame_.substr(1);
  odom_msg_template_->pose.pose.position.x = odom_ros_tf.getOrigin().x();
  odom_msg_template_->pose.pose.position.y = odom_ros_tf.getOrigin().y();
  odom_msg_template_->pose.pose.position.z = 0;

  odom_msg_template_->pose.pose.orientation.x = odom_ros_tf.getRotation().x();
  odom_msg_template_->pose.pose.orientation.y = odom_ros_tf.getRotation().y();
  odom_msg_template_->pose.pose.orientation.z = odom_ros_tf.getRotation().z();
  odom_msg_template_->pose.pose.orientation.w = odom_ros_tf.getRotation().w();
  odom_publisher_.publish(*odom_msg_template_);
}

void ZimaNode::PublishScan() {
  WriteLocker lock(publish_scan_msg_access_);
  if (publish_scan_msg_ == nullptr) {
    return;
  }
  scan_publisher_.publish(*publish_scan_msg_);
  // ZINFO << "Publish scan: " << std::setprecision(13)
  //       << publish_scan_msg_->header.stamp.toSec();
  publish_scan_msg_.reset();
}

void ZimaNode::PublishSlamValueMap(const ros::Time &time,
                                   SlamValueGridMap2D::SPtr map) {
  slam_value_map_publisher_.publish(zima_ros::SlamValueGridMapToOccupancyGrid(
      TransformManager::Instance()->kWorldFrame_, time, map));//发布slam地图
}

void ZimaNode::PublishPointCloudInChassisFrame(
    const PointCloud::SPtr &point_cloud_in_chassis_frame) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kRobotFrame_;
  // marker.header.stamp = ros::Time();
  {
    ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
    marker.header.stamp = ros::Time(point_cloud_in_chassis_frame->GetTimeStamp());
    // ZINFO << "Publish at " << point_cloud_in_chassis_frame->GetTimeStamp();
  }
  marker.ns = "zima_scan_in_chassis_frame";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = robot_z;
  auto orientation = tf::createQuaternionFromYaw(0);
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  ReadLocker lock(point_cloud_in_chassis_frame->GetLock());

  std_msgs::ColorRGBA color;
  color.a = 0.5;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  for (auto &&_point : point_cloud_in_chassis_frame->GetPointsConstRef()) {
    geometry_msgs::Point point;
    // ZINFO << "Add for " << _point.DebugString();
    if (!isnanf(_point.X()) && !isnanf(_point.Y())) {
      point.x = _point.X();
      point.y = _point.Y();
      marker.points.emplace_back(point);
      marker.colors.emplace_back(color);
    }
  }
  // geometry_msgs::Point point;
  // point.x = 1;
  // point.y = 1;
  // marker.points.emplace_back(point);
  // marker.colors.emplace_back(color);
  // ZWARN << "marker size " << marker.points.size();

  point_cloud_in_chassis_frame_publisher_.publish(marker);
}

void ZimaNode::PublishPointCloudInWorldFrame(
    const PointCloud::SPtr &point_cloud_in_world_frame) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = TransformManager::Instance()->kWorldFrame_;
  // marker.header.stamp = ros::Time();
  {
    ReadLocker lock(point_cloud_in_world_frame->GetLock());
    marker.header.stamp = ros::Time(point_cloud_in_world_frame->GetTimeStamp());
    // ZINFO << "Publish at " << point_cloud_in_world_frame->GetTimeStamp();
  }
  marker.ns = "zima_scan_in_world_frame";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = robot_z;
  auto orientation = tf::createQuaternionFromYaw(0);
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  ReadLocker lock(point_cloud_in_world_frame->GetLock());

  std_msgs::ColorRGBA color;
  color.a = 0.5;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  for (auto &&_point : point_cloud_in_world_frame->GetPointsConstRef()) {
    geometry_msgs::Point point;
    // ZINFO << "Add for " << _point.DebugString();
    if (!isnanf(_point.X()) && !isnanf(_point.Y())) {
      point.x = _point.X();
      point.y = _point.Y();
      marker.points.emplace_back(point);
      marker.colors.emplace_back(color);
    }
  }
  // geometry_msgs::Point point;
  // point.x = 1;
  // point.y = 1;
  // marker.points.emplace_back(point);
  // marker.colors.emplace_back(color);
  // ZWARN << "marker size " << marker.points.size();

  point_cloud_in_world_frame_publisher_.publish(marker);
}

void ZimaNode::PublishLineSegments(const LineSegments &line_segments,
                                   const std::string &name_space) {
  visualization_msgs::Marker marker;
  if (line_segments.empty()) {
    return;
  }
  marker.header.frame_id = TransformManager::Instance()->kRobotFrame_;
  marker.header.stamp = ros::Time();
  marker.ns = name_space;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = robot_z;
  auto orientation = tf::createQuaternionFromYaw(0);
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  std_msgs::ColorRGBA color;
  color.a = 0.5;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  for (auto &&line : line_segments) {
    geometry_msgs::Point point;
    point.x = line.GetX1();
    point.y = line.GetY1();
    marker.points.emplace_back(point);
    marker.colors.emplace_back(color);
    point.x = line.GetX2();
    point.y = line.GetY2();
    marker.points.emplace_back(point);
    marker.colors.emplace_back(color);
  }

  line_segments_publisher_.publish(marker);
}

void ZimaNode::PublishTf(const geometry_msgs::TransformStamped &tf) {
  robot_tf_broadcaster_.sendTransform(tf);
}

void ZimaNode::SlamMapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  WriteLocker lock(cache_slam_occupancy_grid_map_access_);
  cache_slam_occupancy_grid_map_ = msg;
  // operation_data_->UpdateRawSlamValueGridMap2D(
  //     zima_ros::OccupancyGridToSlamValueGridMap("slam map", msg));
  // MultiResolutionSlamValueGridMap2D multi_resolution_slam_map(
  //     "Multi resolution slam map", slam_grid_map_, 7);
  slam_map_received_.store(true);
}

void ZimaNode::SetVirtualWallCb(
    const zima_ros::VirtualWall::ConstPtr &msg) {
  ZGINFO;
  WriteLocker lock(cmd_lock_);
  if (msg->cells_value.empty() && msg->points_value.size() != 4) {
    ZERROR << msg->points_value.size()
           << " value received, it should be 2 points with 4 value.";
    return;
  }
  if (msg->points_value.empty() && msg->cells_value.size() != 4) {
    ZERROR << msg->cells_value.size()
           << " value received, it should be 2 cells with 4 value.";
    return;
  }

  NavData::VirtualWall::SPtr virtual_wall = nullptr;
  if (!msg->cells_value.empty()) {
    virtual_wall = std::make_shared<NavData::VirtualWall>(
        MapPoint(msg->cells_value.at(0) * NavMap::GetResolution(),
                 msg->cells_value.at(1) * NavMap::GetResolution()),
        MapPoint(msg->cells_value.at(2) * NavMap::GetResolution(),
                 msg->cells_value.at(3) * NavMap::GetResolution()),
        msg->enable > 0);
  } else {
    virtual_wall = std::make_shared<NavData::VirtualWall>(
        MapPoint(msg->points_value.at(0), msg->points_value.at(1)),
        MapPoint(msg->points_value.at(2), msg->points_value.at(3)),
        msg->enable > 0);
  }

  VirtualWallBlockAreaEvent::Operation::List operation_list;
  NavData::VirtualWall::SPtr existing_virtual_wall = nullptr;

  ReadLocker operation_data_lock(operation_data_access_);
  if (operation_data_ == nullptr) {
    ZERROR << "Operation data not created now.";
    return;
  }

  if (operation_data_->GetVirtualWall(msg->index, existing_virtual_wall)) {
    if (msg->enable) {
      operation_list.emplace_back(
          VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall,
          VirtualWallBlockAreaEvent::Operation::OperationType::kUpdate,
          virtual_wall, nullptr, msg->index);
    } else {
      operation_list.emplace_back(
          VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall,
          VirtualWallBlockAreaEvent::Operation::OperationType::kRemove,
          virtual_wall, nullptr, msg->index);
    }
  } else {
    operation_list.emplace_back(
        VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall,
        VirtualWallBlockAreaEvent::Operation::OperationType::kAdd, virtual_wall,
        nullptr, msg->index);
  }
  operation_data_lock.Unlock();

  if (!operation_list.empty()) {
    auto &event_manager = *EventManager::Instance();
    event_manager.PushUserEvent(
        std::make_shared<VirtualWallBlockAreaEvent>(operation_list));
  }
}

void ZimaNode::SetBlockAreaCb(const zima_ros::BlockArea::ConstPtr &msg) {
  ZGINFO;
  WriteLocker lock(cmd_lock_);
  if (msg->cells_value.empty() && msg->points_value.size() != 8) {
    ZERROR << msg->points_value.size()
           << " value received, it should be 4 points with 8 value.";
    return;
  }
  if (msg->points_value.empty() && msg->cells_value.size() != 8) {
    ZERROR << msg->cells_value.size()
           << " value received, it should be 4 cells with 8 value.";
    return;
  }

  NavData::BlockArea::SPtr block_area = nullptr;
  if (!msg->cells_value.empty()) {
    block_area = std::make_shared<NavData::BlockArea>(
        MapPoint(msg->cells_value.at(0) * NavMap::GetResolution(),
                 msg->cells_value.at(1) * NavMap::GetResolution()),
        MapPoint(msg->cells_value.at(2) * NavMap::GetResolution(),
                 msg->cells_value.at(3) * NavMap::GetResolution()),
        MapPoint(msg->cells_value.at(4) * NavMap::GetResolution(),
                 msg->cells_value.at(5) * NavMap::GetResolution()),
        MapPoint(msg->cells_value.at(6) * NavMap::GetResolution(),
                 msg->cells_value.at(7) * NavMap::GetResolution()),
        msg->enable > 0, static_cast<NavData::UserBlockType>(msg->type));
  } else {
    block_area = std::make_shared<NavData::BlockArea>(
        MapPoint(msg->points_value.at(0), msg->points_value.at(1)),
        MapPoint(msg->points_value.at(2), msg->points_value.at(3)),
        MapPoint(msg->points_value.at(4), msg->points_value.at(5)),
        MapPoint(msg->points_value.at(6), msg->points_value.at(7)),
        msg->enable > 0, static_cast<NavData::UserBlockType>(msg->type));
  }

  VirtualWallBlockAreaEvent::Operation::List operation_list;
  NavData::BlockArea::SPtr existing_block_area = nullptr;

  ReadLocker operation_data_lock(operation_data_access_);
  if (operation_data_ == nullptr) {
    ZERROR << "Operation data not created now.";
    return;
  }

  if (operation_data_->GetBlockArea(msg->index, existing_block_area)) {
    if (msg->enable) {
      operation_list.emplace_back(
          VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall,
          VirtualWallBlockAreaEvent::Operation::OperationType::kUpdate, nullptr,
          block_area, msg->index);
    } else {
      operation_list.emplace_back(
          VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall,
          VirtualWallBlockAreaEvent::Operation::OperationType::kRemove, nullptr,
          block_area, msg->index);
    }
  } else {
    operation_list.emplace_back(
        VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall,
        VirtualWallBlockAreaEvent::Operation::OperationType::kAdd, nullptr,
        block_area, msg->index);
  }
  operation_data_lock.Unlock();

  if (!operation_list.empty()) {
    auto &event_manager = *EventManager::Instance();
    event_manager.PushUserEvent(
        std::make_shared<VirtualWallBlockAreaEvent>(operation_list));
  }
}

void ZimaNode::ModeSwitchCb(const zima_ros::ModeSwitch::ConstPtr &msg) {
  ZGINFO;
  WriteLocker lock(cmd_lock_);
  auto &event_manager = *EventManager::Instance();
  switch (msg->cmd) {
    case 1: {
      event_manager.PushUserEvent(std::make_shared<PauseEvent>());
      break;
    }
    case 2: {
      event_manager.PushUserEvent(std::make_shared<StopCleaningEvent>());
      break;
    }
    case 0: {
      event_manager.PushUserEvent(std::make_shared<StartCleaningEvent>());
      break;
    }
    default: {
      event_manager.PushUserEvent(std::make_shared<GeneralOperationEvent>());
      break;
    }
  }
}

void ZimaNode::ROSSpinThread(const ZimaThreadWrapper::ThreadParam &param) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  while (ros::ok()) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    ros::spinOnce();
    Time::SleepMSec(0.1);
  }

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void ZimaNode::TfThread(const ZimaThreadWrapper::ThreadParam &param) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  // For gazebo.
  if (use_sim_time_) {
    while (!shutdown_request_ && !sim_time_received_.load()) {
      thread_manager->UpdateThreadCycle(param.thread_name_);
      Time::SleepSec(0.05);
      continue;
    }
    ZGINFO << "Sim time received.";
  }

  tf::TransformListener listener;
  tf::StampedTransform correction;
  // tf::StampedTransform pose;
  MapPoint last_correction;
  auto &tf_manager = *TransformManager::Instance();
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    const float duration = 0.01;

    // Subscribe to external transform correction.
    if (!use_simple_slam_) {
      try {
        listener.waitForTransform(tf_manager.kWorldFrame_,
                                  tf_manager.kOdomFrame_, ros::Time(0),
                                  ros::Duration(0.2));
        listener.lookupTransform(tf_manager.kWorldFrame_.substr(1),
                                 tf_manager.kOdomFrame_.substr(1), ros::Time(0),
                                 correction);
        // listener.waitForTransform(tf_manager.kWorldFrame_,
        //                           tf_manager.kRobotFrame_,
        //                           ros::Time(0), ros::Duration(0.2));
        // listener.lookupTransform(tf_manager.kWorldFrame_.substr(1),
        //                           tf_manager.kRobotFrame_.substr(1),
        //                          ros::Time(0), pose);
      } catch (tf::TransformException &ex) {
        ZERROR << ex.what();
        Time::SleepSec(duration);
        continue;
      }
      // ZINFO << "Ros pose: " <<
      // zima_ros::TFTransformToMapPoint(pose).DebugString();
      auto correction_point = zima_ros::TFTransformToMapPoint(correction);
      if (!DoubleEqual(correction_point.X(), last_correction.X()) ||
          !DoubleEqual(correction_point.Y(), last_correction.Y()) ||
          !DoubleEqual(correction_point.Degree(), last_correction.Degree())) {
        // ZWARN << "Correction: " << correction_point.DebugString();
        Transform new_correction_tf(tf_manager.kWorldFrame_,
                                    tf_manager.kOdomFrame_,
                                    correction_point.X(), correction_point.Y(),
                                    correction_point.Degree());
        tf_manager.UpdateTransform(new_correction_tf);
        last_correction = correction_point;
      }
    } else {
      // Transform odom_tf("", "");
      // tf_manager.GetTransform(tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
      //                         odom_tf);
      // MapPoint odom_pose(odom_tf.X(), odom_tf.Y(), odom_tf.Degree());

      // // We hav P(world) as robot world pose, P(odom) as robot odom
      // // pose. And we need to calculate transform between world frame
      // // and odom frame.
      // // In order to use Transform::CoordinateTransformationXX
      // // function and for more comprehensible, firstly we inversly
      // // calculate the coordinate of origin point of odom frme on
      // // "pose frame". Then we get full transform chain from 'world'
      // // to 'pose' to 'odom'.
      // MapPoint odom_origin_on_odom_pose_coordinate;
      // {
      //   odom_origin_on_odom_pose_coordinate.SetDegree(-odom_pose.Degree());
      //   double x, y;
      //   Transform::CoordinateTransformationAB(
      //       0, 0, odom_pose.X(), odom_pose.Y(),
      //       DegreesToRadians(odom_pose.Degree()), x, y);
      //   odom_origin_on_odom_pose_coordinate.SetX(x);
      //   odom_origin_on_odom_pose_coordinate.SetY(y);
      // }
      // auto slam = dynamic_pointer_cast<SimpleSlam>(slam_wrapper_);
      // auto world_pose = slam->GetLastWorldPose();
      // ZINFO << "World pose: " << world_pose.DebugString();
      // MapPoint odom_origin_on_world_coordinate;
      // {
      //   odom_origin_on_world_coordinate.SetDegree(
      //       world_pose.Degree() +
      //       odom_origin_on_odom_pose_coordinate.Degree());
      //   double x, y;
      //   Transform::CoordinateTransformationBA(
      //       odom_origin_on_odom_pose_coordinate.X(),
      //       odom_origin_on_odom_pose_coordinate.Y(), world_pose.X(),
      //       world_pose.Y(), DegreesToRadians(world_pose.Degree()), x, y);
      //   odom_origin_on_world_coordinate.SetX(x);
      //   odom_origin_on_world_coordinate.SetY(y);
      // }

      // {
      //   auto correction = odom_origin_on_world_coordinate;
      //   // ZINFO << "correction: " << correction_.DebugString();
      //   Transform correction_tf(tf_manager.kWorldFrame_, tf_manager.kOdomFrame_,
      //                           correction.X(), correction.Y(),
      //                           correction.Degree());
      //   tf_manager.UpdateTransform(correction_tf);
      // }

      Transform correction_tf("", "");
      tf_manager.GetTransform(tf_manager.kWorldFrame_, tf_manager.kOdomFrame_,
                              correction_tf);
      MapPoint correction(correction_tf.X(), correction_tf.Y(),
                          correction_tf.Degree());
      auto correction_ros_tf = MapPointToTFTransform(correction);
      auto now = ros::Time::now();
      PublishTf(zima_ros::GeometryTransformToStampedTransform(
          now, tf_manager.kWorldFrame_, tf_manager.kOdomFrame_,
          zima_ros::TFToGeometryMsgTransform(correction_ros_tf)));
    }
    Time::SleepSec(duration);
  }

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void ZimaNode::PublishSensorDataThread(
    const ZimaThreadWrapper::ThreadParam &param) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  // For gazebo.
  if (use_sim_time_) {
    while (!shutdown_request_ && !sim_time_received_.load()) {
      thread_manager->UpdateThreadCycle(param.thread_name_);
      Time::SleepSec(0.05);
      continue;
    }
    ZGINFO << "Sim time received.";
  }

  uint32_t published_point_cloud_in_chassis_frame_index = 0;
  uint32_t published_point_cloud_in_world_frame_index = 0;
  const float duration = 0.02;
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    Time::SleepSec(duration);
    if (slam_wrapper_->IsRunning()) {
      PublishOdom();
      if (Time::Now() - slam_wrapper_->GetStartOrResumeTimeStamp() > 0.4) {
        PublishScan();
      }

      if (FLAGS_debug_enable) {
        uint32_t seq = 0;
        if (chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudSeq(seq) &&
            seq != published_point_cloud_in_chassis_frame_index) {
          auto point_cloud = chassis_->GetLidar(chassis_->kLidar_)
                                 ->GetPointCloudInChassisFrame();
          if (point_cloud != nullptr) {
            PublishPointCloudInChassisFrame(point_cloud);
            // ZINFO << "Publish point cloud.";
          }
          published_point_cloud_in_chassis_frame_index = seq;
        }

        if (use_simple_slam_) {
          auto point_cloud =
              slam_wrapper_->GetLastMatchPointCloudInWorldFrame();
          if (point_cloud != nullptr &&
              point_cloud->GetSeq() !=
                  published_point_cloud_in_world_frame_index) {
            PublishPointCloudInWorldFrame(point_cloud);
            published_point_cloud_in_world_frame_index = point_cloud->GetSeq();
          }
        }
      }
    }
  }

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void ZimaNode::DataProcessThread(const ZimaThreadWrapper::ThreadParam &param) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  // For gazebo.
  if (use_sim_time_) {
    while (!shutdown_request_ && !sim_time_received_.load()) {
      thread_manager->UpdateThreadCycle(param.thread_name_);
      Time::SleepSec(0.05);
      continue;
    }
    ZGINFO << "Sim time received.";
  }

  uint32_t map_seq = 0;

  std::shared_ptr<Timer> process_map_timer(
      new Timer("process data timer", 2, true, true));

  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    const float duration = 0.001;
    Time::SleepSec(duration);

    if (!ProcessData(map_seq, process_map_timer)) {
    }
  }

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void ZimaNode::PublishForRvizThread(
    const ZimaThreadWrapper::ThreadParam &param) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  // For gazebo.
  if (use_sim_time_) {
    while (!shutdown_request_ && !sim_time_received_.load()) {
      thread_manager->UpdateThreadCycle(param.thread_name_);
      Time::SleepSec(0.05);
      continue;
    }
    ZGINFO << "Sim time received.";
  }

  auto &tf = *TransformManager::Instance();

  Timer publish_map_timer("publish map timer", 5, true, true, true);
  Timer publish_info_timer("Publish info timer", 5, true, true, true);
  Timer debug_memory_use_timer("Debug memory use", 3, true, true, true);
  Timer debug_map_timer("debug map timer", 20, true, true, true);

  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    const float duration = 0.02;
    Time::SleepSec(duration);

    if (publish_info_timer.TimeUp()) {
      ReadLocker lock(operation_data_access_);
      auto operation_data = operation_data_;
      lock.Unlock();
      if (operation_data != nullptr) {
        ZINFO << operation_data->DebugOperationInfo();
        if (FLAGS_debug_enable) {
          PublishMap(operation_data->GetNavMapRef()->GetPrintLayer(),
                     MapType::kPrintMap);
          Time::SleepMSec(50);
        }
        PublishMap(operation_data->GetNavMapRef()->GetUserBlockLayer(),
                   MapType::kUserAreaMap);
        Time::SleepMSec(50);
        auto all_steps = operation_data->GetAllSteps();
        PublishPath(operation_data->GetNavMapRef()->GetPrintLayer(), all_steps);
        Time::SleepMSec(50);
        auto step_area = operation_data->GetNavMapRef()->GetStepAreaSize();
        PublishCleaningInfo(operation_data->GetOptimizedSlamValueGridMap2DRef(),
                            step_area, operation_data->GetDuration() / 60);
        PublishUnderLayerMap(
            operation_data->GetOptimizedSlamValueGridMap2DRef());
      }
      publish_info_timer.Reset();
    }

    if (publish_map_timer.TimeUp()) {
      ReadLocker lock(operation_data_access_);
      auto operation_data = operation_data_;
      lock.Unlock();

      if (operation_data != nullptr) {
        PublishMap(operation_data->GetNavMapConstRef()->GetSlamLayer(),
                   MapType::kSlamCharMap);
        Time::SleepMSec(50);
        PublishMap(operation_data->GetNavMapConstRef()->GetRoomLayer(),
                   MapType::kRoomMap);
        Time::SleepMSec(50);

        if (FLAGS_debug_enable) {
          PublishSlamValueMap(ros::Time(),
                              operation_data->GetRawSlamValueGridMap2DRef());
          Time::SleepMSec(50);
        }
      }
      publish_map_timer.Reset();
    }

    if (debug_memory_use_timer.TimeUp()) {
      ZINFO << "Current memory usage: "
            << std::to_string(ZimaGetProcessMemoryUsageInKB()) << " Kb.";
      ZGERROR << "Current memory usage: "
              << std::to_string(ZimaGetProcessMemoryUsageInKB()) << " Kb.";
      debug_memory_use_timer.Reset();
    }

    if (!FLAGS_debug_enable && debug_map_timer.TimeUp()) {
      ReadLocker lock(operation_data_access_);
      if (operation_data_ != nullptr) {
        operation_data_->GetOptimizedSlamValueGridMap2DRef()->Print(
            __FILE__, __FUNCTION__, __LINE__);
      }
      debug_map_timer.Reset();
    }

    Transform robot_tf("", "");
    tf.GetTransform(tf.kWorldFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint world_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint odom_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());

    PublishRobot(world_pose);
    PublishWallSensor(world_pose);
  }

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

bool ZimaNode::ProcessData(uint32_t &map_seq,
                           std::shared_ptr<Timer> &process_map_timer) {
  bool ret = true;
  return ret;
}

void ZimaNode::Shutdown() {
  ZWARN << "Shutdown.";
  shutdown_request_ = true;
}

}  // namespace zima_ros
