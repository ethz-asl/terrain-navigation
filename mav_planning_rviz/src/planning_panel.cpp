#include <stdio.h>
#include <functional>
#include <thread>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>
// #include <mav_planning_msgs/PlannerService.h>
#include <ros/names.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>

#include <mavros_msgs/SetMode.h>
#include <planner_msgs/NavigationStatus.h>
#include <planner_msgs/SetPlannerState.h>
#include <planner_msgs/SetService.h>
#include <planner_msgs/SetString.h>
#include <planner_msgs/SetVector3.h>

#include "mav_planning_rviz/edit_button.h"
#include "mav_planning_rviz/goal_marker.h"
#include "mav_planning_rviz/planning_panel.h"
#include "mav_planning_rviz/pose_widget.h"

namespace mav_planning_rviz {

PlanningPanel::PlanningPanel(QWidget* parent) : rviz::Panel(parent), nh_(ros::NodeHandle()), interactive_markers_(nh_) {
  createLayout();
  goal_marker_ = std::make_shared<GoalMarker>(nh_);
  planner_state_sub_ = nh_.subscribe("/planner_status", 1, &PlanningPanel::plannerstateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
}

void PlanningPanel::onInitialize() {
  interactive_markers_.initialize();
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this, std::placeholders::_1));

  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto& kv : pose_widget_map_) {
    mav_msgs::EigenTrajectoryPoint pose;
    kv.second->getPose(&pose);
    interactive_markers_.enableMarker(kv.first, pose);
  }
}

void PlanningPanel::createLayout() {
  QGridLayout* service_layout = new QGridLayout;

  // Planner services and publications.
  service_layout->addWidget(createTerrainLoaderGroup(), 0, 0, 1, 1);
  service_layout->addWidget(createPlannerCommandGroup(), 1, 0, 1, 1);
  service_layout->addWidget(createPlannerModeGroup(), 2, 0, 4, 1);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(service_layout);
  setLayout(layout);
}

QGroupBox* PlanningPanel::createPlannerModeGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Planner Actions"));
  QGridLayout* service_layout = new QGridLayout;
  set_planner_state_buttons_.push_back(new QPushButton("NAVIGATE"));
  set_planner_state_buttons_.push_back(new QPushButton("ROLLOUT"));
  set_planner_state_buttons_.push_back(new QPushButton("ABORT"));
  set_planner_state_buttons_.push_back(new QPushButton("RETURN"));

  service_layout->addWidget(set_planner_state_buttons_[0], 0, 0, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[1], 0, 1, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[3], 0, 2, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[2], 0, 3, 1, 1);
  groupBox->setLayout(service_layout);

  connect(set_planner_state_buttons_[0], SIGNAL(released()), this, SLOT(setPlannerModeServiceNavigate()));
  connect(set_planner_state_buttons_[1], SIGNAL(released()), this, SLOT(setPlannerModeServiceRollout()));
  connect(set_planner_state_buttons_[2], SIGNAL(released()), this, SLOT(setPlannerModeServiceAbort()));
  connect(set_planner_state_buttons_[3], SIGNAL(released()), this, SLOT(setPlannerModeServiceReturn()));

  return groupBox;
}

QGroupBox* PlanningPanel::createPlannerCommandGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Set Planner Problem"));
  QGridLayout* service_layout = new QGridLayout;

  planner_service_button_ = new QPushButton("Engage Planner");
  set_goal_button_ = new QPushButton("Update Goal");
  set_start_button_ = new QPushButton("Update Start");
  set_current_loiter_button_ = new QPushButton("Loiter Start");
  set_current_segment_button_ = new QPushButton("Current Segment");
  trigger_planning_button_ = new QPushButton("Plan");
  update_path_button_ = new QPushButton("Update Path");
  planning_budget_editor_ = new QLineEdit;
  max_altitude_button_enable_ = new QPushButton("Enable Max altitude");
  max_altitude_button_disable_ = new QPushButton("Disable Max altitude");

  waypoint_button_ = new QPushButton("Disengage Planner");
  controller_button_ = new QPushButton("Send To Controller");

  // Input the namespace.
  service_layout->addWidget(set_start_button_, 0, 0, 1, 1);
  service_layout->addWidget(set_goal_button_, 0, 1, 1, 1);

  service_layout->addWidget(set_current_loiter_button_, 0, 2, 1, 1);
  service_layout->addWidget(set_current_segment_button_, 0, 3, 1, 1);

  service_layout->addWidget(new QLabel("Planning budget:"), 2, 0, 1, 1);
  service_layout->addWidget(planning_budget_editor_, 2, 1, 1, 1);
  service_layout->addWidget(trigger_planning_button_, 2, 2, 1, 2);
  service_layout->addWidget(new QLabel("Max Altitude Constraints:"), 3, 0, 1, 1);
  service_layout->addWidget(max_altitude_button_enable_, 3, 1, 1, 1);
  service_layout->addWidget(max_altitude_button_disable_, 3, 2, 1, 1);
  service_layout->addWidget(planner_service_button_, 4, 0, 1, 2);
  service_layout->addWidget(waypoint_button_, 4, 2, 1, 2);

  groupBox->setLayout(service_layout);

  // Hook up connections.
  connect(planner_service_button_, SIGNAL(released()), this, SLOT(callPlannerService()));
  connect(set_goal_button_, SIGNAL(released()), this, SLOT(setGoalService()));
  connect(update_path_button_, SIGNAL(released()), this, SLOT(setPathService()));
  connect(set_start_button_, SIGNAL(released()), this, SLOT(setStartService()));
  connect(set_current_loiter_button_, SIGNAL(released()), this, SLOT(setStartLoiterService()));
  connect(set_current_segment_button_, SIGNAL(released()), this, SLOT(setCurrentSegmentService()));
  connect(waypoint_button_, SIGNAL(released()), this, SLOT(publishWaypoint()));
  connect(planning_budget_editor_, SIGNAL(editingFinished()), this, SLOT(updatePlanningBudget()));
  connect(trigger_planning_button_, SIGNAL(released()), this, SLOT(setPlanningBudgetService()));
  connect(max_altitude_button_enable_, SIGNAL(released()), this, SLOT(EnableMaxAltitude()));
  connect(max_altitude_button_disable_, SIGNAL(released()), this, SLOT(DisableMaxAltitude()));
  connect(controller_button_, SIGNAL(released()), this, SLOT(publishToController()));
  connect(terrain_align_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(terrainAlignmentStateChanged(int)));

  return groupBox;
}

QGroupBox* PlanningPanel::createTerrainLoaderGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Terrain Loader"));
  QGridLayout* service_layout = new QGridLayout;
  // Input the namespace.
  service_layout->addWidget(new QLabel("Terrain Location:"), 0, 0, 1, 1);
  planner_name_editor_ = new QLineEdit;
  service_layout->addWidget(planner_name_editor_, 0, 1, 1, 1);
  terrain_align_checkbox_ = new QCheckBox("Virtual Terrain");
  service_layout->addWidget(terrain_align_checkbox_, 0, 2, 1, 1);
  load_terrain_button_ = new QPushButton("Load Terrain");
  service_layout->addWidget(load_terrain_button_, 0, 3, 1, 1);

  connect(planner_name_editor_, SIGNAL(editingFinished()), this, SLOT(updatePlannerName()));
  connect(load_terrain_button_, SIGNAL(released()), this, SLOT(setPlannerName()));

  groupBox->setLayout(service_layout);

  return groupBox;
}

void PlanningPanel::terrainAlignmentStateChanged(int state) {
  if (state == 0) {
    align_terrain_on_load_ = 1;
  } else {
    align_terrain_on_load_ = 0;
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setNamespace(const QString& new_namespace) {
  ROS_DEBUG_STREAM("Setting namespace from: " << namespace_.toStdString() << " to " << new_namespace.toStdString());
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(namespace_.toStdString() + "/waypoint", 1, false);
      controller_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(namespace_.toStdString() + "/command/pose", 1, false);
      odometry_sub_ = nh_.subscribe(namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
                                    &PlanningPanel::odometryCallback, this);
    }
  }
}

void PlanningPanel::updatePlannerName() {
  QString new_planner_name = planner_name_editor_->text();
  std::cout << "New Terrain name: " << new_planner_name.toStdString() << std::endl;
  if (new_planner_name != planner_name_) {
    planner_name_ = new_planner_name;
    Q_EMIT configChanged();
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setPlannerName() {
  std::cout << "[PlanningPanel] Loading new terrain:" << planner_name_.toStdString() << std::endl;
  // Load new environment using a service
  std::string service_name = "/terrain_planner/set_location";
  std::string new_planner_name = planner_name_.toStdString();
  bool align_terrain = align_terrain_on_load_;
  std::thread t([service_name, new_planner_name, align_terrain] {
    planner_msgs::SetString req;
    req.request.string = new_planner_name;
    req.request.align = align_terrain;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::updatePlanningBudget() { setPlanningBudget(planning_budget_editor_->text()); }

void PlanningPanel::setPlanningBudget(const QString& new_planning_budget) {
  if (new_planning_budget != planning_budget_value_) {
    planning_budget_value_ = new_planning_budget;
    Q_EMIT configChanged();
  }
}
// void PlanningPanel::updateOdometryTopic() { setOdometryTopic(odometry_topic_editor_->text()); }

// Set the topic name we are publishing to.
void PlanningPanel::setOdometryTopic(const QString& new_odometry_topic) {
  // Only take action if the name has changed.
  if (new_odometry_topic != odometry_topic_) {
    odometry_topic_ = new_odometry_topic;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      odometry_sub_ = nh_.subscribe(namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
                                    &PlanningPanel::odometryCallback, this);
    }
  }
}

void PlanningPanel::startEditing(const std::string& id) {
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
  interactive_markers_.enableSetPoseMarker(pose);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string& id) {
  if (currently_editing_ == id) {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
  ros::spinOnce();
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget) {
  pose_widget_map_[widget->id()] = widget;
  connect(widget, SIGNAL(poseUpdated(const std::string&, mav_msgs::EigenTrajectoryPoint&)), this,
          SLOT(widgetPoseUpdated(const std::string&, mav_msgs::EigenTrajectoryPoint&)));
}

void PlanningPanel::registerEditButton(EditButton* button) {
  edit_button_map_[button->id()] = button;
  connect(button, SIGNAL(startedEditing(const std::string&)), this, SLOT(startEditing(const std::string&)));
  connect(button, SIGNAL(finishedEditing(const std::string&)), this, SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("planning_budget", planning_budget_value_);
  config.mapSetValue("odometry_topic", odometry_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString topic;
  QString ns;
  if (config.mapGetString("planner_name", &planner_name_)) {
    planner_name_editor_->setText(planner_name_);
  }
  if (config.mapGetString("planning_budget", &planning_budget_value_)) {
    planning_budget_editor_->setText(planning_budget_value_);
  }
}

void PlanningPanel::updateInteractiveMarkerPose(const mav_msgs::EigenTrajectoryPoint& pose) {
  if (currently_editing_.empty()) {
    return;
  }
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string& id, mav_msgs::EigenTrajectoryPoint& pose) {
  if (currently_editing_ == id) {
    interactive_markers_.setPose(pose);
  }
  interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::callPlannerService() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([service_name] {
    mavros_msgs::SetMode req;
    req.request.custom_mode = "OFFBOARD";

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::callPublishPath() {
  std_srvs::Empty req;
  std::string service_name = namespace_.toStdString() + "/" + planner_name_.toStdString() + "/publish_path";
  try {
    if (!ros::service::call(service_name, req)) {
      ROS_WARN_STREAM("Couldn't call service: " << service_name);
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
  }
}

void PlanningPanel::publishWaypoint() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([service_name] {
    mavros_msgs::SetMode req;
    req.request.custom_mode = "AUTO.RTL";

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::EnableMaxAltitude() { setMaxAltitudeConstrant(true); }

void PlanningPanel::DisableMaxAltitude() { setMaxAltitudeConstrant(false); }

void PlanningPanel::setMaxAltitudeConstrant(bool set_constraint) {
  std::cout << "[PlanningPanel] Loading new terrain:" << planner_name_.toStdString() << std::endl;
  // Load new environment using a service
  std::string service_name = "/terrain_planner/set_max_altitude";
  std::string new_planner_name = "";
  bool align_terrain = set_constraint;
  std::thread t([service_name, new_planner_name, align_terrain] {
    planner_msgs::SetString req;
    req.request.string = new_planner_name;
    req.request.align = align_terrain;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setGoalService() {
  std::string service_name = "/terrain_planner/set_goal";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  std::thread t([service_name, goal_pos, goal_altitude] {
    planner_msgs::SetVector3 req;
    req.request.vector.x = goal_pos(0);
    req.request.vector.y = goal_pos(1);
    // if ()
    req.request.vector.z = goal_altitude;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setPathService() {
  std::string service_name = "/terrain_planner/set_path";
  std::cout << "Planner Service" << std::endl;
  std::thread t([service_name] {
    planner_msgs::SetVector3 req;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setPlanningBudgetService() {
  std::string service_name = "/terrain_planner/trigger_planning";
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double planning_budget{-1.0};

  try {
    planning_budget = std::stod(planning_budget_value_.toStdString());
    std::cout << "[PlanningPanel] Set Planning Budget: " << planning_budget << std::endl;
  } catch (const std::exception& e) {
    std::cout << "[PlanningPanel] InvalidPlanning Budget: " << e.what() << std::endl;
  }

  std::thread t([service_name, planning_budget] {
    planner_msgs::SetVector3 req;
    // if ()
    req.request.vector.z = planning_budget;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setPlannerModeServiceNavigate() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", 2);
}

void PlanningPanel::setPlannerModeServiceAbort() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", 4);
}

void PlanningPanel::setPlannerModeServiceReturn() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", 5);
}

void PlanningPanel::setPlannerModeServiceRollout() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", 3);
}

void PlanningPanel::callSetPlannerStateService(std::string service_name, const int mode) {
  std::thread t([service_name, mode] {
    planner_msgs::SetPlannerState req;
    req.request.state = mode;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setStartService() {
  std::string service_name = "/terrain_planner/set_start";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  std::thread t([service_name, goal_pos, goal_altitude] {
    planner_msgs::SetVector3 req;
    req.request.vector.x = goal_pos(0);
    req.request.vector.y = goal_pos(1);
    // if ()
    req.request.vector.z = goal_altitude;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setStartLoiterService() {
  std::string service_name = "/terrain_planner/set_start_loiter";

  std::thread t([service_name] {
    planner_msgs::SetService req;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::setCurrentSegmentService() {
  std::string service_name = "/terrain_planner/set_current_segment";

  std::thread t([service_name] {
    planner_msgs::SetService req;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::publishToController() {
  mav_msgs::EigenTrajectoryPoint goal_point;
  goal_pose_widget_->getPose(&goal_point);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = vis_manager_->getFixedFrame().toStdString();
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point, &pose);

  ROS_DEBUG_STREAM("Publishing controller goal on " << controller_pub_.getTopic()
                                                    << " subscribers: " << controller_pub_.getNumSubscribers());

  controller_pub_.publish(pose);
}

void PlanningPanel::odometryCallback(const nav_msgs::Odometry& msg) {
  ROS_INFO_ONCE("Got odometry callback.");
  if (align_terrain_on_load_) {
    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(msg, &odometry);
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = odometry.position_W;
    point.orientation_W_B = odometry.orientation_W_B;
    pose_widget_map_["start"]->setPose(point);
    interactive_markers_.updateMarkerPose("start", point);
  }
}

void PlanningPanel::plannerstateCallback(const planner_msgs::NavigationStatus& msg) {
  switch (msg.state) {
    case PLANNER_STATE::HOLD: {
      set_planner_state_buttons_[0]->setDisabled(false);  // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(false);  // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(true);   // ABORT
      set_planner_state_buttons_[3]->setDisabled(false);  // RETURN
      break;
    }
    case PLANNER_STATE::NAVIGATE: {
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);  // RETURN
      break;
    }
    case PLANNER_STATE::ROLLOUT: {
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);   // RETURN
      break;
    }
    case PLANNER_STATE::ABORT: {
      set_planner_state_buttons_[0]->setDisabled(true);  // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);  // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(true);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);  // RETURN
      break;
    }
    case PLANNER_STATE::RETURN: {
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);   // RETURN
      break;
    }
  }
}

}  // namespace mav_planning_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanel, rviz::Panel)
