#include "mav_planning_rviz/planning_panel.h"

#include <stdio.h>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
// #include <mav_planning_msgs/PlannerService.h>
// #include <ros/names.h>
#include <mavros_msgs/srv/set_mode.hpp>
#include <planner_msgs/msg/navigation_status.hpp>
#include <planner_msgs/srv/set_planner_state.hpp>
#include <planner_msgs/srv/set_service.hpp>
#include <planner_msgs/srv/set_string.hpp>
#include <planner_msgs/srv/set_vector3.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <std_srvs/srv/empty.hpp>

#include "mav_planning_rviz/edit_button.h"
#include "mav_planning_rviz/goal_marker.h"
#include "mav_planning_rviz/pose_widget.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace mav_planning_rviz {

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      node_(std::make_shared<rclcpp::Node>("mav_planning_rviz")),
      interactive_markers_(node_) {
  createLayout();
  goal_marker_ = std::make_shared<GoalMarker>(node_);
  planner_state_sub_ = node_->create_subscription<planner_msgs::msg::NavigationStatus>(
      "/planner_status", 1, std::bind(&PlanningPanel::plannerstateCallback, this, _1));
}

void PlanningPanel::onInitialize() {
  interactive_markers_.initialize();
  interactive_markers_.setPoseUpdatedCallback(std::bind(&PlanningPanel::updateInteractiveMarkerPose, this, _1));

  interactive_markers_.setFrameId(getDisplayContext()->getFixedFrame().toStdString());
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
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "Setting namespace from: " << namespace_.toStdString() << " to " << new_namespace.toStdString());
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();

    std::string error;
    //! @todo(srmainwaring) port to ROS 2
    // if (ros::names::validate(namespace_.toStdString(), error))
    {
      waypoint_pub_ =
          node_->create_publisher<geometry_msgs::msg::PoseStamped>(namespace_.toStdString() + "/waypoint", 1);
      controller_pub_ =
          node_->create_publisher<geometry_msgs::msg::PoseStamped>(namespace_.toStdString() + "/command/pose", 1);
      odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          std::bind(&PlanningPanel::odometryCallback, this, _1));
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
  std::thread t([this, service_name, new_planner_name, align_terrain] {
    auto client = node_->create_client<planner_msgs::srv::SetString>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetString::Request>();
    req->string = new_planner_name;
    req->align = align_terrain;

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger, "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
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
    //! @todo(srmainwaring) port to ROS 2
    // if (ros::names::validate(namespace_.toStdString(), error))
    {
      odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          std::bind(&PlanningPanel::odometryCallback, this, _1));
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
  interactive_markers_.setFrameId(getDisplayContext()->getFixedFrame().toStdString());
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
  rclcpp::spin_some(node_);
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
void PlanningPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("planning_budget", planning_budget_value_);
  config.mapSetValue("odometry_topic", odometry_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
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
  std::thread t([this, service_name] {
    auto client = node_->create_client<mavros_msgs::srv::SetMode>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";
    //! @todo(srmainwaring) for AP custom mode is "GUIDED".
    // req->custom_mode = "GUIDED";

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::callPublishPath() {
  std::string service_name = namespace_.toStdString() + "/" + planner_name_.toStdString() + "/publish_path";
  auto client = node_->create_client<std_srvs::srv::Empty>(service_name);
  if (!client->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  auto result = client->async_send_request(req);

  //! @todo(srmainwaring) prevent race condition with async service calls
  const std::lock_guard<std::mutex> lock(node_mutex_);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    return;
  }

  // try {
  //   if (!ros::service::call(service_name, req)) {
  //     RCLCPP_WARN_STREAM(node_->get_logger(), "Couldn't call service: " << service_name);
  //   }
  // } catch (const std::exception& e) {
  //   RCLCPP_ERROR_STREAM(node_->get_logger(), "Service Exception: " << e.what());
  // }
}

void PlanningPanel::publishWaypoint() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([this, service_name] {
    auto client = node_->create_client<mavros_msgs::srv::SetMode>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "AUTO.RTL";

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
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
  std::thread t([this, service_name, new_planner_name, align_terrain] {
    auto client = node_->create_client<planner_msgs::srv::SetString>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetString::Request>();
    req->string = new_planner_name;
    req->align = align_terrain;

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setGoalService() {
  std::string service_name = "/terrain_planner/set_goal";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  std::thread t([this, service_name, goal_pos, goal_altitude] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = goal_pos(0);
    req->vector.y = goal_pos(1);
    req->vector.z = goal_altitude;

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setPathService() {
  std::string service_name = "/terrain_planner/set_path";
  std::cout << "Planner Service" << std::endl;
  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
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

  std::thread t([this, service_name, planning_budget] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    // if ()
    req->vector.z = planning_budget;

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
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
  std::thread t([this, service_name, mode] {
    auto client = node_->create_client<planner_msgs::srv::SetPlannerState>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetPlannerState::Request>();
    req->state = mode;

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setStartService() {
  std::string service_name = "/terrain_planner/set_start";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  std::thread t([this, service_name, goal_pos, goal_altitude] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = goal_pos(0);
    req->vector.y = goal_pos(1);
    req->vector.z = goal_altitude;

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setStartLoiterService() {
  std::string service_name = "/terrain_planner/set_start_loiter";

  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setCurrentSegmentService() {
  std::string service_name = "/terrain_planner/set_current_segment";

  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();

    auto result = client->async_send_request(req);

    //! @todo(srmainwaring) prevent race condition with async service calls
    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::publishToController() {
  mav_msgs::EigenTrajectoryPoint goal_point;
  goal_pose_widget_->getPose(&goal_point);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = getDisplayContext()->getFixedFrame().toStdString();
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point, &pose);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Publishing controller goal on "
                                               << controller_pub_->get_topic_name()
                                               << " subscribers: " << controller_pub_->get_subscription_count());

  controller_pub_->publish(pose);
}

void PlanningPanel::odometryCallback(const nav_msgs::msg::Odometry& msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry callback.");
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

void PlanningPanel::plannerstateCallback(const planner_msgs::msg::NavigationStatus& msg) {
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
      set_planner_state_buttons_[3]->setDisabled(false);  // RETURN
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

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanel, rviz_common::Panel)
