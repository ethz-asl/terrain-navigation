#ifndef MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
#define MAV_PLANNING_RVIZ_PLANNING_PANEL_H_

#ifndef Q_MOC_RUN
//! @todo(srmainwaring) prevent race condition with async service calls
#include <QGroupBox>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/navigation_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "mav_planning_rviz/edit_button.h"
#include "mav_planning_rviz/goal_marker.h"
#include "mav_planning_rviz/planning_interactive_markers.h"
#include "mav_planning_rviz/pose_widget.h"
#endif

enum PLANNER_STATE { HOLD = 1, NAVIGATE = 2, ROLLOUT = 3, ABORT = 4, RETURN = 5 };

class QLineEdit;
class QCheckBox;
namespace mav_planning_rviz {

class PlanningPanel : public rviz_common::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
 public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit PlanningPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;
  virtual void onInitialize();

  // All the settings to manage pose <-> edit mapping.
  void registerPoseWidget(PoseWidget* widget);
  void registerEditButton(EditButton* button);

  // Callback from ROS when the pose updates:
  void updateInteractiveMarkerPose(const mav_msgs::EigenTrajectoryPoint& pose);
  // And when we get robot odometry:
  void odometryCallback(const nav_msgs::msg::Odometry& msg);

  void plannerstateCallback(const planner_msgs::msg::NavigationStatus& msg);

  // Next come a couple of public Qt slots.
 public Q_SLOTS:
  void updatePlannerName();
  void updatePlanningBudget();
  void setPlannerName();
  void startEditing(const std::string& id);
  void finishEditing(const std::string& id);
  void widgetPoseUpdated(const std::string& id, mav_msgs::EigenTrajectoryPoint& pose);
  void callPlannerService();
  void callPublishPath();
  void setGoalService();
  void setPlanningBudgetService();
  void setStartService();
  void setStartLoiterService();
  void setCurrentSegmentService();
  void setPathService();
  void publishWaypoint();
  void publishToController();
  void terrainAlignmentStateChanged(int state);
  void EnableMaxAltitude();
  void DisableMaxAltitude();
  void setPlannerModeServiceNavigate();
  void setPlannerModeServiceRollout();
  void setPlannerModeServiceAbort();
  void setPlannerModeServiceReturn();

 protected:
  // Set up the layout, only called by the constructor.
  void createLayout();
  void setNamespace(const QString& new_namespace);
  void setOdometryTopic(const QString& new_odometry_topic);
  void setPlanningBudget(const QString& new_planning_budget);
  void setMaxAltitudeConstrant(bool set_constraint);
  void callSetPlannerStateService(std::string service_name, const int mode);
  QGroupBox* createPlannerModeGroup();
  QGroupBox* createPlannerCommandGroup();
  QGroupBox* createTerrainLoaderGroup();

  // ROS Stuff:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controller_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<planner_msgs::msg::NavigationStatus>::SharedPtr planner_state_sub_;

  std::shared_ptr<GoalMarker> goal_marker_;

  //! @todo(srmainwaring) prevent race condition with async service calls
  std::mutex node_mutex_;  // protects node_

  // QT stuff:
  QLineEdit* namespace_editor_;
  QLineEdit* planner_name_editor_;
  QLineEdit* odometry_topic_editor_;
  QLineEdit* planning_budget_editor_;
  QCheckBox* terrain_align_checkbox_;
  PoseWidget* start_pose_widget_;
  PoseWidget* goal_pose_widget_;
  QPushButton* planner_service_button_;
  QPushButton* set_goal_button_;
  QPushButton* set_start_button_;
  QPushButton* set_current_loiter_button_;
  QPushButton* set_current_segment_button_;
  QPushButton* trigger_planning_button_;
  QPushButton* update_path_button_;
  QPushButton* waypoint_button_;
  QPushButton* max_altitude_button_enable_;
  std::vector<QPushButton*> set_planner_state_buttons_;
  QPushButton* max_altitude_button_disable_;
  QPushButton* controller_button_;
  QPushButton* load_terrain_button_;

  // Keep track of all the pose <-> button widgets as they're related:
  std::map<std::string, PoseWidget*> pose_widget_map_;
  std::map<std::string, EditButton*> edit_button_map_;
  // ROS state:
  PlanningInteractiveMarkers interactive_markers_;

  // QT state:
  QString namespace_;
  QString planner_name_;
  QString planning_budget_value_{"100.0"};
  QString odometry_topic_;
  bool align_terrain_on_load_{true};

  // Other state:
  std::string currently_editing_;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
