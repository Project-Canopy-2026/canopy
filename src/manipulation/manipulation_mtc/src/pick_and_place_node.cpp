/**
 * pick_and_place_node.cpp
 *
 * MoveIt Task Constructor pick-and-place for the xarm7 seedling-planting robot.
 *
 * Replaces the Python planner_node.py with a structured C++ MTC implementation.
 * The overall sequence mirrors the original Python steps but uses MTC for
 * collision-aware, IK-integrated planning at every arm motion stage.
 *
 * ── Gripper note ──────────────────────────────────────────────────────────────
 * The gripper is NOT a MoveIt-controlled group; it is driven via Trigger service
 * calls (/gripper/open, /gripper/close).  Gripper actuations therefore happen
 * between MTC task executions rather than inside a task stage.
 *
 * ── Task structure ───────────────────────────────────────────────────────────
 *   Task 1 "pre_grasp"    : CurrentState → MoveTo(PRE_GRASP_JOINTS, Pilz PTP)
 *   [open gripper service call]
 *   Task 2 "approach"     : CurrentState → SerialContainer{
 *                               MoveRelative(approach along EEF z),
 *                               GeneratePose(grasp_pose) → ComputeIK }
 *   [close gripper service call + attach pot as collision object to EEF]
 *   Task 3 "lift_place"   : CurrentState → MoveRelative(lift +Z)
 *                                        → MoveTo(DROP_JOINTS, free planner)
 *   [open gripper service call + detach/remove pot from scene]
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

// ── Robot / scene configuration ───────────────────────────────────────────────

static constexpr const char * ARM_GROUP  = "xarm7";
static constexpr const char * BASE_FRAME = "link_base";
static constexpr const char * OBJECT_ID  = "pot";

static const std::vector<std::string> JOINT_NAMES = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
};

// Pre-grasp joint positions [rad] ← arm_config.PRE_GRASP_JOINTS_DEG
// = [-61.1, -3.7, -30.5, 2.0, -181.5, 84.7, -90.0] deg
// static const std::map<std::string, double> PRE_GRASP_JOINTS = {
//   {"joint1", -1.0664}, {"joint2", -0.0646}, {"joint3", -0.5323},
//   {"joint4",  0.0349}, {"joint5", -3.1679}, {"joint6",  1.4784}, {"joint7", -1.5708}
// };
// Drop joint positions [rad] ← arm_config.DROP_JOINTS_DEG
// = [-35, -25, 0, 75, -160, -10, -180] deg
// static const std::map<std::string, double> DROP_JOINTS = {
//   {"joint1", -0.6109}, {"joint2", -0.4363}, {"joint3",  0.0000},
//   {"joint4",  1.3090}, {"joint5", -2.7925}, {"joint6", -0.1745}, {"joint7", -3.1416}
// };
static const std::map<std::string, double> PRE_GRASP_JOINTS = {
  {"joint1", -1.3199}, {"joint2",  0.6109}, {"joint3", -0.0279},
  {"joint4",  0.6807}, {"joint5", -2.8760}, {"joint6",  1.6423}, {"joint7", -1.5556}
};
static const std::map<std::string, double> LIFT_JOINTS = {
  {"joint1", -1.2950}, {"joint2",  0.2583}, {"joint3",  0.0367},
  {"joint4",  0.3822}, {"joint5", -2.8207}, {"joint6",  1.5304}, {"joint7", -1.5556}
};
static const std::map<std::string, double> ABOVE_CHUTE_JOINTS = {
  {"joint1", -1.4785}, {"joint2", -0.1641}, {"joint3",  0.0401},
  {"joint4",  1.3826}, {"joint5", -3.1504}, {"joint6",  0.1065}, {"joint7", -1.5708}
};
static const std::map<std::string, double> DROP_JOINTS = {
  {"joint1",  1.2185}, {"joint2", -0.3142}, {"joint3",  0.2618},
  {"joint4",  1.4015}, {"joint5", -2.9845}, {"joint6", -0.2060}, {"joint7", -1.5708}
};


// Grasp orientation [rad] ← arm_config.GRASP_RPY = (89.7°, -90°, 0°)
static constexpr double GRASP_ROLL  =  1.5655; //1.5655;  //  89.7 deg
static constexpr double GRASP_PITCH =  0.0; //-1.5708;  // -90.0 deg
static constexpr double GRASP_YAW   =  0;

// Cartesian motion
static constexpr double LIFT_HEIGHT     = 0.09;   // m — matches (LIFT_Z - GRASP_Z)
static constexpr double APPROACH_MIN    = 0.02;   // m — min approach distance
static constexpr double APPROACH_MAX    = 0.15;   // m — max approach distance
static constexpr double CARTESIAN_STEP  = 0.005;   // m — interpolation step size
static constexpr double MAX_VEL         = 0.3;
static constexpr double MAX_ACC         = 0.3;

// Pot collision geometry
static constexpr double POT_HEIGHT = 0.10;  // m (cylinder height)
static constexpr double POT_RADIUS = 0.03;  // m (cylinder radius)

// ── Node ─────────────────────────────────────────────────────────────────────

class PickAndPlaceNode : public rclcpp::Node
{
public:
  explicit PickAndPlaceNode(const rclcpp::NodeOptions & options)
  : Node("pick_and_place", options)
  {
    declare_parameter("sim_mode",   false);
    declare_parameter("ee_link",    std::string("tool_tcp"));
    declare_parameter("sim_pot_x",  0.0);
    declare_parameter("sim_pot_y", -0.350);
    declare_parameter("sim_pot_z",  0.160);

    sim_mode_ = get_parameter("sim_mode").as_bool();
    ee_link_  = get_parameter("ee_link").as_string();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Gripper service clients
    gripper_open_  = create_client<std_srvs::srv::Trigger>("/gripper/open");
    gripper_close_ = create_client<std_srvs::srv::Trigger>("/gripper/close");

    // FSM publishers
    seedling_dropped_pub_ =
      create_publisher<std_msgs::msg::Bool>("behavior/seedling_dropped", 10);
    detect_pub_ =
      create_publisher<std_msgs::msg::Bool>("behavior/enable_pot_detection", 10);

    // FSM subscribers
    create_subscription<std_msgs::msg::Bool>(
      "behavior/do_planting", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        should_run_ = msg->data;
        RCLCPP_INFO(get_logger(),
          msg->data ? "Planting command received" : "Stop command received");
      });

    create_subscription<std_msgs::msg::Bool>(
      "/chute_in_position", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        chute_in_position_ = msg->data;
      });

    create_subscription<geometry_msgs::msg::PointStamped>(
      "pot/center_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        pot_center_ = msg;
      });

    // Simulation mode: pre-populate pot center and auto-trigger
    if (sim_mode_) {
      RCLCPP_INFO(get_logger(), "*** SIMULATION MODE ENABLED ***");
      auto fake = std::make_shared<geometry_msgs::msg::PointStamped>();
      fake->header.frame_id = BASE_FRAME;
      fake->point.x = get_parameter("sim_pot_x").as_double();
      fake->point.y = get_parameter("sim_pot_y").as_double();
      fake->point.z = get_parameter("sim_pot_z").as_double();
      pot_center_        = fake;
      chute_in_position_ = true;
      RCLCPP_INFO(get_logger(), "[SIM] Fake pot: (%.3f, %.3f, %.3f)",
        fake->point.x, fake->point.y, fake->point.z);

      sim_timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {
        sim_timer_->cancel();
        RCLCPP_INFO(get_logger(), "[SIM] Auto-triggering pick and place");
        should_run_ = true;
      });
    }

    RCLCPP_INFO(get_logger(), "PickAndPlaceNode ready — ee_link=%s", ee_link_.c_str());
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return get_node_base_interface();
  }

  /** Blocking loop — run from a dedicated thread. */
  void run()
  {
    while (rclcpp::ok()) {
      if (should_run_) {
        should_run_ = false;
        runPickAndPlace();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

private:
  // ── Gripper helpers ──────────────────────────────────────────────────────

  bool callGripper(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    const char * action)
  {
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Gripper %s service not available", action);
      return false;
    }
    auto future =
      client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Gripper %s timed out", action);
      return false;
    }
    auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(get_logger(), "Gripper %s failed: %s", action, result->message.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Gripper %s OK", action);
    return true;
  }

  bool openGripper()  { return callGripper(gripper_open_,  "open"); }
  bool closeGripper() { return callGripper(gripper_close_, "close"); }

  // ── Planning scene helpers ────────────────────────────────────────────────

  /** Add static collision objects (floor, planting assembly). */
  void setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // Clear stale objects from a previous run
    psi.removeCollisionObjects({"floor", "planting_assembly", OBJECT_ID});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    auto makeBox = [](const std::string & id,
                      double cx, double cy, double cz,
                      double sx, double sy, double sz)
    {
      moveit_msgs::msg::CollisionObject o;
      o.id               = id;
      o.header.frame_id  = BASE_FRAME;
      o.operation        = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive prim;
      prim.type          = shape_msgs::msg::SolidPrimitive::BOX;
      prim.dimensions    = {sx, sy, sz};
      o.primitives.push_back(prim);
      geometry_msgs::msg::Pose p;
      p.position.x  = cx; p.position.y = cy; p.position.z = cz;
      p.orientation.w = 1.0;
      o.primitive_poses.push_back(p);
      return o;
    };

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    // Floor slab (z centred on -0.025 so top face is at 0)
    // objects.push_back(makeBox("floor",             0.0,  0.0, -0.025, 3.0, 3.0, 0.05));
    // Planting assembly column on the right side
    // objects.push_back(makeBox("planting_assembly", 0.45, 0.0,  0.20,  0.2, 1.0, 0.5));

    psi.applyCollisionObjects(objects);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    RCLCPP_INFO(get_logger(), "Planning scene ready");
  }

  /** Add the pot as a free cylinder collision object at the detected position. */
  void addPotToScene(const geometry_msgs::msg::PointStamped & center)
  {
    moveit_msgs::msg::CollisionObject o;
    o.id              = OBJECT_ID;
    o.header.frame_id = center.header.frame_id.empty() ? BASE_FRAME : center.header.frame_id;
    o.operation       = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type       = shape_msgs::msg::SolidPrimitive::CYLINDER;
    prim.dimensions = {POT_HEIGHT, POT_RADIUS};
    o.primitives.push_back(prim);

    geometry_msgs::msg::Pose p;
    p.position    = center.point;
    p.orientation.w = 1.0;
    o.primitive_poses.push_back(p);

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(o);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    RCLCPP_INFO(get_logger(), "Pot added to scene at (%.3f, %.3f, %.3f)",
      center.point.x, center.point.y, center.point.z);
  }

  /**
   * Attach the pot to the EEF link so that the lift/transport planner
   * treats it as part of the robot (correct collision checking).
   * MoveIt moves the existing world object to the attached-object list.
   */
  void attachPotToEEF()
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name        = ee_link_;
    aco.object.id        = OBJECT_ID;
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Allow contact between the pot and the EEF / gripper links
    aco.touch_links = {ee_link_, "link_eef", "link7", "link6"};

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyAttachedCollisionObject(aco);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(get_logger(), "Pot attached to %s", ee_link_.c_str());
  }

  /** Detach the pot and remove it from the scene (seedling was released). */
  void detachAndRemovePot()
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name        = ee_link_;
    aco.object.id        = OBJECT_ID;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyAttachedCollisionObject(aco);
    psi.removeCollisionObjects({OBJECT_ID});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(get_logger(), "Pot detached and removed from scene");
  }

  // ── Grasp pose from perception ─────────────────────────────────────────────

  /**
   * Build a PoseStamped for the grasp from the pot's detected centre point.
   *
   * The orientation is fixed to GRASP_RPY from arm_config (roll 89.7°, pitch -90°)
   * which places the gripper z-axis pointing horizontally toward the pot.
   * If the perception pipeline computes a custom orientation (get_grasp_pose()),
   * replace the setRPY call with the quaternion from that computation.
   */
  geometry_msgs::msg::PoseStamped buildGraspPose(
    const geometry_msgs::msg::PointStamped & pot) const
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = pot.header.frame_id.empty() ? BASE_FRAME : pot.header.frame_id;
    ps.header.stamp    = now();
    ps.pose.position   = pot.point;

    // Look up current EEF pose in base frame
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        BASE_FRAME, ee_link_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(),
        "TF lookup failed, falling back to fixed RPY: %s", e.what());
      tf2::Quaternion q;
      q.setRPY(GRASP_ROLL, GRASP_PITCH, GRASP_YAW);
      q.normalize();
      tf2::convert(q, ps.pose.orientation);
      return ps;
    }

    // Current EEF position and orientation
    Eigen::Vector3d gripper_pos(
      tf_stamped.transform.translation.x,
      tf_stamped.transform.translation.y,
      tf_stamped.transform.translation.z);

    Eigen::Quaterniond q_current(
      tf_stamped.transform.rotation.w,
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z);
    Eigen::Matrix3d current_R = q_current.toRotationMatrix();
    Eigen::Vector3d y_ref = current_R.col(1);  // current EEF Y axis in base frame

    // Pot position
    Eigen::Vector3d pot_pos(pot.point.x, pot.point.y, pot.point.z);

    // Approach axis: vector from gripper toward pot, normalized → EEF z
    Eigen::Vector3d z_axis = (pot_pos - gripper_pos).normalized();

    // Sideways axis: cross(y_ref, z_axis)
    Eigen::Vector3d x_axis = y_ref.cross(z_axis);
    if (x_axis.norm() < 1e-8) {
      // y_ref is parallel to z_axis — fall back to current EEF X axis
      x_axis = current_R.col(0).cross(z_axis);
    }
    x_axis.normalize();

    // Remaining axis, then re-orthogonalize
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    Eigen::Quaterniond q_grasp(R);
    q_grasp.normalize();

    ps.pose.orientation.x = q_grasp.x();
    ps.pose.orientation.y = q_grasp.y();
    ps.pose.orientation.z = q_grasp.z();
    ps.pose.orientation.w = q_grasp.w();

    return ps;
  }

  // ── MTC task runner ───────────────────────────────────────────────────────

  bool executeTask(mtc::Task & task, const char * label)
  {
    try {
      task.init();
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, label << ": init failed — " << e);
      return false;
    }

    if (!task.plan(5)) {
      RCLCPP_ERROR(LOGGER, "%s: planning failed", label);
      return false;
    }
    RCLCPP_INFO(LOGGER, "%s: %zu solution(s) found", label, task.solutions().size());

    // Publish to MTC introspection (visible in RViz MTC panel)
    task.introspection().publishSolution(*task.solutions().front());

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "%s: execution failed (error code %d)", label, result.val);
      return false;
    }
    return true;
  }

  // ── MTC task definitions ──────────────────────────────────────────────────

  /**
   * Task 1 — Pre-grasp
   * Plans a smooth Pilz PTP joint-space move to the pre-grasp configuration.
   * This positions the arm close to the grasp pose in joint space so the
   * subsequent approach IK finds a nearby, consistent solution.
   */
  mtc::Task createPreGraspTask()
  {
    mtc::Task task;
    task.stages()->setName("pre_grasp");
    task.loadRobotModel(shared_from_this());
    task.setProperty("group",    std::string(ARM_GROUP));
    task.setProperty("ik_frame", ee_link_);

    // JointInterpolationPlanner: MTC's built-in LERP solver.
    // Always produces properly time-stamped joint trajectories and requires
    // no external planning pipeline — avoids the t=0/t=0 rejection that
    // occurs when OMPL falls back without running time parameterisation.
    auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    joint_planner->setMaxVelocityScalingFactor(MAX_VEL);
    joint_planner->setMaxAccelerationScalingFactor(MAX_ACC);

    task.add(std::make_unique<mtc::stages::CurrentState>("current_state"));

    auto move = std::make_unique<mtc::stages::MoveTo>("move_to_pre_grasp", joint_planner);
    move->setGroup(ARM_GROUP);
    move->setGoal(PRE_GRASP_JOINTS);
    task.add(std::move(move));

    return task;
  }

  /**
   * Task 2 — Approach and reach grasp pose
   *
   * Uses a SerialContainer so MTC can plan bidirectionally:
   *   - Forward from pre-grasp through the free space
   *   - Backward from the grasp pose to find the exact approach direction
   *
   * Stages inside the SerialContainer:
   *   MoveRelative "approach"   — slide forward along the EEF z-axis
   *                               (final N cm before contact)
   *   GeneratePose + ComputeIK  — IK to the perception-derived grasp pose
   *
   * The outer Connect stage bridges the current robot state to the start of
   * the approach motion (free-space planning, any orientation).
   */
  mtc::Task createApproachTask(const geometry_msgs::msg::PoseStamped & grasp_pose)
  {
    mtc::Task task;
    task.stages()->setName("approach_grasp");
    task.loadRobotModel(shared_from_this());
    task.setProperty("group",    std::string(ARM_GROUP));
    task.setProperty("ik_frame", ee_link_);

    // JointInterpolationPlanner for the Connect stage: identical to the pre-grasp
    // task approach.  PipelinePlanner (OMPL) falls back without time parameterisation
    // and the controller rejects the trajectory with "Time between points not strictly
    // increasing" → MoveItErrorCodes::FAILURE (99999).  JointInterpolationPlanner is
    // built into MTC, requires no external pipeline, and always produces time-stamped
    // joint trajectories.
    auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    joint_planner->setMaxVelocityScalingFactor(MAX_VEL);
    joint_planner->setMaxAccelerationScalingFactor(MAX_ACC);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(MAX_VEL);
    cartesian_planner->setMaxAccelerationScalingFactor(MAX_ACC);
    cartesian_planner->setStepSize(CARTESIAN_STEP);

    // Track current state so GeneratePose can use it as the monitoring seed
    mtc::Stage * current_state_ptr = nullptr;
    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current_state");
      current_state_ptr = s.get();
      task.add(std::move(s));
    }

    // Free-space connect from current state to the start of the approach
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "free_space_to_approach",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, joint_planner}});
      connect->setTimeout(10.0);
      connect->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      task.add(std::move(connect));
    }

    // SerialContainer groups the constrained approach + IK stages
    auto approach_container = std::make_unique<mtc::SerialContainer>("grasp_sequence");
    task.properties().exposeTo(approach_container->properties(), {"group", "ik_frame"});
    approach_container->properties().configureInitFrom(
      mtc::Stage::PARENT, {"group", "ik_frame"});

    // 2a. Constrained Cartesian approach: slide along the EEF z-axis toward the object
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("approach_object", cartesian_planner);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setIKFrame(ee_link_);  // MoveRelative stores ik_frame as PoseStamped internally;
                                // must be set via setIKFrame(), not inherited from parent
      s->properties().set("marker_ns", "approach_object");
      s->setMinMaxDistance(APPROACH_MIN, APPROACH_MAX);
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = ee_link_;
      dir.vector.z        = 1.0;   // positive z = forward along gripper axis
      s->setDirection(dir);
      approach_container->add(std::move(s));
    }

    // 2b. Generate the specific perception-derived grasp pose and solve IK
    {
      auto gen = std::make_unique<mtc::stages::GeneratePose>("generate_grasp_pose");
      gen->setPose(grasp_pose);
      gen->setMonitoredStage(current_state_ptr);  // seed IK from current state

      auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp_ik", std::move(gen));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(1.0);
      ik->setIKFrame(ee_link_);
      ik->properties().configureInitFrom(mtc::Stage::PARENT,    {"group", "ik_frame"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      approach_container->add(std::move(ik));
    }

    task.add(std::move(approach_container));
    return task;
  }

  /**
   * Task 3 — Lift, transport, and lower to drop pose
   *
   * Called after the pot is physically grasped (gripper closed) and attached
   * to the EEF in the MoveIt planning scene.  The attached object changes the
   * robot's collision geometry so the planner avoids clipping the pot.
   *
   *   MoveRelative "lift"        — straight up in world z (Cartesian)
   *   MoveTo       "drop"        — free-space swing to DROP_JOINTS (OMPL/Pilz)
   */
  mtc::Task createLiftAndPlaceTask()
  {
    mtc::Task task;
    task.stages()->setName("lift_and_place");
    task.loadRobotModel(shared_from_this());
    task.setProperty("group",    std::string(ARM_GROUP));
    task.setProperty("ik_frame", ee_link_);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

    // Use half speed for the lift to protect the seedling
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(MAX_VEL * 0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(MAX_ACC * 0.5);
    cartesian_planner->setStepSize(CARTESIAN_STEP);

    task.add(std::make_unique<mtc::stages::CurrentState>("current_state"));

    // Straight vertical lift (world frame z)
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("lift_object", cartesian_planner);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setIKFrame(ee_link_);
      s->properties().set("marker_ns", "lift_object");
      s->setMinMaxDistance(LIFT_HEIGHT * 0.8, LIFT_HEIGHT * 1.5);
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = BASE_FRAME;
      dir.vector.z        = 1.0;
      s->setDirection(dir);
      task.add(std::move(s));
    }

    // Free-space transport + lower to the drop joint configuration.
    // Use JointInterpolationPlanner for the same reason as pre-grasp:
    // guarantees proper time stamps without depending on pipeline availability.
    {
      auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
      joint_planner->setMaxVelocityScalingFactor(MAX_VEL);
      joint_planner->setMaxAccelerationScalingFactor(MAX_ACC);
      auto s = std::make_unique<mtc::stages::MoveTo>("transport_to_drop", joint_planner);
      s->setGroup(ARM_GROUP);
      s->setGoal(DROP_JOINTS);
      task.add(std::move(s));
    }

    return task;
  }

  // ── Main orchestration ────────────────────────────────────────────────────

  void runPickAndPlace()
  {
    RCLCPP_INFO(get_logger(), "=== Starting pick and place ===");

    if (!pot_center_) {
      RCLCPP_ERROR(get_logger(), "No pot position available — aborting");
      return;
    }

    // ── Step 0: Setup planning scene ──────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Step 0: Setting up planning scene...");
    setupPlanningScene();

    // ── Step 1: Move to pre-grasp (Pilz PTP) ─────────────────────────────
    RCLCPP_INFO(get_logger(), "Step 1: Moving to pre-grasp...");
    {
      auto task = createPreGraspTask();
      if (!executeTask(task, "pre_grasp")) return;
    }

    // ── Step 2: Open gripper ──────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Step 2: Opening gripper...");
    if (!openGripper()) return;

    // ── Step 3: Approach and reach grasp pose (MTC collision-aware IK) ───
    const auto grasp_pose = buildGraspPose(*pot_center_);
    RCLCPP_INFO(get_logger(),
      "Step 3: Approaching grasp pose at (%.3f, %.3f, %.3f) [%s]...",
      grasp_pose.pose.position.x, grasp_pose.pose.position.y,
      grasp_pose.pose.position.z, grasp_pose.header.frame_id.c_str());
    {
      auto task = createApproachTask(grasp_pose);
      if (!executeTask(task, "approach_grasp")) return;
    }

    // ── Step 4: Close gripper ─────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Step 4: Closing gripper...");
    if (!closeGripper()) return;

    // // ── Step 5: Wait for chute in position ────────────────────────────────
    // if (!sim_mode_) {
    //   RCLCPP_INFO(get_logger(), "Step 5: Waiting for chute in position...");
    //   while (rclcpp::ok() && !chute_in_position_) {
    //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    //       "Waiting for /chute_in_position...");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //   }
    // } else {
    //   RCLCPP_INFO(get_logger(), "Step 5: [SIM] Skipping chute wait.");
    // }

    // // ── Step 6: Lift, transport, lower to drop (pot attached) ─────────────
    // RCLCPP_INFO(get_logger(), "Step 6: Lifting and transporting to drop zone...");
    // {
    //   auto task = createLiftAndPlaceTask();
    //   if (!executeTask(task, "lift_and_place")) return;
    // }

    // // ── Step 7: Release seedling ──────────────────────────────────────────
    // RCLCPP_INFO(get_logger(), "Step 7: Opening gripper to release seedling...");
    // if (!openGripper()) {
    //   RCLCPP_ERROR(get_logger(),
    //     "Gripper open failed at drop — seedling may not have released");
    // }

    // ── Notify FSM ─────────────────────────────────────────────────────────
    std_msgs::msg::Bool dropped;
    dropped.data = true;
    seedling_dropped_pub_->publish(dropped);

    RCLCPP_INFO(get_logger(), "=== Pick and place complete ===");
  }

  // ── Members ───────────────────────────────────────────────────────────────

  bool should_run_{false};
  bool sim_mode_{false};
  bool chute_in_position_{false};
  std::string ee_link_{"tool_tcp"};

  geometry_msgs::msg::PointStamped::ConstSharedPtr pot_center_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_open_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_close_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr seedling_dropped_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detect_pub_;

  rclcpp::TimerBase::SharedPtr sim_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<PickAndPlaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  // Spin the ROS2 executor in a dedicated thread (callbacks, TF, MoveGroup comms)
  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface());
  });

  // Run the pick-and-place loop in a second thread (blocks on planning/execution)
  auto run_thread = std::make_unique<std::thread>([&node]() {
    node->run();
  });

  spin_thread->join();
  run_thread->join();
  rclcpp::shutdown();
  return 0;
}
