#include <unistd.h>

#include "ros/ros.h"
#include "wam_node/GravityComp.h"
#include "wam_node/JointMove.h"
#include "wam_node/CartMove.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>

static const int PUBLISH_FREQ = 1;

using namespace barrett;

bool grav_state = true;
int wam_dof = 0;
systems::Wam<4>* wam4 = NULL;
systems::Wam<7>* wam7 = NULL;
units::JointPositions<4>::type* jp4;
units::JointPositions<4>::type* jp4home;
units::JointPositions<7>::type* jp7;
units::JointPositions<7>::type* jp7home;
units::CartesianPosition::type* cpp;
Eigen::Quaterniond to;
Eigen::Quaterniond tog;

class WamNode
{
protected:
  //ros::NodeHandle n_;
  sensor_msgs::JointState joint_state;
  geometry_msgs::PoseStamped wam_pose;
  ros::Publisher joint_pub;
  ros::Publisher pose_pub;
  ros::ServiceServer gravity_srv;
  ros::ServiceServer jmove_srv;
  ros::ServiceServer cmove_srv;
  ros::ServiceServer home_srv;

public:
  WamNode(){}

  void init(std::vector<double> &dof)
  {
    WamNode wam;
    ros::NodeHandle n_("wam");
    wam_dof = dof.size();
    gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, &wam);
    jmove_srv = n_.advertiseService("joint_move", &WamNode::joint_move, &wam);
    cmove_srv = n_.advertiseService("cartesian_move", &WamNode::cart_move, &wam);
    home_srv = n_.advertiseService("go_home",&WamNode::go_home, &wam);
    joint_pub = n_.advertise<sensor_msgs::JointState>("wam_joints", 100);
    pose_pub = n_.advertise<geometry_msgs::PoseStamped>("wam_pose", 100);

    //explicitly naming joints until loaded from urdf
    const char* strarray[] = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    std::vector<std::string> joints(strarray, strarray + 7);
    joint_state.name = joints;
    joint_state.name.resize(wam_dof);
    joint_state.position.resize(wam_dof);
    joint_state.velocity.resize(wam_dof);
    joint_state.effort.resize(wam_dof);

    if (wam_dof == 4)
    {
      *jp4home = wam4->getJointPositions();
    }
    else if (wam_dof == 7)
    {
      *jp7home = wam7->getJointPositions();
    }
  }

  ~WamNode(){}

  bool gravity(wam_node::GravityComp::Request &req, wam_node::GravityComp::Response &res);
  bool joint_move(wam_node::JointMove::Request &req, wam_node::JointMove::Response &res);
  bool cart_move(wam_node::CartMove::Request &req, wam_node::CartMove::Response &res);
  bool go_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void update_publisher(std::vector<double> &j_pos, std::vector<double> &j_vel, std::vector<double> &j_tor,
                   std::vector<double> &c_pos);
};

bool WamNode::gravity(wam_node::GravityComp::Request &req, wam_node::GravityComp::Response &res)
{
  grav_state = req.gravity;
  if (wam4 != NULL)
    wam4->gravityCompensate(grav_state);
  else if (wam7 != NULL)
    wam7->gravityCompensate(grav_state);

  ROS_INFO("Gravity Compensation Request: %s", (req.gravity) ? "true" : "false");
  return true;
}

bool WamNode::joint_move(wam_node::JointMove::Request &req, wam_node::JointMove::Response &res)
{
  if (req.joints.size() != (size_t)wam_dof)
  {
    ROS_INFO("Request Failed: %d-DOF request received, must be %d-DOF", req.joints.size(), wam_dof);
    return false;
  }

  if (wam_dof == 4)
  {
    for (int i = 0; i < 4; i++)
      (*jp4)[i] = req.joints[i];
    wam4->moveTo(*jp4);
  }
  else
  {
    for (int j = 0; j < 7; j++)
      (*jp7)[j] = req.joints[j];
    wam7->moveTo(*jp7);
  }
  ROS_INFO("Moving Robot to Commanded Joint Pose");
  return true;
}

bool WamNode::cart_move(wam_node::CartMove::Request &req, wam_node::CartMove::Response &res)
{
  if (req.coordinates.size() != 3)
  {
    ROS_INFO("Request Failed: %d coordinate request received, must be 3 coordinates [X,Y,Z]", req.coordinates.size());
    return false;
  }
  for (int i = 0; i < 3; i++)
    (*cpp)[i] = req.coordinates[i];

  if (wam_dof == 4)
    wam4->moveTo(*jp4);
  else
    wam7->moveTo(*jp7);

  ROS_INFO("Moving Robot to Commanded Coordinate Pose");
  return true;
}

bool WamNode::go_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (wam_dof == 4)
  {
    wam4->moveTo(*jp4home);
  }
  else
  {
    wam7->moveTo(*jp7home);
  }
  ROS_INFO("Moving Robot to Starting Position");
  return true;
}

void WamNode::update_publisher(std::vector<double> &j_pos, std::vector<double> &j_vel, std::vector<double> &j_tor,
                               std::vector<double> &c_pos)
{
  //publishing sensor_msgs/JointState to wam_joints
  for (size_t i = 0; i < j_pos.size(); i++)
  {
    joint_state.position[i] = j_pos[i];
    joint_state.velocity[i] = j_vel[i];
    joint_state.effort[i] = j_tor[i];
  }
  joint_pub.publish(joint_state);

  //publishing geometry_msgs/PoseStamed to wam_pose
  wam_pose.pose.position.x = c_pos[0];
  wam_pose.pose.position.y = c_pos[1];
  wam_pose.pose.position.z = c_pos[2];
  wam_pose.pose.orientation.w = to.w();
  wam_pose.pose.orientation.x = to.x();
  wam_pose.pose.orientation.y = to.y();
  wam_pose.pose.orientation.z = to.z();
  pose_pub.publish(wam_pose);
}

template<size_t DOF>
  int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
    wam.gravityCompensate(grav_state);
    ros::init(argc, argv, "barrettWam");
    WamNode wam_node;

    if (pm.foundWam4())
    {
      wam4 = pm.getWam4();
    }
    else if (pm.foundWam7())
      wam7 = pm.getWam7();

    jp_type jp;
    jv_type jv;
    jt_type jt;
    cp_type cp;

    cpp = new units::CartesianPosition::type;
    if (DOF == 4)
    {
      jp4 = new units::JointPositions<4>::type;
      jp4home = new units::JointPositions<4>::type;

    }
    else if (DOF == 7)
    {
      jp7 = new units::JointPositions<7>::type;
      jp7home = new units::JointPositions<7>::type;
    }

    std::vector<double> joint_vel;
    std::vector<double> joint_tor;
    std::vector<double> joint_pos;
    std::vector<double> cart_pos;
    joint_pos.resize(jp.size());
    joint_vel.resize(jv.size());
    joint_tor.resize(jt.size());
    cart_pos.resize(cp.size());

    wam_node.init(joint_pos);

    int pub_rate = 1000000 * (1 / PUBLISH_FREQ);
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
    {
      ros::spinOnce();
      jp = wam.getJointPositions();
      jt = wam.getJointTorques();
      jv = wam.getJointVelocities();
      cp = wam.getToolPosition();
      to = wam.getToolOrientation();
      for (size_t i = 0; i < DOF; i++)
      {
        joint_pos[i] = jp[i];
        joint_vel[i] = jv[i];
        joint_tor[i] = jt[i];
        if (i < 3)
          cart_pos[i] = cp[i];
      }
      wam_node.update_publisher(joint_pos, joint_vel, joint_tor, cart_pos);
      usleep(pub_rate); // publishing at frequency PUBLISH_FREQ
    }
    delete cpp;
    delete jp4;
    delete jp4home;
    delete jp7;
    delete jp7home;
    return 0;
  }

