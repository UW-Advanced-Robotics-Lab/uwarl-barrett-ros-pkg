#include <iostream>
#include <string>
#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wam_node/GravityComp.h"

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>


using namespace barrett;

bool grav_state = true;
systems::Wam<4>* wam4 = NULL;
systems::Wam<7>* wam7 = NULL;


bool gravity(wam_node::GravityComp::Request &req, wam_node::GravityComp::Response &res) {
  grav_state = req.gravity;

  if(wam4 != NULL)
	  wam4->gravityCompensate(grav_state);
  else if(wam7 != NULL)
	  wam7->gravityCompensate(grav_state);

  ROS_INFO("Gravity Compensation Request: %s",(req.gravity)?"true":"false");
  return true;
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  wam.gravityCompensate(grav_state); 
  ros::init(argc, argv, "barrettWam");
  ros::NodeHandle nh_;

  if(pm.foundWam4()){
	 wam4 = pm.getWam4();
  }else if(pm.foundWam7())
     wam7 = pm.getWam7();

  std::cerr << "got Wam" << std::endl;
  ros::Rate loop_rate(100);
  ros::ServiceServer service = nh_.advertiseService("wam_gravity_comp",gravity);
  while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}









