#include <iostream>
#include <stdio.h>
#include <sstream>
#include <time.h>

#include <pan_tilt_driver/pan_tilt_driver.h>

#include <ros/ros.h>
#include <pan_tilt_driver/ptulimits.h>
#include <pan_tilt_driver/ptureset.h>
#include <pan_tilt_driver/ptugoto.h>
#include <pan_tilt_driver/ptudesiredspeed.h>

PanTiltDriverPtr pt_ptr;


bool ptureset(pan_tilt_driver::ptureset::Request &req, pan_tilt_driver::ptureset::Response &res) {
  res.result = pt_ptr->reset();
  return res.result;
}

bool ptulimits(pan_tilt_driver::ptulimits::Request &req, pan_tilt_driver::ptulimits::Response &res) {
  // res.result = pt_ptr->get_limits(res.pan_max,res.pan_min,res.tilt_max, res.tilt_min,
  // 				  res.pan_upper_speed, res.tilt_upper_speed);
  // return res.result;
  return pt_ptr->goto_limits();
}

bool ptugoto(pan_tilt_driver::ptugoto::Request &req, pan_tilt_driver::ptugoto::Response &res) {

  std::cout << "Executing goto command: pan:" << req.pan << " tilt:" << req.tilt << std::endl; 
  res.result = pt_ptr->ptu_goto((float)req.pan,(float)req.tilt);
  return res.result;
}

bool ptudesiredspeed(pan_tilt_driver::ptudesiredspeed::Request &req, pan_tilt_driver::ptudesiredspeed::Response &res) {
  

  res.result = pt_ptr->set_speed(req.pan_speed, req.tilt_speed);
  return res.result;
}

int main(int argc, char *argv[]){

  ros::init(argc, argv, "pan_tilt_driver");
  ros::NodeHandle n;

  //read the simulation parameter to show if its a simulation or real operation
  //the default is real operation
  bool simulation(false);
  
  if(!n.getParam("pan_tilt_driver_node/simulation",simulation)) {
    ROS_INFO("Simulation parameter is not set. Will assume its not simulation");
  }

  pt_ptr.reset(new PanTiltDriver(argc,argv,simulation));

  //advertise the services
  std::cout << "Advertising the ptu services" << std::endl;
  ros::ServiceServer ptureset_service = n.advertiseService("ptu/reset",ptureset);
  ros::ServiceServer ptulimits_service = n.advertiseService("ptu/limits",ptulimits);
  ros::ServiceServer ptugoto_service = n.advertiseService("ptu/goto",ptugoto);
  ros::ServiceServer ptudesiredspeed_service = n.advertiseService("ptu/desiredspeed",ptudesiredspeed);

  ROS_INFO_STREAM("PTU services started. Waiting for commands...");
  ros::spin();
  
  return 0;
}






