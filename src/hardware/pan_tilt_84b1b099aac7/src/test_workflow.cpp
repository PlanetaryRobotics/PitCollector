#include <iostream>
#include <stdio.h>
#include <sstream>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

extern "C" {
#include <estrap.h>
}

#define NR_ITERATIONS	2
#define PAN_HOME		0
#define TILT_HOME		0

/* blocking go-to-position */
void ptu_go_to(struct cerial *cer, int pan, int tilt){
  uint16_t status;

  if(cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, pan) ||
     cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_SET, tilt))
    {
      die("Failed to go to min position.\n");
    }
  if(cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, pan) ||
     cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, tilt))
    {
      die("Blocking for reset completion failed.\n");
    }
}
/* Reset PTU All */
void ptu_reset(struct cerial *cer, cpi_reset_type type){
  uint16_t status;

  if(cpi_ptcmd(cer, &status, OP_RESET, type) )
    {
      die("Failed to execute Reset.\n");
    }
  if(cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, PAN_HOME) ||
     cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, TILT_HOME)){
    die("Blocking failed.\n");
  }

}

bool listen_to_left_image(false),listen_to_right_image(false);
cv::Mat *left_image = NULL,*right_image = NULL;

void left_callback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_STREAM("Received left image.");
  if(listen_to_left_image) {
    try
      {
	if(left_image != NULL) {
	  delete left_image;
	}
	left_image = new cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image.clone());
	listen_to_left_image = false;
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
  }
}

void right_callback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_STREAM("Received right image.");
  if(listen_to_right_image) {
    try
      {
	if(right_image != NULL) {
	  delete right_image;
	}
	right_image = new cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image.clone());
	listen_to_right_image = false;
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
  }
}

const std::string current_date_time() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);
  
  return buf;
}

int main(int argc, char *argv[]){

  ros::init(argc, argv, "pan_tilt_driver");
  ros::NodeHandle n;

  std::string left_topic("/left/image_color");
  ROS_INFO_STREAM("Subscribing to topic:" << left_topic);
  ros::Subscriber left_sub = n.subscribe(left_topic, 1, left_callback);

  std::string right_topic("/right/image_color");
  ROS_INFO_STREAM("Subscribing to topic:" << right_topic);
  ros::Subscriber right_sub = n.subscribe(right_topic, 1, right_callback);


  struct cerial   *cer;
  uint16_t        status;
  int             pn, px, tn, tx, pu, tu;
  int i = 0,j=0;
  int psteps=10,tsteps=8;

  if((cer = estrap_in(argc, argv)) == NULL){
    return 1;
  }

  ros::spinOnce();

  // Set terse mode
  if(cpi_ptcmd(cer, &status, OP_FEEDBACK_SET, CPI_ASCII_FEEDBACK_TERSE)){
    die("Failed to set feedback mode.\n");
  }

  /* read min/max position and speed */
  if(cpi_ptcmd(cer, &status, OP_PAN_MAX_POSITION, &px) ||
     cpi_ptcmd(cer, &status, OP_PAN_MIN_POSITION, &pn) ||
     cpi_ptcmd(cer, &status, OP_TILT_MAX_POSITION, &tx) ||
     cpi_ptcmd(cer, &status, OP_TILT_MIN_POSITION, &tn) ||
     cpi_ptcmd(cer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET, &pu) ||
     cpi_ptcmd(cer, &status, OP_TILT_UPPER_SPEED_LIMIT_GET, &tu)){
    die("Basic unit queries failed.\n");
  }

  printf("Maxima px %d  pn %d  tx %d  tn %d  pu %d  tu %d\n",px,pn,tx,tn,pu,tu);
  printf("Range p %d  t %d\n",px-pn,tx-tn);
  /* Test reset */
  ptu_reset(cer, CPI_RESET_ALL);

  ros::spinOnce();

  /* set desired speed to half the upper speed */

  //  pu /= 2;
  tu = (int)((float)tu * (((float)tx-(float)tn)/((float)px-(float)pn)));
  if(cpi_ptcmd(cer, &status, OP_PAN_DESIRED_SPEED_SET, pu ) ||
     cpi_ptcmd(cer, &status, OP_TILT_DESIRED_SPEED_SET, tu )) {
    die("Setting PS/TS failed.\n");
  }

  pn /= 2;
  px /= 2;
  //  tn /= 2;
  //  tx /= 2;
  printf("Limits px %d  pn %d  tx %d  tn %d  pu %d  tu %d\n",px,pn,tx,tn,pu,tu);

  std::string dir("/home/dsw/Data/");
  std::stringstream left_filename;
  std::stringstream right_filename;

  int counter=0;
  for ( i=pn; i < px; i += (px-pn)/psteps) {
    for ( j=tn; j < tx; j += (tx-tn)/tsteps)
    {
      ptu_go_to(cer,i,j);
      usleep(2*1e6);

      //printf("Click\n");
      listen_to_left_image = listen_to_right_image = true;

      ROS_INFO_STREAM("Waiting to grab images...");
      while(listen_to_left_image  || listen_to_right_image) {
	ros::spinOnce();
      }

      std::string tstamp(current_date_time());
      
      left_filename.str("");
      left_filename << dir << "left_" << tstamp << "_" <<
	std::setfill('0') << std::setw(3) << counter << "_" << 
	std::setfill('0') << std::setw(5) << i << "_" <<
	std::setfill('0') << std::setw(5) << j << ".ppm";

      right_filename.str("");
      right_filename << dir << "right_" << tstamp << "_" << 
	std::setfill('0') << std::setw(3) << counter << "_" <<
	std::setfill('0') << std::setw(5) <<  i << "_" <<
	std::setfill('0') << std::setw(5) << j << ".ppm";

      counter++;

      if(left_image == NULL or right_image == NULL) {
	ROS_ERROR_STREAM("Null images. Did you start the camera driver? Did one of the drivers died?. Exiting...");
	exit(-1);
      }

      //write the images to files
      ROS_INFO_STREAM("Wrting image:" << left_filename.str());
      cv::imwrite(left_filename.str(),*left_image);
      ROS_INFO_STREAM("Writing image:" << right_filename.str());
      cv::imwrite(right_filename.str(),*right_image);

    }
  }
  /* and go home */
  ptu_go_to(cer, 0, 0);

  estrap_out(cer);
  return 0;
}
