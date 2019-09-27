#ifndef __PAN_TILT_DRIVER_H__
#define __PAN_TILT_DRIVER_H__

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <memory>


extern "C" {
#include <estrap.h>
}

#include <iostream>
#include <memory>

class PanTiltDriver {

 public:
  PanTiltDriver(int argc, char** argv,bool simulation=false);

  void set_ascii_feedback();
  bool ptu_goto(float pan_angle, float tilt_angle);
  bool goto_limits();
  bool _ptu_goto(int pan,int tilt);
  bool reset();
  bool get_limits(int &pan_max, int &pan_min, 
		  int &tilt_max, int &tilt_min,
		  int &pan_speed_limit, int &tilt_speed_limit);

  bool set_speed(int pan_speed, int tilt_speed);

 protected:
  struct cerial   *cer;
  int pan_max, pan_min, tilt_max, tilt_min;
  int pan_speed_limit, tilt_speed_limit;
  static const int PAN_HOME=0;
  static const int TILT_HOME=0;
  bool simulation;
};

 typedef std::shared_ptr<PanTiltDriver> PanTiltDriverPtr;


#endif //__PAN_TILT_DRIVER_H__
