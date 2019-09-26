#include <pan_tilt_driver/pan_tilt_driver.h>
#include <unistd.h>

PanTiltDriver::PanTiltDriver(int argc, char** argv, bool simulate):
  simulation(simulate) {

  if(simulation) {
    std::cout << "!!!!Pan-tilt driver running in simulation mode!!!!" << std::endl;
    return;
  }
  
  if((cer = estrap_in(argc, argv)) == NULL) {
    std::cerr << "Could not open the serial device. Exiting..." << std::endl;
    exit(-1);
  }
  std::cout << "Opening the serial connection..." << std::endl;

  set_ascii_feedback();
  reset();
  get_limits(pan_max, pan_min, tilt_max, tilt_min, pan_speed_limit, tilt_speed_limit);
  std::cout << "Pan max:" << pan_max << " min:" << pan_min <<
    "Tilt max:" << tilt_max << " min:" << tilt_min << std::endl;
}

void PanTiltDriver::set_ascii_feedback() {

  std::cout << "Setting ascii feedback" << std::endl;

  if(simulation) {
    return;
  }
  
  uint16_t status;
  // Set terse mode
  if(cpi_ptcmd(cer, &status, OP_FEEDBACK_SET, CPI_ASCII_FEEDBACK_TERSE)){
    std::cout << "Failed to set feedback mode. Exiting" << std::endl;
    exit(-1);
  }
}

bool PanTiltDriver::ptu_goto(float pan_angle, float tilt_angle) {
  int pan_val = -(int)((pan_max/168.)*pan_angle);
  //168 degree with limits
  //188 degrees with limits disabled
  int tilt_val = 0;
  if(tilt_angle > 0) {
    tilt_val = (int)((tilt_max/30.)*tilt_angle);
  } else {
    tilt_val = -(int)((tilt_min/90.)*tilt_angle);
  }
  
  return _ptu_goto(pan_val,tilt_val);
}

bool PanTiltDriver::goto_limits() {
  _ptu_goto(0,tilt_max);
  usleep(5*1e6);
  _ptu_goto(0,tilt_min);
  usleep(5*1e6);
  _ptu_goto(pan_max,0);
  usleep(5*1e6);
  _ptu_goto(pan_min,0);
  usleep(5*1e6);
  _ptu_goto(0,0);
  return true;
}


bool PanTiltDriver::_ptu_goto(int pan, int tilt) {

  std::cout << "Goto pan:" << pan << " tilt:" << tilt << std::endl;
  uint16_t status;

  if(simulation) {
    return true;
  }
  
  if(cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, pan) ||
     cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_SET, tilt)) {
    std::cerr << "Failed to go to min position." << std::endl;
    return false;
  }

  if(cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, pan) ||
     cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, tilt)) {
    std::cerr << "Blocking for reset completion failed." << std::endl;
    return false;
  }

  return true;
}


bool PanTiltDriver::reset() {

  std::cout << "Resetting the ptu" << std::endl;
  uint16_t status;

  if(simulation) {
    return true;
  }
  
  if(cpi_ptcmd(cer, &status, OP_RESET, CPI_RESET_ALL)) {
    std::cerr << "Failed to execute Reset. " << std::endl;
    return false;
  }

  //if(cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, PAN_HOME) ||
  //   cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, TILT_HOME)) {

  //  std::cerr<< "Blocking failed." << std::endl;
  //  return false;
  //}
  usleep(2000);

  std::cout << "DEBUG: Finished resetting" << std::endl;
  return true;
}

bool PanTiltDriver::get_limits(int &pan_max, int &pan_min, 
			       int &tilt_max, int &tilt_min,
			       int &pan_upper_speed, int &tilt_upper_speed) {

  if(simulation) {
    return true;
  }
  
  uint16_t status;
  // read min/max position and speed 
  if(cpi_ptcmd(cer, &status, OP_PAN_MAX_POSITION, &pan_max) ||
     cpi_ptcmd(cer, &status, OP_PAN_MIN_POSITION, &pan_min) ||
     cpi_ptcmd(cer, &status, OP_TILT_MAX_POSITION, &tilt_max) ||
     cpi_ptcmd(cer, &status, OP_TILT_MIN_POSITION, &tilt_min) ||
     cpi_ptcmd(cer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET, &pan_upper_speed) ||
     cpi_ptcmd(cer, &status, OP_TILT_UPPER_SPEED_LIMIT_GET, &tilt_upper_speed)){
    std::cerr << "Basic unit queries failed." << std::endl;
    return false;
  }
  return true;
}


bool PanTiltDriver::set_speed(int pan_speed, int tilt_speed) {

  if(simulation) {
    return true;
  }
  
  uint16_t status;
  if(cpi_ptcmd(cer, &status, OP_PAN_DESIRED_SPEED_SET, pan_speed ) ||
     cpi_ptcmd(cer, &status, OP_TILT_DESIRED_SPEED_SET, tilt_speed )) {
    std::cerr << "Setting PS/TS failed." << std::endl;
    return false;
  }
  return true;
}

