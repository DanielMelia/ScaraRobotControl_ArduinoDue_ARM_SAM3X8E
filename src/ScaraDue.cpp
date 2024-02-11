#include "ScaraDue.h"
#include "math.h"

/** NOTES AND TODOS
 * _toggleMotorPin : test using |= instead of = to set individual bits
 * Init : Not finished
 * _checkValidTargetPosition : only checking XYZ, not R
 * Move_TrajectoryMode : set isMovingZ and isMovingXYR = false at the beginning
 * checkPositionIsWithinWS : need to add limit switches logic. Not checking R axis
 * Documenting the code https://www.doxygen.nl/manual/docblocks.html
*/

ScaraDue SCARA;

// By default GCC shows a warning on #pragma region
// On platform.ini add:
//    build_flags = 
//        -Wno-unknown-pragmas


#pragma region Setup_Stepper_Motors

/// @brief Configure a GPIO pin as an Input/Output and configure the pull-up resistors
/// @param GPIO_x GPIO port
/// @param pin_number GPIO pin number
/// @param mode pin mode: Output, Input or Input_PullUp
void GPIO_Pin_Setup(Pio* GPIO_x, uint8_t pin_number, PinMode mode){
  if(mode == Output){
    GPIO_x->PIO_PER |= (1<<pin_number);  // Enable the GPIO pin
    GPIO_x->PIO_OER |= (1<<pin_number);  // Enable output mode for the GPIO pin (set it as output)
  }else if(mode == Input){
    GPIO_x->PIO_PER |= (1<<pin_number);  // Enable the GPIO pin
    GPIO_x->PIO_ODR |= (1<<pin_number);  // Disable output mode for the GPIO pin (set it as input)
    GPIO_x->PIO_PUDR |= (1<<pin_number); // Disable pull-up resistor for the GPIO pin
  }else if(mode == Input_PullUp){
    GPIO_x->PIO_PER |= (1<<pin_number);  // Enable the GPIO pin
    GPIO_x->PIO_ODR |= (1<<pin_number);  // Disable output mode for the GPIO pin (set it as input)
    GPIO_x->PIO_PUER |= (1<<pin_number); // Enable pull-up resistor for the GPIO pin
  }
}

/// @brief Read GPIO pin state
/// @param GPIO_x  GPIO port
/// @param pin_number GPIO pin number
/// @return TRUE if pin is set HIGH. FALSE otherwise
bool GPIO_Pin_Read(Pio* GPIO_x, uint8_t pin_number){
  // Read the Pin Data Status Register
  return (GPIO_x->PIO_PDSR & (1 << pin_number)) != 0; 
  // Inequality comparison checks if the result of the bitwise and is not zero. If result is NOT ZERO, means pin is HIGH, and will return TRUE
  // (PIOA_PDSR & (1 << pin)) ? HIGH : LOW // this uses ternary (or conditional) operator, and returs a uint8_t
}

/// @brief Writes HIGH/LOW to a GPIO pin
/// @param GPIO_x  GPIO port
/// @param pin_number GPIO pin number
/// @param value TRUE sets the pin to HIGH. FALSE clears the pin
void GPIO_Pin_Write(Pio* GPIO_x, uint8_t pin_number, bool value){
  if(value)
    GPIO_x->PIO_SODR |= (1 << pin_number); // Set output high
  else
    GPIO_x->PIO_CODR |= (1 << pin_number); // Set output low
}

/** Sets the GPIO port that controls the stepper motors
* @param GPIO_x Pio Hardware Register. Select PIOA, PIOB, PIOC, PIOD
*/
void ScaraDue::SetSteppersGPIOPort(Pio* GPIO_x){
  _GPIO_STEPPERS = GPIO_x;
  Serial.println("1 - Steppers GPIO Port Set");
}

/** Set axis Z main parameters
 * @param PPR axis pulses per revolution
 * @param pitch axis pitch in mm per revolution
 * @param pulse_pin port pin number (not physical pin) of the pulse signal
 * @param dir_pin pin used for setting motor direction (port pin unmber)
*/
void ScaraDue::SetAxis_Z(uint32_t PPR, float pitch, uint8_t pulse_pin, uint8_t dir_pin){
  _PLS_PIN_Z = pulse_pin;
  _DIR_PIN_Z = dir_pin;
  Z._ppr = PPR;
  Z._pitch_mm = pitch;
  Z._minStep = Z._pitch_mm / Z._ppr;
  //Z._minStep = 2 * PI / Z._ppr;
  Z._arePinsConfigured = true;
  Serial.println("2 - Z Axis Set");
}

/** Toggle Z pulse pin
*/
void ScaraDue::_step_Z(){
  _toggleMotorPin(_PLS_PIN_Z, Z._pulseStatus);
}

/** Set direction of the Z motor. Direction depends on the motor wiring
 * @param dir set True of False to move CW or CCW
*/
void ScaraDue::_setDirection_Z(bool dir){
  if (dir == true) {
    _setMotorPin(_DIR_PIN_Z);
    Z._dir = 1;
  } else {
    _clearMotorPin(_DIR_PIN_Z);
    Z._dir = -1;
  }  
}

/** Set axis TH1 main parameters
 * @param PPR axis pulses per revolution
 * @param cw_pulse_pin port pin number (not physical pin) to execute one step in CW direction
 * @param ccw_pulse_pin port pin number (not physical pin) to execute one step in CCW direction
*/
void ScaraDue::SetAxis_TH1(uint32_t PPR, uint8_t cw_pulse_pin, uint8_t ccw_pulse_pin){
  _PLSCW_PIN_TH1 = cw_pulse_pin;
  _PLSCCW_PIN_TH1 = ccw_pulse_pin;
  TH1._ppr = PPR;
  TH1._minStep = 2 * PI / TH1._ppr;
  TH1._arePinsConfigured = true;
  Serial.println("3 - TH1 Axis Set");
}

/** Toggle TH1 pulse pin
*/
void ScaraDue::_step_TH1(){
  if (TH1._dir == 1)
    _toggleMotorPin(_PLSCW_PIN_TH1, TH1._pulseStatus);
  else
    _toggleMotorPin(_PLSCCW_PIN_TH1, TH1._pulseStatus);  
}

/** Set direction of the TH1 motor. Driver must be set to 2-pulse input mode
 * Check Oriental Motor driver datasheet
 * @param dir set True of False to move CW or CCW
*/
void ScaraDue::_setDirection_TH1(bool dir){
  if (dir == true)
    TH1._dir = 1;
  else
    TH1._dir = -1;
}

/** Set axis TH2 main parameters
 * @param PPR axis pulses per revolution
 * @param cw_pulse_pin port pin number (not physical pin) to execute one step in CW direction
 * @param ccw_pulse_pin port pin number (not physical pin) to execute one step in CCW direction
*/
void ScaraDue::SetAxis_TH2(uint32_t PPR, uint8_t cw_pulse_pin, uint8_t ccw_pulse_pin){
  _PLSCW_PIN_TH2 = cw_pulse_pin;
  _PLSCCW_PIN_TH2 = ccw_pulse_pin;
  TH2._ppr = PPR;   
  TH2._minStep = 2 * PI / TH2._ppr;
  TH2._arePinsConfigured = true;
  Serial.println("4 - TH2 Axis Set");
}

/** Toggle TH2 pulse pin
*/
void ScaraDue::_step_TH2(){
  if (TH2._dir == 1)
    _toggleMotorPin(_PLSCW_PIN_TH2, TH2._pulseStatus);
  else
    _toggleMotorPin(_PLSCCW_PIN_TH2, TH2._pulseStatus); 
}

/** Set direction of the TH2 motor. Driver must be set to 2-pulse input mode
 * Check Oriental Motor driver datasheet
 * @param dir set True of False to move CW or CCW
*/
void ScaraDue::_setDirection_TH2(bool dir){
  if (dir == true)
    TH2._dir = 1;
  else
    TH2._dir = -1;
}

/** Set axis TH3 main parameters
 * @param PPR axis pulses per revolution
 * @param pulse_pin port pin number (not physical pin) of the pulse signal
 * @param dir_pin pin used for setting motor direction (port pin unmber)
*/
void ScaraDue::SetAxis_TH3(uint32_t PPR, uint8_t pulse_pin, uint8_t dir_pin){
  _PLS_PIN_TH3 = pulse_pin;
  _DIR_PIN_TH3 = dir_pin;
  TH3._ppr = PPR;
  TH3._minStep = 2 * PI / TH3._ppr;
  TH3._arePinsConfigured = true;
  Serial.println("5 - TH3 Axis Set");
}

/** Toggle TH3 pulse pin
*/
void ScaraDue::_step_TH3(){
  _toggleMotorPin(_PLS_PIN_TH3, TH3._pulseStatus);
}

/** Set direction of the TH3 motor. Direction depends on the motor wiring
 * @param dir set True of False to move CW or CCW
*/
void ScaraDue::_setDirection_TH3(bool dir){
  if (dir) {
    _setMotorPin(_DIR_PIN_TH3);
    TH3._dir = 1;
  } else {
    _setMotorPin(_DIR_PIN_TH3);
    TH3._dir = -1;
  }  
}

/** Enable motors GPIO Port registers and configure them as Outputs
*/
bool ScaraDue::_initialiseMotorPins(){
  if(Z._arePinsConfigured && TH1._arePinsConfigured && TH2._arePinsConfigured & TH3._arePinsConfigured){

    uint32_t pin_mask = (1<<_PLS_PIN_Z) | (1<<_DIR_PIN_Z) | (1<<_PLSCW_PIN_TH1) | (1<<_PLSCCW_PIN_TH1) | (1<<_PLSCW_PIN_TH2) | (1<<_PLSCCW_PIN_TH2) | (1<<_PLS_PIN_TH3) | (1<<_DIR_PIN_TH3);
      // Enable PIO (Peripheral I/O) for the specified pins
      _GPIO_STEPPERS->PIO_PER |= pin_mask;
      // Set pins as output
      _GPIO_STEPPERS->PIO_OER |= pin_mask; 
      Serial.println("6 - Pins Initialisation SUCCESS");
      return true;
  }else{
    Serial.println("6 - Pins Initialisation FAIL");
    return false;
  }
}

//NOTE: test using |= instead of = to set individual bits
void ScaraDue::_toggleMotorPin(uint32_t pin_mask, volatile bool &pin_status) {
  if (pin_status) {
    _GPIO_STEPPERS->PIO_CODR |= (1<<pin_mask); // Clear Output Data Register
    pin_status = false;
  } else {
    _GPIO_STEPPERS->PIO_SODR |= (1<<pin_mask); // Set Output Data Register
    pin_status = true;
  }
}
void ScaraDue::_setMotorPin(uint32_t pin_mask) {
  _GPIO_STEPPERS->PIO_SODR |= (1 << pin_mask);
}
void ScaraDue::_clearMotorPin(uint32_t pin_mask) {
  _GPIO_STEPPERS->PIO_CODR |= (1 << pin_mask);
}

#pragma endregion



/** Initialise SCARA robot
 * (Not Finished)
*/
bool ScaraDue::Init(){
  // configure Timers
  _prescaler = 8;

  // configure / Initialise axes
  bool init_motor_pins = _initialiseMotorPins();
  _lowest_z = 100;

  Z._minPos = 0;
  Z._maxPos = 100;
  _L1 = 200;
  _L2 = 200;

  return init_motor_pins;
}

/** Run this function on the main loop to update periodically the robot position.
 * The position calulation is based on the motor step count, which is done on the timer interrupts
*/
void ScaraDue::UpdateEndEffectorCartesianPosition(){
  if(_isHomed){
    // disable interrupts momentarily to avoid a timer interrupt changing the value of the step position while reading
    noInterrupts();
    Z._pos = Z._posSteps * Z._minStep;
    TH1._pos = TH1._posSteps * TH1._minStep;
    TH2._pos = TH2._posSteps * TH2._minStep;
    TH3._pos = TH3._posSteps * TH3._minStep;
    interrupts();
    _curr_x = float(_L1 * cos(TH1._pos) + _L2 * cos(TH1._pos + TH2._pos));
    _curr_y = float(_L1 * sin(TH1._pos) + _L2 * sin(TH1._pos + TH2._pos));
    _curr_z = float(Z._pos);
    _curr_r = float(TH3._pos + (TH1._pos + TH2._pos)); 
  }
  else{
    _curr_x = -999;
    _curr_y = -999;
    _curr_z = -999;
    _curr_r = -999;     
  }
}

/** Get Current Robot Position
 * @return current end-effector position
*/
PositionXYZR ScaraDue::GetCurrentPosition(){
  PositionXYZR currPos;
  if(_isHomed){        
    currPos.X = _curr_x;
    currPos.Y = _curr_y;
    currPos.Z = _curr_z;
    currPos.R = _curr_r; 
  }else{
    currPos.X = -999;
    currPos.Y = -999;
    currPos.Z = -999;
    currPos.R = -999;     
  }
  return currPos;
}

#pragma region Run_Control_Modes

/** Move the motors a given number of steps at a given frequency
 * @param steps_z number of steps to move in the z motor
 * @param steps_th1 number of steps to move in the th1 motor
 * @param steps_th2 number of steps to move in the th2 motor
 * @param steps_th3 number of steps to move in the th3 motor
 * @param freq_hz frequency of the steps in hz
 * @return true if successfull
*/
bool ScaraDue::Move_ManualMode(int32_t steps_z, int32_t steps_th1, int32_t steps_th2, int32_t steps_th3, uint32_t freq_hz){
  // Make sure the robot is in stop mode
  if(controlMode != STP)
    return false;

  // Set Motors Direction
  if(steps_z > 0){
    Serial.println("Set Dir Pin");
    _setDirection_Z(true);
  }
  else{
    Serial.println("Clear Dir Pin");
    _setDirection_Z(false);
  }

  if(steps_th1 > 0)
    _setDirection_TH1(true);
  else
    _setDirection_TH1(false);

  if(steps_th2 > 0)
    _setDirection_TH2(true);
  else
    _setDirection_TH2(false);

  if(steps_th3 > 0)
    _setDirection_TH3(true);
  else
    _setDirection_TH3(false);    

  // Set the number of steps to move for each motor
  Z._targetSteps = abs(steps_z);
  TH1._targetSteps = abs(steps_th1);
  TH2._targetSteps = abs(steps_th2);
  TH3._targetSteps = abs(steps_th3);
  Serial.print("trg _freq: ");Serial.println(freq_hz);
  // Set motors _frequency
  Z._freq = freq_hz;
  TH1._freq = freq_hz;
  TH2._freq = freq_hz;
  TH3._freq = freq_hz;
  Serial.print("stepper _freq: ");Serial.println(freq_hz);

  // Start Timers
  controlMode = MAN;
  if(Z._targetSteps > 0)
    _startZTimer();
  if(TH1._targetSteps > 0)
    _startTh1Timer();
  if(TH2._targetSteps > 0)
    _startTh2Timer();
  if(TH3._targetSteps > 0)
    _startTh3Timer();   
  Serial.println("7 - Move Manual Mode Started");
  return true;     
}

/** Move the robot with a pre-defined 3rd-order polynomial trajectory.
 * Uses cartesian space coordinates
 * @param trg_x target position in the X axis
 * @param trg_y target position in the Y axis
 * @param trg_z target position in the Z axis
 * @param trg_r target position in the R axis
 * @param t_xyr time to execute the trajectory in the XY-R plane
 * @param t_z time to execute the trajectory in the Z axis
 * @return true if successfull
*/
bool ScaraDue::Move_TrajectoryMode(float trg_x, float trg_y, float trg_z, float trg_r, float t_xyr, float t_z){
  // Make sure the robot is in stop mode
  if(controlMode != STP)
    return false; 
  // 1 - Check if target position is within range of motion
  if(!_isHomed)
    return false;

  // Initialise variables
  // _isMovingZ = false;
  // _isMovingXYR = false;

  bool trg_ok = _checkValidTargetPosition(trg_x, trg_y, trg_z, trg_r);
  if (!trg_ok) 
    return false;

  // 2 - Check if trajectory in the XY plane can be done in a straight line
  //bool traj_ok = _checkValidTrajectory(trg_x, trg_y);

  // 2 - Check if z is moving up or down
  if(trg_z > _curr_z + 0.01){
    _isMovingZ = true;
    _isMovingUp = true;
  }
  else if (trg_z < _curr_z - 0.01){
    _isMovingZ = true;
    _isMovingUp = false;   
  }
  else
    _isMovingZ = false;

  // 3 - Check if the robot is moving in the XYR plane
  if (abs(trg_x - _curr_x) > 0.01 || abs(trg_y - _curr_y) > 0.01 || abs(trg_r - _curr_r) > 0.0017)
    _isMovingXYR = true;
  else
    _isMovingXYR = false;

  // 4 - Compute trajectories
  float t_r = 0;
  _checkTrajectoryTimes(trg_x, trg_y, trg_z, trg_r, t_z, t_xyr, t_r);
  t_xyr = max(t_xyr, t_r);
  if (_isMovingZ) //Compute Z trajectory  
    _trajectoryCalculationZ(t_z, trg_z);
  if (_isMovingXYR) //Compute XY+R trajectory
    _trajectoryCalculationXYR(t_xyr, trg_x, trg_y, trg_r);

  // 5 - If moving UP, move Z first and then rest. If moving DOWN, move XYR plane first and then Z
  _setVelocityDirectionFlags(0,0,0,0);
  if ((_isMovingUp && _isMovingZ) || (_isMovingZ && !_isMovingXYR)) {   
    //Initialise z timer
    _isMovingZ = false;
    _startZTimer();
  } else {
    //Initialise other timers
    if (_isMovingXYR) {      
      _isMovingXYR = false;
      _startXYRTimers();
    }

  }  

  return true;

 
}

/// @brief Move in Trajectory Mode but allows moving in one direction only by indicating the distance to move and the direction
/// @param distance distance to move in mm in the case of XYZ axes, or radians in the case of R
/// @param axis axis to move - Z(0), X(1), Y(2), R(3)
/// @param t_time trajectory time
/// @return true if successfull
bool ScaraDue::Move_TrajectoryMode_SingleAxis(float distance, int axis, float t_time){
  // Make sure the robot is in stop mode
  if(controlMode != STP)
    return false;
  if(!_isHomed)
    return false;  
  // 1 - Check if motion occurs in Z axis or XYR plane and is within limits
  if (axis == 0) {
    _isMovingZ = true;
    _isMovingXYR = false;
  } else if (axis == 1 || axis == 2 || axis == 3) {
    _isMovingXYR = true;
    _isMovingZ = false;
  } else{
    return false;
  }

  // 2 - Check trajectory time
  if (t_time < 0.2) 
    t_time = 0.2;  

  // 3 - Compute and execute trajectories
  // _ti_xyr = t_time / vec_length;
  // _ti_z = t_time / vec_length;
  // Z._freq = round(1 / _ti_z);
  // TH1._freq = round(1 / _ti_xyr);
  // TH2._freq = round(1 / _ti_xyr);
  // TH3._freq = round(1 / _ti_xyr);

  if (_isMovingZ) {
    _trajectoryCalculationZ(t_time, _curr_z + distance);
    _setVelocityDirectionFlags(0, 0, distance, 0);
    _startZTimer();
  }
  else if (_isMovingXYR) {
    //Compute XY+R trajectory
    if (axis == 1) {
      _trajectoryCalculationXYR(t_time, _curr_x + distance, _curr_y, _curr_r);
      _setVelocityDirectionFlags(distance, 0, 0, 0);
    } else if (axis == 2) {
      _trajectoryCalculationXYR(t_time, _curr_x, _curr_y + distance, _curr_r);
      _setVelocityDirectionFlags(0, distance, 0, 0);
    } else if (axis == 3) {
      _trajectoryCalculationXYR(t_time, _curr_x, _curr_y, _curr_r + distance);
      _setVelocityDirectionFlags(0, 0, 0, distance);
    }
    _startXYRTimers();
  }
  return true;
}

/// @brief Sets the Z-axis to move down following a trapezoidal trajectory while monitoring the air pressure level to detect an object pick-up or drop event
/// @param lowest_z_mm minimum allowed Z-level. the robot will stop if reached
/// @param vf_mms descent velocity in mm per second
/// @param pressureThreshold air pressure leevel that will make the robot stop if reached
/// @param descent_mode 1 -> Controlled descend for pickup, 2 -> Controlled descend for jig load
/// @param acc_dist_mm distance in mm until final velocity is reached
/// @return TRUE if successful
bool ScaraDue::setupDescendModeTrajectory(float lowest_z_mm, float vf_mms, float pressure_threshold, uint8_t descent_mode, float acc_dist_mm){
  // Make sure the robot is in stop mode
  if(controlMode != STP)
    return false;
  if (descent_mode != 1 && descent_mode != 2)
    return false;
  _lowest_z = lowest_z_mm;
  _descentMode = descent_mode;
  _refAirPressure = pressure_threshold;
  _descendMode_result = 0;

  _setVelocityDirectionFlags(0,0,-1,0);

  if(acc_dist_mm < 0)
    acc_dist_mm = 1;

  uint32_t _max_wf_freq = VARIANT_MCK / (2 * _prescaler);
  _accSteps = (int)(acc_dist_mm * Z._ppr / Z._pitch_mm);
  int trg_freq = (int)(vf_mms * Z._ppr / Z._pitch_mm);
  float acc = _convert_AccSteps_2_AccRads2(trg_freq, _accSteps, Z._minStep);
  _c0 = (int)(0.676 * _max_wf_freq * sqrt(2 * Z._minStep / acc));

  Z._targetSteps = (long)((_curr_z - _lowest_z)/Z._minStep);
  //DescendMode_TotalSteps = (long)((_curr_z - lowest_z)/Z._minStep);
  if(_accSteps*2 > Z._targetSteps){ // DescendMode_TotalSteps = Z._targetSteps
    _accSteps = (int)(Z._targetSteps/2);
  }
  _decStart = Z._targetSteps - _accSteps;
  //DescendMode_DecStart = DescendMode_TotalSteps - _accSteps;
              
  Z._freq = (VARIANT_MCK / _prescaler) / _c0;
  _prev_counter_value = _c0;
  _rest = 0;  

  _setDirection_Z(false); // Configure motor
  //Z._freq = VM_Z_f0;
  Z._currentSteps = 0;
  //DescendMode_CurrentStepCount = 0;
  controlMode = VEL;   // Set Control Mode (Speed Control)
  _stopVelMode = false;

  _startZTimer();     

  return true;
}

// CHECKS VALID POSITION BEFORE MOTION
/** Use before setting a new trajectory to check if the target position is within the robot workspace
 * @param x target X-coordinate
 * @param y target Y-coordinate
 * @param z target Z-coordinate
 * @param r target R-coordinate
 * @return TRUE if the target position is within the workspace. FALSE otherwise
*/
bool ScaraDue::_checkValidTargetPosition(float x, float y, float z, float r) {
  // Target position in the XY plane should be within a circular region of less than _workSpace_XY_min_radius but more than _workSpace_XY_min_radius.
  // Also, if y is negative then, x cannot be within the range -_workSpace_XY_min_radius to _workSpace_XY_min_radius.
  // If target meets the criteria the function returns TRUE

  // Calculate radius of the target position in the XY plane
  float radius = sqrt(sq(x) + sq(y));

  // Check that XY coordinates meets the criteria
  if (radius < _workSpace_XY_min_radius || radius > _workSpace_XY_max_radius)
    return false;
  if (y < 0) {
    if (x > -_workSpace_XY_min_radius && x < _workSpace_XY_min_radius)
      return false;
  }

  // Check that Z target coordinate is within the Z limits
  if (z < _workSpace_Z_min || z > _workSpace_Z_max)
    return false;

  return true;

}

/** Check that trajectory times are valid and the robot doesn't exeed the maximum allowed speeds on each axis. Otherwise, re-calculate times based on maximum allowed speeds
 * @param trg_x target position X-coordinate
 * @param trg_y target position Y-coordinate
 * @param trg_z target position Z-coordinate
 * @param trg_r target position R-coordinate
 * @param t_z trajectory time in the Z-axis
 * @param t_xy trajectory time in the XY-plane
 * @param t_r trajectory time in the R-axis
*/
void ScaraDue::_checkTrajectoryTimes(float trg_x, float trg_y, float trg_z, float trg_r, float &t_z, float &t_xy, float &t_r) {
  // Calculate peak v_z based on trajectory time and target position
  float v_z = abs(1.5 * (trg_z - _curr_z) / t_z);

  //Serial.print("Z Max Vel: ");Serial.println(max_z_vel);
  //Serial.print("Z Vel: ");Serial.println(v_z);

  // If the peak velocity exceeds the maximum allowed speed, then re-calculate the trajectory time
  if (v_z > _maxSpeed_Z)
    t_z = abs(1.5 * (trg_z - _curr_z) / _maxSpeed_Z);
  
  // Do the same for XY and R
  float v_x = abs(1.5 * (trg_x - _curr_x) / t_xy);
  float v_y = abs(1.5 * (trg_y - _curr_y) / t_xy);

  float v_xy = v_x, trg_xy = trg_x, curr_xy = _curr_x;
  if (v_y > v_x) {
    v_xy = v_y;
    trg_xy = trg_y;
    curr_xy = _curr_y;
  }

  //Serial.print("v_x: "); Serial.println(v_x);
  //Serial.print("v_y: "); Serial.println(v_y);
  //Serial.print("v_xy: "); Serial.println(v_xy);

  if (v_xy > _maxSpeed_XY)
    t_xy = abs(1.5 * (trg_xy - curr_xy) / _maxSpeed_XY);


  float v_r = abs(1.5 * (trg_r - _curr_r) / t_xy);
  if (v_r > _maxSpeed_R)
    t_r = abs(1.5 * (trg_r - _curr_r) / _maxSpeed_R);
  else
    t_r = t_xy;
  
}

/** Calculates the 3rd order polinomial trajectory in the Z axis, and performs the trajectory digitization by converting the angle of rotation into motor steps. 
 * Updates the _mZ_steps array with the number of steps to perform at each time interval
 * @param tf total trajectory time
 * @param tg_z target position Z-coordinate
*/
void ScaraDue::_trajectoryCalculationZ(float tf, float tg_z) {  //Trajectory calculation for the vertical direction (Z axis)

  float rz[vec_length];  //Create array (float) to store trajectory on one axis

  _zCartesianPath(rz, _curr_z, tg_z, tf, vec_length);
  //cartesian_path(rz, vec_length - 1, tg_z, _curr_z, tf); //Compute cartesian path (3rd order polynomial) and store data in rz array

  //Takes into account the minimum step of the motors to adjust joint trajectories to fit the closest full-step position
  _trajDigitization(rz, _mZ_steps, vec_length, Z._minStep, false);  //Store values in m?_steps array (of int)

  // Update Z-trajectory time interval and Z motor timer frequency
  _ti_z = tf / vec_length;
  Z._freq = round(1 / _ti_z);

//  Serial.print("Min step: "); Serial.println(Z._minStep, 6);
//  Serial.println("STEPS VECTORS:");
//  long z_tot_steps = 0;
//  for (int i = 0; i <= 99; i++) {
//    Serial.print(_mZ_steps[i]);
//    z_tot_steps += _mZ_steps[i];
//    Serial.print("\t");
//  }
//  Serial.println("");
//  Serial.print("z_tot_steps: "); Serial.println(z_tot_steps);
//  Serial.println("");
  
}


/** Calculates the 3rd order polinomial trajectory in the XYR spaces, converts it into joint space coordinates, and performs the trajectory digitization
 * by converting the angle of rotation into motor steps. 
 * @param tf total trajectory time in seconds
 * @param tg_x target position X-coordinate
 * @param tg_y target position Y-coordinate
 * @param tg_r target position R-coordinate
*/
void ScaraDue::_trajectoryCalculationXYR(float tf, float tg_x, float tg_y, float tg_r) {  //Trajectory calculation for the robot arm (in XY axes)

  float rx[vec_length];  //Create array (float) to store trajectory on one axis
  float ry[vec_length];  //Length of the array is defined by vec_length
  //NOTE: Arrays take a lot of memory. They are defined inside function, so memory is released once function call ends

  // Check if trajectory in the XY plane can be done in a straight line
  bool traj_ok = _checkValidTrajectory(tg_x, tg_y);

  // If traj_ok, calculate trajectory in straight line
  if (traj_ok) {
    _xyCartesianPath(rx, ry, _curr_x, _curr_y, tg_x, tg_y, tf, vec_length);
    //cartesian_path(rx, vec_length - 1, tg_x, _curr_x, tf); //Compute cartesian path (3rd order polynomial) and store data in rx and ry arrays
    //cartesian_path(ry, vec_length - 1, tg_y, _curr_y, tf);

  // else, calculate trajectory with via points
  } else {
    float vp1_x, vp1_y, vp2_x, vp2_y;
    if (_curr_x < -_workSpace_XY_min_radius && tg_x < _workSpace_XY_min_radius) {
      vp1_x = -_workSpace_XY_min_radius;
      vp1_y = 175;
      vp2_x = 0;
      vp2_y = 0;
    } else if (_curr_x < -_workSpace_XY_min_radius && tg_x > _workSpace_XY_min_radius) {
      vp1_x = -_workSpace_XY_min_radius;
      vp1_y = 175;
      vp2_x = _workSpace_XY_min_radius;
      vp2_y = 175;
    } else if (_curr_x > _workSpace_XY_min_radius && tg_x > -_workSpace_XY_min_radius) {
      vp1_x = _workSpace_XY_min_radius;
      vp1_y = 175;
      vp2_x = 0;
      vp2_y = 0;
    } else if (_curr_x > _workSpace_XY_min_radius && tg_x < -_workSpace_XY_min_radius) {
      vp1_x = _workSpace_XY_min_radius;
      vp1_y = 175;
      vp2_x = -_workSpace_XY_min_radius;
      vp2_y = 175;
    } else if (_curr_x > -_workSpace_XY_min_radius && _curr_x < _workSpace_XY_min_radius && tg_x < -_workSpace_XY_min_radius) {
      vp1_x = -_workSpace_XY_min_radius;
      vp1_y = 175;
      vp2_x = 0;
      vp2_y = 0;
    } else if (_curr_x > -_workSpace_XY_min_radius && _curr_x < _workSpace_XY_min_radius && tg_x > _workSpace_XY_min_radius) {
      vp1_x = _workSpace_XY_min_radius;
      vp1_y = 175;
      vp2_x = 0;
      vp2_y = 0;
    }
    else{return;}
    //Serial.println("VP_x:");
    _xyCartesianPathWithViaPoints(rx, ry, _curr_x, _curr_y, tg_x, tg_y, vp1_x, vp2_x, vp1_y, vp2_y, tf, vec_length);
    //cartesianPathWithViaPoints(rx, vec_length - 1, tg_x, _curr_x, tf, vp1_x, vp2_x);
    //Serial.println("VP_y:");
    //cartesianPathWithViaPoints(ry, vec_length - 1, tg_y, _curr_y, tf, vp1_y, vp2_y);
  }

  //  Serial.println("CARTESIAN PATH:");
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.print(rx[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.println("");
  //  //SerialUSB.println(" ");
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.print(ry[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.println("");
  //  Serial.println("");
  
  // Calculate angular trajectories of TH1 and TH2 joints
  _xyToAngularTrajectories(rx, ry, vec_length);  //Convert cartesian data in rx and ry arrays to joint trajectories

  //  Serial.println("ANGULAR TRAJECTORIES:");
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.print(rx[i], 3);
  //    Serial.print("\t");
  //  }
  //  Serial.println("");
  //  //SerialUSB.println(" ");
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.print(ry[i], 3);
  //    Serial.print("\t");
  //  }
  //  Serial.println("");
  //  //Serial.println("");
  //Serial.print("t_i: ");Serial.println(ti_xyr,4);
  // Serial.println("ANGULAR VELOCITIES:");
  // for (int i = 1; i <= 99; i++) {
  //   double th1_d = (rx[i] - rx[i-1]) / ti_xyr;
  //   double th2_d = (ry[i] - ry[i-1]) / ti_xyr;
  //   Serial.print(th1_d);
  //   Serial.print("\t");
  //   Serial.println(th2_d); 
  // }
  // Serial.println("");
  // //SerialUSB.println(" ");
  // for (int i = 1; i <= 99; i++) {
  //   double th2_d = (ry[i] - ry[i-1]) / ti_xyr;
  //   Serial.print(th2_d);
  //   Serial.print("\t");
  // }
  // Serial.println("");
  //Serial.println("");

  // Calculate angular trajectory of TH3 joint
  float r_th[vec_length];  //Length of the array is defined by vec_length
  _th3AngularTrajectory(rx, ry, r_th, _curr_r, tg_r, tf, vec_length);
  //th3_trajectory(rx, ry, r_th, vec_length, tg_r, _curr_r, tf); // Compute the trajectory of the rotation axis

  //   Serial.println("R AXIS TRAJECTORY:");
  //   for (int i = 0; i <= 99; i++) {
  //     Serial.print(r_th[i],5);
  //     Serial.print("\t");
  //   }
  //   Serial.println("");
  //   Serial.println("");


  //Takes into account the minimum step of the motors to adjust joint trajectories to fit the closest full-step position
  _trajDigitization(rx, _mTH1_steps, vec_length, TH1._minStep, true);  //Store values in m?_steps array (of int)
  _trajDigitization(ry, _mTH2_steps, vec_length, TH2._minStep, true);
  _trajDigitization(r_th, _mTH3_steps, vec_length, TH3._minStep, true);
  
  //  //SerialUSB.print("Min step: "); SerialUSB.println(TH1._minStep, 6);
  //  Serial.println("STEPS VECTORS:");
  //  int th1_tot_steps = 0, th2_tot_steps = 0, th3_tot_steps = 0;
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.print(_mTH1_steps[i]);
  //    Serial.print("\t");
  //    Serial.println(_mTH2_steps[i]);
  //    th1_tot_steps += _mTH1_steps[i];
  //    th2_tot_steps += _mTH2_steps[i];
  //    //Serial.print("\t");
  //  }
  //  Serial.println("");
    //SerialUSB.println(" ");
    
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.println(_mTH2_steps[i]);
  //    th2_tot_steps += _mTH2_steps[i];
  //    //Serial.print("\t");
  //  }
  //  Serial.println("");
  //  
  //  for (int i = 0; i <= 99; i++) {
  //    Serial.print(_mTH3_steps[i]);
  //    th3_tot_steps += _mTH3_steps[i];
  //    Serial.print("\t");
  //  }
  //  Serial.println("");
  //  Serial.println("");
  //  //    for (int i = 0; i <= 99; i++) {
  //  //      SerialUSB.print(_mTH3_steps[i]);
  //  //      th1_tot_steps += _mTH1_steps[i];
  //  //      SerialUSB.print("\t");
  //  //    }
  //  //SerialUSB.println(" ");
  //  //SerialUSB.println(" ");
  //  Serial.print("th1_tot_steps: "); Serial.println(th1_tot_steps);
  //  Serial.print("th2_tot_steps: "); Serial.println(th2_tot_steps);
  //  Serial.print("th3_tot_steps: "); Serial.println(th3_tot_steps);

  // Update XYR-trajectory time interval and initial motor timer frequency
  _ti_xyr = tf / vec_length;
  TH1._freq = round(1 / _ti_xyr);
  TH2._freq = round(1 / _ti_xyr);
  TH3._freq = round(1 / _ti_xyr);

}

/** Used only in Velocity and Manual Mode to stop the motors if the position crosses the workspace limits. Not used in Trajectory mode as the target position is pre-defined
 * Each of the 8 bits of the _velocityDirectionFlags register indicates if the robot is moving in the positive or negative direction in any of the 4 axes
 * @param vx velocity in the X direction (positive or negative)
 * @param vy velocity in the Y direction (positive or negative)
 * @param vz velocity in the Z direction (positive or negative)
 * @param vr velocity in the R direction (positive or negative)
*/
void ScaraDue::_setVelocityDirectionFlags(float vx, float vy, float vz, float vr){
  // Set/Clear bits 0-1 depending on the direction of vx
  if (vx > 0) {
    _velocityDirectionFlags |= (1<<0);  // Set bit 0
    _velocityDirectionFlags &= ~(1<<1); // Clear bit 1
    //vel_dir[0] = true;
    //vel_dir[1] = false;
  } else if (vx < 0) {
    _velocityDirectionFlags &= ~(1<<0); // Clear bit 0 
    _velocityDirectionFlags |= (1<<1);  // Set bit 1
    //vel_dir[0] = false;
    //vel_dir[1] = true;
  } else {
    _velocityDirectionFlags &= ~(1<<0);  // Clear bit 0 
    _velocityDirectionFlags &= ~(1<<1);  // Clear bit 1
    //vel_dir[0] = false;
    //vel_dir[1] = false;
  }
  // Set/Clear bits 2-3 depending on the direction of vy
  if (vy > 0) {
    _velocityDirectionFlags |= (1<<2);  // Set bit 2
    _velocityDirectionFlags &= ~(1<<3); // Clear bit 3
    //vel_dir[2] = true;
    //vel_dir[3] = false;
  } else if (vy < 0) {
    _velocityDirectionFlags &= ~(1<<2); // Clear bit 2 
    _velocityDirectionFlags |= (1<<3);  // Set bit 3
    //vel_dir[2] = false;
    //vel_dir[3] = true;
  } else {
    _velocityDirectionFlags &= ~(1<<2);  // Clear bit 2 
    _velocityDirectionFlags &= ~(1<<3);  // Clear bit 3
    //vel_dir[2] = false;
    //vel_dir[3] = false;
  }
  // Set/Clear bits 4-5 depending on the direction of vz
  if (vz > 0) {
    _velocityDirectionFlags |= (1<<4);  // Set bit 4
    _velocityDirectionFlags &= ~(1<<5); // Clear bit 5
    //vel_dir[4] = true;
    //vel_dir[5] = false;
  } else if (vz < 0) {
    _velocityDirectionFlags &= ~(1<<4); // Clear bit 4 
    _velocityDirectionFlags |= (1<<5);  // Set bit 5
    //vel_dir[4] = false;
    //vel_dir[5] = true;
  } else {
    _velocityDirectionFlags &= ~(1<<4);  // Clear bit 4 
    _velocityDirectionFlags &= ~(1<<5);  // Clear bit 5
    //vel_dir[4] = false;
    //vel_dir[5] = false;
  }
  // Set/Clear bits 6-7 depending on the direction of vr
  if (vr > 0) {
    _velocityDirectionFlags |= (1<<6);  // Set bit 6
    _velocityDirectionFlags &= ~(1<<7); // Clear bit 7
    //vel_dir[6] = true;
    //vel_dir[7] = false;
  } else if (vr < 0) {
    _velocityDirectionFlags &= ~(1<<6); // Clear bit 6 
    _velocityDirectionFlags |= (1<<7);  // Set bit 7
    //vel_dir[6] = false;
    //vel_dir[7] = true;
  } else {
    _velocityDirectionFlags &= ~(1<<6);  // Clear bit 6 
    _velocityDirectionFlags &= ~(1<<7);  // Clear bit 7
    //vel_dir[6] = false;
    //vel_dir[7] = false;
  }
}

/** Run this function in the main loop to check that the current position is within the workspace limits
 * It stops the current motion if the robot position reaches the workspace limit in any direction and also the robot is moving towards this direction
 * @param x current position X-coordinate
 * @param y current position Y-coordinate
 * @param z current position Z-coordinate
 * @param r current position R-coordinate
 * @return TRUE if the position is within the XY workspace. FALSE otherwise
*/
bool ScaraDue::checkPositionIsWithinWS(float x, float y, float z, float r) {
  //vel_flags: 0 (X pos) ; 1 (X neg) ; 2 (Y pos) ; 3 (Y neg)
  // 0 - CHECK LIMIT SWITCHES
  // Z_LimSw_Val = digitalRead(Z_LimSw_PIN);
  // R_LimSw_Val = digitalRead(R_LimSw_PIN);

  // if (!Z_LimSw_Val == HIGH && vel_dir[4]) {
  //   _stopTimerZ();
  // }

  // if (!R_LimSw_Val == HIGH && vel_dir[6]) { //TO DO: check if vel[6] OR vel[7]
  //   _stopTimerTH3();
  // }
  
  // 1 - CHECK THE XY WORKSPACE
  float radius = sqrt(sq(x) + sq(y));
  if (radius > _workSpace_XY_max_radius) {
    if (y > 0 && getBit(_velocityDirectionFlags, 2)) {
      //Serial.println("CR1");
      _stopAllTimers();
      return false;
    } else if (y < 0 && getBit(_velocityDirectionFlags, 3)) {
      //Serial.println("CR2");
      _stopAllTimers();
      return false;
    }

    if (x > 0 && getBit(_velocityDirectionFlags, 0)) {
      //Serial.println("CR3");
      _stopAllTimers();
      return false;
    } else if (x < 0 && getBit(_velocityDirectionFlags, 1)) {
      //Serial.println("CR4");
      _stopAllTimers();
      return false;
    }

  } else if (radius < _workSpace_XY_min_radius && y > 0) {
    if (getBit(_velocityDirectionFlags, 3)) {
      //Serial.println("CR5");
      _stopAllTimers();
      return false;
    }
    if (x > 0 && getBit(_velocityDirectionFlags, 1)) {
      //Serial.println("CR6");
      _stopAllTimers();
      return false;
    } else if (x < 0 && getBit(_velocityDirectionFlags, 0)) {
      //Serial.println("CR7");
      _stopAllTimers();
      return false;
    }

  } else {
    if (y < 0 && x > 0) {
      if (x < _workSpace_XY_min_radius && getBit(_velocityDirectionFlags, 1)) {
        //Serial.println("CR8");
        _stopAllTimers();
        return false;
      }
    } else if (y < 0 && x < 0) {
      if (x > -_workSpace_XY_min_radius && getBit(_velocityDirectionFlags, 0)) {
        //Serial.println("CR9");
        _stopAllTimers();
        return false;
      }
    }
  }
  // 2 - CHECK TH1 AND TH2 LIMITS

  // 3 - CHECK THE Z-AXIS
  if ((z < _workSpace_Z_min && getBit(_velocityDirectionFlags, 5)) || (z > _workSpace_Z_max && getBit(_velocityDirectionFlags, 4))) {
    _stopAllTimers();
    return false;
  }
  // 4 - CHECK R-AXIS


  return true;
}

/** Calculates if the straight line between current and target position crosses any of the inner boundaries of the workspace. This boundaries are:
 *  - Vertical line at x = _workSpace_XY_min_radius for negative y values
 *  - Vertical line at x = -_workSpace_XY_min_radius for negative y values
 *  - Semi-circle of radius = _workSpace_XY_min_radius on the positive y side
 * It doesn't check if the target position is valid. For that use _checkValidTargetPosition
 * @param trg_x target position in the X-coordinate
 * @param trg_y target position in the Y-coordinate
 * @return TRUE if the target XY position can be reached using a straight line. FALSE otherwise
*/
bool ScaraDue::_checkValidTrajectory(float trg_x, float trg_y) {
  //bool _checkValidTrajectory(float m, float b){

  // 1 - If both the current and target x are outside the range -_workSpace_XY_min_radius to _workSpace_XY_min_radius, then no need to check for intersections
  if (_curr_x > _workSpace_XY_min_radius && trg_x > _workSpace_XY_min_radius) {
    return true;
  }
  if (_curr_x < -_workSpace_XY_min_radius && trg_x < -_workSpace_XY_min_radius) {
    return true;
  }
  if (_curr_y > _workSpace_XY_min_radius && trg_y > _workSpace_XY_min_radius) {
    return true;
  }

  // 2 - Calculate Line Equation for current trajectory
  float m = (trg_y - _curr_y) / (trg_x - _curr_x);
  float b = _curr_y - m * _curr_x;
  float y_lim = sqrt(sq(_workSpace_XY_max_radius) - sq(_workSpace_XY_min_radius));
  //Serial.print("m = "); Serial.println(m);
  //Serial.print("b = "); Serial.println(b);

  // 3 - Check intersection with vertical line x = -_workSpace_XY_min_radius, from y = 0 to y = -sqrt(sq(_workSpace_XY_max_radius) - sq(_workSpace_XY_min_radius)) if the trajectory crosses x = -_workSpace_XY_min_radius
  if ((_curr_x < -_workSpace_XY_min_radius && trg_x > -_workSpace_XY_min_radius) || (_curr_x > -_workSpace_XY_min_radius && trg_x < -_workSpace_XY_min_radius)) {
    //Serial.println("Check Line 1");
    //float x_min = -_workSpace_XY_min_radius;
    float y_min = m * (-_workSpace_XY_min_radius) + b;

    //Serial.print("1st Check (x=-100): "); Serial.println(y_min);
    if (y_min > -y_lim && y_min < 0) {
      //Serial.print("Line 1 fail: y_lim = "); Serial.print(y_lim);Serial.print(" ; y_min = ");Serial.println(y_min);
      return false;
    }
  }

  // 4 - Check intersection with vertical line x = _workSpace_XY_min_radius, from y = 0 to y = -sqrt(sq(_workSpace_XY_max_radius) - sq(_workSpace_XY_min_radius))  if the trajectory crosses x = _workSpace_XY_min_radius
  if ((_curr_x < _workSpace_XY_min_radius && trg_x > _workSpace_XY_min_radius) || (_curr_x > _workSpace_XY_min_radius && trg_x < _workSpace_XY_min_radius)) {
    //Serial.println("Check Line 2");
    //float x_max = _workSpace_XY_min_radius;
    float y_max = m * _workSpace_XY_min_radius + b;
    //Serial.print("2on Check (x=100): "); Serial.println(y_max);
    if (y_max > -y_lim && y_max < 0) {
      //Serial.print("Line 2 fail: y_lim = "); Serial.print(y_lim);Serial.print(" ; y_max = ");Serial.println(y_max);
      return false;
    }
  }

  // 5 - Check intersection with circle x^2 + y^2 = _workSpace_XY_min_radius^2
  // * If r^2 * (1 + m^2) - b^2 is larger than 0, there is an intersection
  float criteria = sq(_workSpace_XY_min_radius) * (sq(1) + sq(m)) - sq(b);
  //Serial.print("3rd Check (criteria): "); Serial.println(criteria);
  if (criteria <= 0) {
    //return true;
  }  // Return true if there's no intersection
  // * If there is intersection, check if it happens on a positive y value
  else {
    //Serial.println("Inner Circle Check");
    float y1 = (b + m * sqrt(criteria)) / (1 + sq(m));
    float y2 = (b - m * sqrt(criteria)) / (1 + sq(m));
    //Serial.print("3rd Check (y1): "); Serial.println(y1);
    //Serial.print("3rd Check (y2): "); Serial.println(y2);
    if ((y1 > 0 && y1 > _curr_y && y1 < trg_y) || (y1 > 0 && y1 < _curr_y && y1 > trg_y)) {
      //Serial.print("Inner Circle fail: y_1 = "); Serial.println(y1);
      return false;
    }
    if ((y2 > 0 && y2 > _curr_y && y2 < trg_y) || (y2 > 0 && y2 < _curr_y && y2 > trg_y)) {
      //Serial.print("Inner Circle fail: y_2 = ");Serial.println(y2);
      return false;
    }

  }

  return true;

}

// ------------------------------------ NOT CHECKED ---------------------------------- //











#pragma endregion

#pragma region Timers_Control_Routines

// ----------------------------------- Timer Start / Update / Stop Generic Functions ------------------------------------ //
void ScaraDue::_timerStart(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq) {
  //Serial.print("F : "); Serial.println(freq);
  // Tell the Power Management Controller to disable the write protection of the (Timer/Counter) registers:
  pmc_set_writeprotect(false);
  // Enable clock for the timer
  pmc_enable_periph_clk(irq);

  // Set up the Timer in waveform mode which creates a PWM in UP mode with automatic trigger on RC Compare
  // and sets it up with the determined internal clock as clock input.
  //_prescalerS:
  // - TIMER_CLOCK1 - MCK/2   - 42MHz
  // - TIMER_CLOCK2 - MCK/8   - 10.5MHz
  // - TIMER_CLOCK3 - MCK/32  - 2.652MHz
  // - TIMER_CLOCK4 - MCK/128 - 656.25KHz
  // - TIMER_CLOCK5 - SLCK    - 32KHz
  // *MCK - Master Clock (84MHz)
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
  //_freq --> desired blinking frequency (in Hz)
  uint32_t rc = VARIANT_MCK / _prescaler / freq;
  //Serial.print("RC : "); Serial.println(rc);
  //Creates a wafeform that goes high at RA and low at RC
  //TC_SetRA(tc, channel, rc >> 1); // 50% duty cycle square wave
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  //interrups occurs only when counter reaches RC:
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;

  NVIC_EnableIRQ(irq);
}
void ScaraDue::_timerUpdate(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq) {
  uint32_t rc = VARIANT_MCK / _prescaler / freq;
  //Serial.println(freq);
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  NVIC_EnableIRQ(irq);
}
void ScaraDue::_timerStop(Tc *tc, uint32_t channel, IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}

// --------------------------------- Stepper Motor Timers Setup --------------------------------------------------------- //

// Initialise Timer for Z Axis Motion
void ScaraDue::_startZTimer() {  //Start the timer at the right frequency and set Z-timer state to true
  Z._currentSteps = 0;
  Z._i = 0;
  if (!Z._isTimerOn) {
    Serial.print(" * Z Timer Started @ freq ");Serial.println(Z._freq);
    _timerStart(TC1, 0, TC3_IRQn, Z._freq);
    Z._isTimerOn = true;
  }
  
}
void ScaraDue::_startTh1Timer() {  //Start the timer at the right _frequency and set Z-timer state to true
  TH1._currentSteps = 0;
  TH1._i = 0;
  if (!TH1._isTimerOn) {
   Serial.print(" * TH1 Timer Started @ freq ");Serial.println(TH1._freq);
    _timerStart(TC1, 1, TC4_IRQn, TH1._freq);
    TH1._isTimerOn = true;
  }

}
void ScaraDue::_startTh2Timer() {  //Start the timer at the right frequency and set Z-timer state to true
  TH2._currentSteps = 0;
  TH2._i = 0;
  if (!TH2._isTimerOn) {
    Serial.println(" * TH2 Timer Started");
    _timerStart(TC1, 2, TC5_IRQn, TH2._freq);
    TH2._isTimerOn = true;
  }
}
void ScaraDue::_startTh3Timer() {  //Start the timer at the right frequency and set Z-timer state to true
  TH3._currentSteps = 0;
  TH3._i = 0;
  if (!TH3._isTimerOn) {
    Serial.println(" * TH3 Timer Started");
    _timerStart(TC2, 0, TC6_IRQn, TH3._freq);
    TH3._isTimerOn = true;
  }
}

// Initialise Timers for motion on the XYR Plane
void ScaraDue::_startXYRTimers() {
  _startTh1Timer();
  _startTh2Timer();
  _startTh3Timer();

  // if (!TH1._isTimerOn) {
  //   _timerStart(TC1, 1, TC4_IRQn, TH1._freq);
  //   TH1._isTimerOn = true;
  // }
  // if (!TH2._isTimerOn) {
  //   _timerStart(TC1, 2, TC5_IRQn, TH2._freq);
  //   TH2._isTimerOn = true;
  // }
  // if (!TH3._isTimerOn) {
  //   _timerStart(TC2, 0, TC6_IRQn, TH3._freq);
  //   TH3._isTimerOn = true;
  // }
}

void ScaraDue::_initialiseAllTimers() {
  //SetupMotorsBeforeStart();
  _startZTimer();
  _startXYRTimers();
}


// Initialise Timer for R Axis Motion
// void StartRTimer() {  //Start the timer at the right frequency and set Z-timer state to true
//   if (!TH3.isTimerOn) {
//     _timerStart(TC2, 0, TC6_IRQn, TH3._freq);
//     TH3.isTimerOn = true;
//   }
// }

// void ScaraDue::SetupMotorsBeforeStart() {
//   // Setup Z Axis
//   Z._currentSteps = 0;
//   Z._i = 0;
//   //Z._freq = freq_init;
//   // Setup XYR plane
//   TH1._currentSteps = 0;
//   TH1._i = 0;

//   TH2._currentSteps = 0;
//   TH2._i = 0;

//   TH3._currentSteps = 0;
//   TH3._i = 0;
// }

// Start Timer7. Used in Speed Control Mode to coordinate XYR Timers
// void Start_VM_XY_Timer() {
//   Timer7_On = true;
//   _timerStart(TC2, 1, TC7_IRQn, T7_freq);
// }

void ScaraDue::_stopAllTimers() {

  _stopTimerZ();
  _stopTimerTH1();
  _stopTimerTH2();
  _stopTimerTH3();

  // if (!Z.isTimerOn && !TH1.isTimerOn && !TH2.isTimerOn && !TH3.isTimerOn && Timer7_On) {
  //   Timer7_On = false;
  //   _timerStop(TC2, 1, TC7_IRQn);
  // }


}
void ScaraDue::_stopTimerZ() {

  if (Z._isTimerOn) {
    if (Z._pulseStatus == HIGH) { // Always stop motion with pulse status = LOW
      _step_Z();
    }
    Z._isTimerOn = false;
    _timerStop(TC1, 0, TC3_IRQn);
    Serial.println(" * Z Timer Stopped");
  }

}
void ScaraDue::_stopTimerTH1() {

  if (TH1._isTimerOn) {
    if (TH1._pulseStatus == HIGH) {
      _step_TH1();
    }
    TH1._isTimerOn = false;
    TH1._isMoving = false;
    _timerStop(TC1, 1, TC4_IRQn);
    Serial.println(" * TH1 Timer Stopped");
  }
}
void ScaraDue::_stopTimerTH2() {
  if (TH2._isTimerOn) {
    if (TH2._pulseStatus == HIGH) {
      _step_TH2();
    }
    TH2._isTimerOn = false;
    TH2._isMoving = false;
    _timerStop(TC1, 2, TC5_IRQn);
    Serial.println(" * TH2 Timer Stopped");
  }
}
void ScaraDue::_stopTimerTH3() {

  if (TH3._isTimerOn) {
    if (TH3._pulseStatus == HIGH) {
      _step_TH3();
    }
    TH3._isTimerOn = false;
    _timerStop(TC2, 0, TC6_IRQn);
    Serial.println(" * TH3 Timer Stopped");
  }
}

#pragma endregion

#pragma region Timers Interrupt_Service_Routines

void ScaraDue::MotorRun_Z(){
  // if (system_paused) {
  //   return;
  // }  
  switch (controlMode) {
    case STP:  // If the robot is in STOP mode, disable timer
      Z._isTimerOn = false;
      _timerStop(TC1, 0, TC3_IRQn);
      break;
    case MAN:  // Manual Mode Operation
      if (Z._targetSteps == 0 || Z._currentSteps >= Z._targetSteps) {
        _stopTimerZ(); // If motion is completed, stop the timer
      } else {
        _step_Z();   // Perform Step
        if (Z._pulseStatus == HIGH) {       // if PULSE pin is HIGH
          Z._currentSteps++;                        // count 1 step
          Z._posSteps += Z._dir;  // and update the current step position
          // if (_isHomed) {
          //   Z._pos += Z._minStep * Z._dir;  // If the robot is homed, update the Cartesian Position
          // }
        }
      }
      break;
    case HOM: //Homing Mode
      // if (Z_LimSw_Val == HIGH) {
      //   _step_Z();
      //   Z_LimSw_Val = digitalRead(Z_LimSw_PIN);
      // } else {
      //   Z_LimSw_Val = digitalRead(Z_LimSw_PIN);
      //   if (!Z_LimSw_Val == HIGH) {
      //     _isHomed_Z = true;
      //     _stopTimerZ();
      //   }
      // }
      break;
    case TRAJ:
      if (_isHomed == false) {   // Exit timer if robot not homed
        Z._isTimerOn = false;
        _timerStop(TC1, 0, TC3_IRQn);
      }
      if (_mZ_steps[Z._i] != 0) { // If the steps to perform during the current time interval are not zero
        Z._freq = (abs(_mZ_steps[Z._i]) * 2) / _ti_z; // Calculate _frequency for next interrupt
        _timerUpdate(TC1, 0, TC3_IRQn, Z._freq);  // Setup the time for the next interrup
        _step_Z(); // Perform step

        if (Z._pulseStatus == HIGH) { // If pulse state is HIGH count a step and update the motor position
          Z._currentSteps++;
          Z._posSteps += Z._dir;
          //Z._pos += Z._minStep * Z._dir;
        } else { // When the motor has completed a full step (LOW to HIGH + HIGH to LOW), then update the remaining steps for this time interval
          if (Z._remSteps > 0) {
            Z._remSteps -= 1;
          } else if (Z._remSteps < 0) {
            Z._remSteps += 1;
          }
        }
        if (Z._remSteps == 0) {                   //If remaining steps is equal to 0
          Z._i++;                                  //Increase index
          Z._remSteps = _mZ_steps[Z._i];  //and look for the next value in the look-up vector
          //Set the stepper motor direction:
          if (Z._remSteps > 0) {
            _setDirection_Z(true);
          } else if (Z._remSteps < 0) {
            _setDirection_Z(false);
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (_mZ_steps[Z._i] == 0) {
        Z._freq = freq_init;
        _timerUpdate(TC1, 0, TC3_IRQn, Z._freq);  //Set the timer to wait for another whole time interval
        Z._i++;                                  //Increase index
        Z._remSteps = _mZ_steps[Z._i];  //and look for the next value in the look-up vector
        //Set the stepper motor direction:
        if (Z._remSteps > 0) {
          _setDirection_Z(true);
        } else if (Z._remSteps < 0) {
          _setDirection_Z(false);
        }
      }
      // If the end of the look-up vector is reached, stop the timer. If motion needed in the XYR plane, then initialise this timers
      if (Z._i >= vec_length) {
        if (_isMovingXYR) {
          _isMovingXYR = false;
          _startXYRTimers();
        }
        _stopTimerZ();
      }
      break;
    case VEL:
      if (_isHomed == false)   // Exit timer if robot not homed
        _stopTimerZ();
      // ---- CHECK CONTROLLED DESCENT MODE RESULT ---- //      
      // When it gets to the minimum Z, check current airPressure. It should be above the threshold in either mode to be considered a success
      if (_curr_z <= _lowest_z && _descentMode != 0 ) {
        if (_airPressure > _refAirPressure)
          _descendMode_result = 1; // Return SUCCESS code
        else
          _descendMode_result = 2; // Return FAIL code
        _stopTimerZ();
      }
      // When in pickup mode need to monitor that current pressure is not above threshold. If that happens, then stop (Pickup successful)
      if (_descentMode == 1) {
        if (_airPressure > _refAirPressure) { // Object has been picked up. Stop descending
          _descendMode_result = 1;
          _stopTimerZ();
        }
        // When in jig load mode need to monitor that current pressure is not below threshold. If that happens, then stop (Jig load fail)
      } else if (_descentMode == 2) {
        if (_airPressure < _refAirPressure) { // Object has dropped. Stop descending
          _descendMode_result = 2;
          _stopTimerZ();
        }
      }
      if (_stopVelMode) {  // Start deceleration phase
        if(Z._currentSteps < Z._targetSteps){
          if (Z._pulseStatus == LOW) {
            int num = 2 * _prev_counter_value + _rest;
            int den = 4 * (Z._currentSteps - Z._targetSteps) + 1;
            unsigned int compare_value = _prev_counter_value - (num / den);
            int _freq = (VARIANT_MCK / _prescaler) / compare_value;
            Z._freq = _freq;
            if (Z._currentSteps != (Z._targetSteps - 1))
              _rest = num % den;
            else
              _rest = 0;
            _prev_counter_value = compare_value;
          }          
        }
      }else if (Z._currentSteps < _accSteps) { // Acceleration phase
        if (Z._pulseStatus == LOW) {
          if (Z._currentSteps == 0) {
            int _freq = (VARIANT_MCK / _prescaler) / _c0;
            Z._freq = _freq;
            _prev_counter_value = _c0;
            _rest = 0;
          } else {
            int compare_value = _prev_counter_value - ((2 * _prev_counter_value + _rest) / (4 * Z._currentSteps + 1));
            int _freq = (VARIANT_MCK / _prescaler) / compare_value;
            Z._freq = _freq;
            _rest = (2 * _prev_counter_value + _rest) % (4 * Z._currentSteps + 1);
            _prev_counter_value = compare_value;

          }
        }
      }
      _timerUpdate(TC1, 0, TC3_IRQn, Z._freq);
      _step_Z(); // Perform step
      if (Z._pulseStatus == HIGH) { // If pulse pin is HIGH count a step and update the motor position
        Z._posSteps += Z._dir;
        //Z._pos += Z._minStep * Z._dir;
        Z._currentSteps++;
        if(Z._currentSteps >= Z._targetSteps){
          if (_descentMode != 0) {
            if (_airPressure > _refAirPressure)
              _descendMode_result = 1;
            else
              _descendMode_result = 2;
          }
          _stopTimerZ();
        }else if (Z._currentSteps >= _decStart) {
          _stopVelMode = true;
        }
      }
      // ---------------------------------------------- //
      break;
    // case VEL:
    //   if (_isHomed == false) {   // Exit timer if robot not homed
    //     //Serial.println("H");
    //     _stopTimerZ();
    //   }

    //   if (!_joystickControl) {
    //     if (_stopVelMode) {  // Start deceleration phase
    //       if (DescendMode_CurrentStepCount < DescendMode_TotalSteps) {
    //         if (Z._pulseStatus == LOW) {
    //           //int cnt = DescendMode_TotalSteps - DescendMode_CurrentStepCount;
    //           int num = 2 * _prev_counter_value + _rest;
    //           int den = 4 * (DescendMode_CurrentStepCount - DescendMode_TotalSteps) + 1;
    //           unsigned int compare_value = _prev_counter_value - (num / den);
    //           int _freq = (VARIANT_MCK / _prescaler) / compare_value;
    //           Z._freq = _freq;
    //           if (DescendMode_CurrentStepCount != (DescendMode_TotalSteps - 1)) {
    //             _rest = num % den;
    //           }
    //           else {
    //             _rest = 0;
    //           }
    //           _prev_counter_value = compare_value;
    //         }
    //       }
    //     } else if (DescendMode_CurrentStepCount < _accSteps) { // Start acceleration phase
    //       if (Z._pulseStatus == LOW) {
    //         if (DescendMode_CurrentStepCount == 0) {
    //           int _freq = (VARIANT_MCK / _prescaler) / _c0;
    //           Z._freq = _freq;
    //           _prev_counter_value = _c0;
    //           _rest = 0;
    //         } else {
    //           int compare_value = _prev_counter_value - ((2 * _prev_counter_value + _rest) / (4 * DescendMode_CurrentStepCount + 1));
    //           int _freq = (VARIANT_MCK / _prescaler) / compare_value;
    //           Z._freq = _freq;
    //           _rest = (2 * _prev_counter_value + _rest) % (4 * DescendMode_CurrentStepCount + 1);
    //           _prev_counter_value = compare_value;

    //         }
    //       }
    //     }
    //     _timerUpdate(TC1, 0, TC3_IRQn, Z._freq);
    //   }
    //   _step_Z(); // Perform step
    //   if (Z._pulseStatus == HIGH) { // If pulse pin is HIGH count a step and update the motor position
    //     Z._posSteps += Z._dir;
    //     Z._pos += Z._minStep * Z._dir;
    //     Z._currentSteps++;
    //     DescendMode_CurrentStepCount++;

    //     if (!_joystickControl) {
    //       if (DescendMode_CurrentStepCount >= DescendMode_TotalSteps) {
    //         if (_descentMode != 0) {
    //           if (airPressure > refAirPressure) {
    //             descendModeResult = 1;
    //           }
    //           else {
    //             descendModeResult = 2;
    //           }
    //         }
    //         _stopTimerZ();
    //       }
    //       else if (DescendMode_CurrentStepCount >= DescendMode_DecStart) {
    //         _stopVelMode = true;
    //       }
    //     }
    //   }
    //   break;
  }
}

void ScaraDue::MotorRun_TH1(){
  // if (system_paused) {
  //   return;
  // }

  switch (controlMode) {
    case STP:   // If the robot is in STOP mode, disable timer
      TH1._isTimerOn = false;
      _timerStop(TC1, 1, TC4_IRQn);
      break;
    case MAN:   // Manual Mode Operation
      if (TH1._targetSteps == 0 || TH1._currentSteps >= TH1._targetSteps) {
        _stopTimerTH1(); // If motion is completed, stop the timer
      } else {
        _step_TH1();
        if (TH1._pulseStatus == HIGH) {       // if PULSE pin is HIGH
          TH1._currentSteps++;                        // count 1 step
          TH1._posSteps += TH1._dir;  // and update the current step position
          TH1._isMoving = true;
          // if (_isHomed) {
          //   TH1._pos += TH1._minStep * TH1._dir;  // If the robot is homed, update the Angular Position
          // }
        }
      }

      break;
    case TRAJ:  // Trajectory Mode Operation
      if (_isHomed == false) {   // Exit timer if robot not homed
        TH1._isTimerOn = false;
        _timerStop(TC1, 1, TC4_IRQn);
      }
      if (_mTH1_steps[TH1._i] != 0) { // If the steps to perform during the current time interval are not zero
        TH1._freq = (abs(_mTH1_steps[TH1._i]) * 2) / _ti_xyr; // Calculate _frequency for next interrupt
        _timerUpdate(TC1, 1, TC4_IRQn, TH1._freq);  // Setup the time for the next interrup
        _step_TH1();   // Perform Step

        if (TH1._pulseStatus == HIGH) { // If pulse state is HIGH count a step and update the motor position
          TH1._currentSteps++;
          TH1._posSteps += TH1._dir;
          //TH1._pos += TH1._minStep * TH1._dir;
          TH1._isMoving = true;
        } else { // When the motor has completed a full step (LOW to HIGH + HIGH to LOW), then update the remaining steps for this time interval
          if (TH1._remSteps > 0) {
            TH1._remSteps -= 1;
          } else if (TH1._remSteps < 0) {
            TH1._remSteps += 1;
          }
        }
        if (TH1._remSteps == 0) {                     //If remaining steps is equal to 0
          TH1._i++;                                    //Increase index
          TH1._remSteps = _mTH1_steps[TH1._i];  //and look for the next value in the look-up vector
          //Set the stepper motor direction:
          if (TH1._remSteps > 0) {
            _setDirection_TH1(true);
            //TH1.toggleFunc = TH1_toggle;
          } else if (TH1._remSteps < 0) {
            _setDirection_TH1(false);
            //TH1.toggleFunc = TH1_CCW_toggle;
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (_mTH1_steps[TH1._i] == 0) {
        TH1._freq = freq_init;
        _timerUpdate(TC1, 1, TC4_IRQn, TH1._freq);    //Set the timer to wait for another whole time interval
        TH1._i++;                                    //Increase index
        TH1._remSteps = _mTH1_steps[TH1._i];  //and look for the next value in the look-up vector
        TH1._isMoving = false;
        //Set the stepper motor direction:
        if (TH1._remSteps > 0) {
          _setDirection_TH1(true);
          //TH1.toggleFunc = TH1_toggle;
        } else if (TH1._remSteps < 0) {
          _setDirection_TH1(false);
          //TH1.toggleFunc = TH1_CCW_toggle;
        }
      }
      // If the end of the look-up vector is reached, stop the timer. If motion needed in the Z axis, then initialise this timer
      if (TH1._i >= vec_length) {
        if (_isMovingZ) {
          _isMovingZ = false;
          _startZTimer();
        }
        _stopTimerTH1();
      }
      break;
    case VEL:   // Speed Mode Operation
      _step_TH1();   // Perform Step
      if (TH1._pulseStatus == HIGH) { // Update motor positions when step pin is HIGH
        TH1._posSteps += TH1._dir;
        //TH1._pos += TH1._minStep * TH1._dir;
        TH1._isMoving = true;
      }
      break;
  }
}

void ScaraDue::MotorRun_TH2(){
  // if (system_paused) {
  //   return;
  // }

  //Serial.println("IN2");
  switch (controlMode) {
    case STP:   // If the robot is in STOP mode, disable timer
      TH2._isTimerOn = false;
      _timerStop(TC1, 2, TC5_IRQn);
      break;
    case MAN:   // Manual Mode Operation
      if (TH2._targetSteps == 0 || TH2._currentSteps >= TH2._targetSteps) {
        _stopTimerTH2(); // If motion is completed, stop the timer
      } else {
        _step_TH2();
        if (TH2._pulseStatus == HIGH) {       // if PULSE pin is HIGH
          TH2._currentSteps++;                        // count 1 step
          TH2._posSteps += TH2._dir;  // and update the current step position
          TH2._isMoving = true;
          // if (_isHomed) {
          //   TH2._pos += TH2._minStep * TH2._dir;  // If the robot is homed, update the Angular Position
          // }
        }
      }
      break;



    case TRAJ:  // Trajectory Mode Operation
      if (_isHomed == false) {   // Exit timer if robot not homed
        TH2._isTimerOn = false;
        _timerStop(TC1, 2, TC5_IRQn);
      }
      if (_mTH2_steps[TH2._i] != 0) { // If the steps to perform during the current time interval are not zero
        TH2._freq = (abs(_mTH2_steps[TH2._i]) * 2) / _ti_xyr; // Calculate frequency for next interrupt
        _timerUpdate(TC1, 2, TC5_IRQn, TH2._freq);  // Setup the time for the next interrup
        _step_TH2(); // Perform Step

        if (TH2._pulseStatus == HIGH) { // If pulse state is HIGH count a step and update the motor position
          TH2._currentSteps++;
          TH2._posSteps += TH2._dir;
          //TH2._pos += TH2._minStep * TH2._dir;
        } else { // When the motor has completed a full step (LOW to HIGH + HIGH to LOW), then update the remaining steps for this time interval
          if (TH2._remSteps > 0) {
            TH2._remSteps -= 1;
          } else if (TH2._remSteps < 0) {
            TH2._remSteps += 1;
          }
        }
        if (TH2._remSteps == 0) {                     //If remaining steps is equal to 0
          TH2._i++;                                    //Increase index
          TH2._remSteps = _mTH2_steps[TH2._i];  //and look for the next value in the steps array
          //Set the stepper motor direction:
          if (TH2._remSteps > 0) {
            _setDirection_TH2(true);
            //TH2.toggleFunc = TH2_CCW_toggle;
          } else if (TH2._remSteps < 0) {
            _setDirection_TH2(false);
            //TH2.toggleFunc = TH2_toggle;
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (_mTH2_steps[TH2._i] == 0) {
        TH2._freq = freq_init;
        _timerUpdate(TC1, 2, TC5_IRQn, TH2._freq);    //Set the timer to wait for another whole time interval
        TH2._i++;                                    //Increase index
        TH2._remSteps = _mTH2_steps[TH2._i];  //and look for the next value in the steps array
        //Set the stepper motor direction:
        if (TH2._remSteps > 0) {
          _setDirection_TH2(true);
          //TH2.toggleFunc = TH2_CCW_toggle;
        } else if (TH2._remSteps < 0) {
          _setDirection_TH2(false);
          //TH2.toggleFunc = TH2_toggle;
        }
      }
      // If the end of the look-up vector is reached, stop the timer.
      if (TH2._i >= vec_length) {
        _stopTimerTH2();
      }
      break;
    case VEL:   // Speed Mode Operation
      _step_TH2();   // Perform Step
      if (TH2._pulseStatus == HIGH) { // Update motor positions when step pin is HIGH
        TH2._posSteps += TH2._dir;
        //TH2._pos += TH2._minStep * TH2._dir;
        TH2._isMoving = true;
      }
      break;
  }  
}

void ScaraDue::MotorRun_TH3(){
  // if (system_paused) {
  //   return;
  // }

  switch (controlMode) {
    case STP:   // If the robot is in STOP mode, disable timer
      TH3._isTimerOn = false;
      _timerStop(TC2, 0, TC6_IRQn);
      break;
    case MAN:   // Manual Mode Operation
      if (TH3._targetSteps == 0 || TH3._currentSteps >= TH3._targetSteps) {
        _stopTimerTH3(); // If motion is completed, stop the timer
      } else {
        _step_TH3();
        if (TH3._pulseStatus == HIGH) {       // if PULSE pin is HIGH
          TH3._currentSteps++;                        // count 1 step
          TH3._posSteps += TH3._dir;  // and update the current step position
          // if (_isHomed) {
          //   TH3._pos += TH3._minStep * TH3._dir;  // If the robot is homed, update the Angular Position
          // }
        }
      }
      break;
    case HOM: //Homing Mode
      // if (R_LimSw_Val == HIGH) {
      //   _step_TH3();
      //   R_LimSw_Val = digitalRead(R_LimSw_PIN);
      // } else {
      //   R_LimSw_Val = digitalRead(R_LimSw_PIN);
      //   if (!R_LimSw_Val == HIGH) {
      //     _isHomed_R = true;
      //     _stopTimerTH3();
      //   }
      // }
      break;
    case TRAJ:  // Trajectory Mode Operation
      if (_isHomed == false) {   // Exit timer if robot not homed
        TH3._isTimerOn = false;
        _timerStop(TC2, 0, TC6_IRQn);
      }
      if (_mTH3_steps[TH3._i] != 0) { // If the steps to perform during the current time interval are not zero
        TH3._freq = (abs(_mTH3_steps[TH3._i]) * 2) / _ti_xyr; // Calculate frequency for next interrupt
        _timerUpdate(TC2, 0, TC6_IRQn, TH3._freq);  // Setup the time for the next interrup
        _step_TH3(); // Perform Step

        if (TH3._pulseStatus == HIGH) { // If pulse state is HIGH count a step and update the motor position
          TH3._currentSteps++;
          TH3._posSteps += TH3._dir;
          //TH3._pos += TH3._minStep * TH3._dir;
        } else { // When the motor has completed a full step (LOW to HIGH + HIGH to LOW), then update the remaining steps for this time interval
          if (TH3._remSteps > 0) {
            TH3._remSteps -= 1;
          } else if (TH3._remSteps < 0) {
            TH3._remSteps += 1;
          }
        }
        if (TH3._remSteps == 0) {                     //If remaining steps is equal to 0
          TH3._i++;                                    //Increase index
          TH3._remSteps = _mTH3_steps[TH3._i];  //and look for the next value in the steps array
          //Set the stepper motor direction:
          if (TH3._remSteps > 0) {
            _setDirection_TH3(true);
          } else if (TH3._remSteps < 0) {
            _setDirection_TH3(false);
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (_mTH3_steps[TH3._i] == 0) {
        TH3._freq = freq_init;
        _timerUpdate(TC2, 0, TC6_IRQn, TH3._freq);    //Set the timer to wait for another whole time interval
        TH3._i++;                                    //Increase index
        TH3._remSteps = _mTH3_steps[TH3._i];  //and look for the next value in the steps array
        //Set the stepper motor direction:
        if (TH3._remSteps > 0) {
          _setDirection_TH3(true);
        } else if (TH3._remSteps < 0) {
          _setDirection_TH3(false);
        }
      }
      // If the end of the look-up vector is reached, stop the timer. If motion needed in the Z axis, then initialise this timer
      if (TH3._i >= vec_length) {
        _stopTimerTH3();
      }
      break;
    case VEL:   // Speed Mode Operation
      _step_TH3();   // Perform Step
      if (TH3._pulseStatus == HIGH) { // Update motor positions when step pin is HIGH
        TH3._posSteps += TH3._dir;
        //TH3._pos += TH3._minStep * TH3._dir;
      }
      break;
  }  
}

void TC3_Handler()  //Z - VERTICAL AXIS
{
  TC_GetStatus(TC1, 0);
  //Serial.println("z");
  SCARA.MotorRun_Z();
}

void TC4_Handler()  //TH1 - SHOULDER JOINT
{
  TC_GetStatus(TC1, 1);  //Don't know what it does but it's important!! Without this you can't set the time properly
  SCARA.MotorRun_TH1();
}

void TC5_Handler()  //TH1 - SHOULDER JOINT
{
  TC_GetStatus(TC1, 2);  //Don't know what it does but it's important!! Without this you can't set the time properly
  SCARA.MotorRun_TH2();
}

void TC6_Handler()  //TH1 - SHOULDER JOINT
{
  TC_GetStatus(TC0, 0);  //Don't know what it does but it's important!! Without this you can't set the time properly
  SCARA.MotorRun_TH3();
}

#pragma endregion
