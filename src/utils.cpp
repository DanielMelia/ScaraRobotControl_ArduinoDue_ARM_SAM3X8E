#include "utils.h"

/** NOTES AND TODOS
 * All large arrays have been changed from double to float. In DUE float is 4-bytes (6-7 decimal places precision), while double is 8-bytes (~15 decimal places precision)
 * https://www.tutorialspoint.com/difference-between-float-and-double-in-arduino#:~:text=Arduino%20Due%20is%20an%20exception,values%20are%20equivalent%20to%20float.
 * _xyToAngularTrajectories : replace local variable L by byref parameters L1 and L2
*/

void Initialise_PORT_CLOCK(Pio* GPIO_x){
  // SAM3X Peripheral Identifiers (Datasheet table 9.1. , page 38)
  // PIOA -> 11
  // PIOB -> 12
  // PIOC -> 13
  // PIOD -> 14
  // PIOE -> 15
  // PIOF -> 16
  // USART0 -> 17
  // USART1 -> 18
  // USART2 -> 19
  // USART3 -> 20  
  if(GPIO_x == PIOA){
    PMC->PMC_PCER0 |= (1 << 11);
  }else if(GPIO_x == PIOB){
    PMC->PMC_PCER0 |= (1 << 12);
  }else if(GPIO_x == PIOC){
    PMC->PMC_PCER0 |= (1 << 13);
  }else if(GPIO_x == PIOD){
    PMC->PMC_PCER0 |= (1 << 14);
  }


}

/// @brief Configure a GPIO pin as an Input/Output and configure the pull-up resistors
/// @param GPIO_x GPIO port
/// @param pin_number GPIO pin number
/// @param mode pin mode: Output, Input or Input_PullUp
void GPIO_Pin_Setup(Pio* GPIO_x, uint32_t pin_number, PinMode mode){
  if(mode == Output){
    GPIO_x->PIO_PER |= (1<<pin_number);  // Enable the GPIO pin
    GPIO_x->PIO_OER |= (1<<pin_number);  // Enable output mode for the GPIO pin (set it as output)
  }else if(mode == Input){
    GPIO_x->PIO_PER |= (1<<pin_number);  // Enable the GPIO pin
    GPIO_x->PIO_ODR |= (1<<pin_number);  // Disable output mode for the GPIO pin (set it as input)
    GPIO_x->PIO_PUDR |= (1<<pin_number); // Disable pull-up resistor for the GPIO pin
  }else if(mode == Input_PullUp){
    //PMC->PMC_PCER0 |= ((1 << 14) | (1 << 15) | (1 << 16) | (1 << 17));
    
    GPIO_x->PIO_PER |= (1<<pin_number);  // Enable the GPIO pin
    GPIO_x->PIO_ODR |= (1<<pin_number);  // Disable output mode for the GPIO pin (set it as input)
    GPIO_x->PIO_PUER |= (1<<pin_number); // Enable pull-up resistor for the GPIO pin
  }
}

/// @brief Read GPIO pin state
/// @param GPIO_x  GPIO port
/// @param pin_number GPIO pin number
/// @return TRUE if pin is set HIGH. FALSE otherwise
bool GPIO_Pin_Read(Pio* GPIO_x, uint32_t pin_number){
  // Read the Pin Data Status Register
  return (GPIO_x->PIO_PDSR & (1 << pin_number)) != 0; 
  // Inequality comparison checks if the result of the bitwise and is not zero. If result is NOT ZERO, means pin is HIGH, and will return TRUE
  // (PIOA_PDSR & (1 << pin)) ? HIGH : LOW // this uses ternary (or conditional) operator, and returs a uint8_t
}

/// @brief Writes HIGH/LOW to a GPIO pin
/// @param GPIO_x  GPIO port
/// @param pin_number GPIO pin number
/// @param value TRUE sets the pin to HIGH. FALSE clears the pin
void GPIO_Pin_Write(Pio* GPIO_x, uint32_t pin_number, bool value){
  if(value)
    GPIO_x->PIO_SODR |= (1 << pin_number); // Set output high
  else
    GPIO_x->PIO_CODR |= (1 << pin_number); // Set output low
}

bool getBit(uint8_t byte, uint8_t position) // position in range 0-7
{
    return (byte >> position) & 0x1;
}

// //Return the quotient and reminder of a division
// void quotient_remainder(float num, uint16_t &quo, float &rem) {
//   quo = (uint16_t)num;
//   rem = num - (uint16_t)num;
// }

/** Calculates 3rd order polynomial path from current to target position in the XY plane
 * @param rx pointer to array of X-coordinate of the cartesian path
 * @param ry pointer to array of Y-coordinate of the cartesian path
 * @param curr_x current position in the x coordinate
 * @param curr_y current position in the y coordinate
 * @param trg_x target position in the x coordinate
 * @param trg_y target position in the y coordinate
 * @param tf total path time in seconds
 * @param vec_length number of elements of the trajectory array 
*/
void _xyCartesianPath(float *rx, float *ry, float curr_x, float curr_y, float trg_x, float trg_y, float tf, uint16_t vec_length) {
  //Calculate 3rd order polynomial coefficients
  //NOTE: assumes t0=0, and v0=vf=0
  float a0x = curr_x;
  float a0y = curr_y;
  //float a1 = 0;
  float a2x = -3 * (curr_x - trg_x) / pow(tf, 2);
  float a2y = -3 * (curr_y - trg_y) / pow(tf, 2);
  float a3x = 2 * (curr_x - trg_x) / pow(tf, 3);
  float a3y = 2 * (curr_y - trg_y) / pow(tf, 3);
  //Compute trajectory and store it in r1 array
  for (uint16_t i = 0; i < vec_length; i++) {
    float t = tf * i / vec_length;
    rx[i] = a0x + a2x * t * t + a3x * t * t * t;
    ry[i] = a0y + a2y * t * t + a3y * t * t * t;
  }
}

/** Calculates 3rd order polynomial path from current to target position in the Z axis
 * @param r pointer to array of Z-coordinate of the cartesian path
 * @param curr current position in the Z coordinate
 * @param trg target position in the Z coordinate
 * @param tf total path time in seconds
 * @param vec_length number of elements of the trajectory array 
*/
void _zCartesianPath(float *r, float curr, float trg, float tf, uint16_t vec_length) {
  //Calculate 3rd order polynomial coefficients
  //NOTE: assumes t0=0, and v0=vf=0
  float a0 = curr;
  //float a1 = 0;
  float a2 = -3 * (curr - trg) / pow(tf, 2);
  float a3 = 2 * (curr - trg) / pow(tf, 3);
  //Compute trajectory and store it in r1 array
  for (uint16_t i = 0; i < vec_length; i++) {
    float t = tf * i / vec_length;
    r[i] = a0 + a2 * t * t + a3 * t * t * t;
  }
}

/** Computes the angular trajectory of the TH3 axis
 * @param r1 array of th1-coordinate in the joint space
 * @param r2 aray of th2-coordinate in the joint space
 * @param r_out aray of th3-coordinate in the joint 
 * @param curr current position in the TH3 coordinate
 * @param trg target position in the TH3 coordinate
 * @param tf total path time in seconds
 * @param vec_length number of elements of the trajectory array
*/
void _th3AngularTrajectory(const float *r1, const float *r2, float *r_out, float curr, float trg, float tf, uint16_t tot_steps) {
  float a0 = curr;
  float a2 = -3 * (curr - trg) / pow(tf, 2);
  float a3 = 2 * (curr - trg) / pow(tf, 3);
  for (uint16_t i = 0; i < tot_steps; i++) {
    float t = tf * i / tot_steps;
    float angle = -(r1[i] + r2[i]) + (a0 + a2 * t * t + a3 * t * t * t);
    if (angle > TWO_PI){angle = angle - TWO_PI;}
    r_out[i] = angle;
  }
}

// TO DO: check formula to see which L0 is L1 and L2

/** Calculates the joint angles th1 and th2 based on an XY cartesian position
 * @param x X-coordinate in the cartesian space
 * @param y Y-coordinate in the cartesian space
 * @param th1 th1-coordinate in the joint space
 * @param th2 th2-coordinate in the joint space
*/
void _inverseKinematics(float x, float y, double *th1, double *th2) {
  double L0 = 200;   // Arm segment length (mm)
  double cQk = (sq(x) + sq(y) - 2 * sq(L0)) / (2 * sq(L0));  //Compute sin and cos of elbow joint
  double sQk = -sqrt(1 - sq(cQk));
  *th1 = atan2(y, x) - atan2(L0 * sQk, L0 + L0 * cQk);   // Compute th1 ('Shoulder Joint')
  *th2 = atan2(sQk, cQk);    // Compute th2 ('Elbow Joint')
}

/** Calculates 3rd order polynomial path from current to target position in the XY plane passing through 1 or 2 via points
 * @param rx pointer to array of X-coordinate of the cartesian path
 * @param ry pointer to array of Y-coordinate of the cartesian path
 * @param curr_x current position in the x coordinate
 * @param curr_y current position in the y coordinate
 * @param trg_x target position in the x coordinate
 * @param trg_y target position in the y coordinate
 * @param vp1x via point 1 X-coordinate
 * @param vp2x via point 2 X-coordinate
 * @param vp1y via point 1 Y-coordinate
 * @param vp2y via point 2 Y-coordinate
 * @param tf total path time in seconds
 * @param vec_length number of elements of the trajectory array 
*/
void _xyCartesianPathWithViaPoints(float *rx, float *ry, float curr_x, float curr_y, float trg_x, float trg_y, float vp1x, float vp2x, float vp1y, float vp2y, float tf, uint16_t vec_length){
  // Case Only 1 Via Point
  if(vp2x == 0){
    // Segment 1: current to via point 1
    float a10x = curr_x;
    float a11x = 0;
    float a12x = (12 * vp1x - 3 * trg_x - 9 * curr_x) / (4 * sq(tf));
    float a13x = (-8 * vp1x + 3 * trg_x + 5 * curr_x) / (4 * tf * tf * tf);

    float a10y = curr_y;
    float a11y = 0;
    float a12y = (12 * vp1y - 3 * trg_y - 9 * curr_y) / (4 * sq(tf));
    float a13y = (-8 * vp1y + 3 * trg_y + 5 * curr_y) / (4 * tf * tf * tf);

    // Segment 2: via point 1 to target

    float a20x = vp1x;
    float a21x = (3 * trg_x - 3 * curr_x) / (4 * tf);
    float a22x = (-12 * vp1x + 6 * trg_x + 6 * curr_x) / (4 * sq(tf));
    float a23x = (8 * vp1x - 5 * trg_x - 3 * curr_x) / (4 * tf * tf * tf);

    float a20y = vp1y;
    float a21y = (3 * trg_y - 3 * curr_y) / (4 * tf);
    float a22y = (-12 * vp1y + 6 * trg_y + 6 * curr_y) / (4 * sq(tf));
    float a23y = (8 * vp1y - 5 * trg_y - 3 * curr_y) / (4 * tf * tf * tf);    

    //Compute trajectory and store it in r1 array
    for (int i = 0; i <= vec_length; i++) {
      float t = (2 * tf * i) / vec_length;
      if (t <= tf) {
        rx[i] = a10x + a11x * t + a12x * t * t + a13x * t * t * t;
        ry[i] = a10y + a11y * t + a12y * t * t + a13y * t * t * t;
      } else {
        float t2 = t - tf;
        rx[i] = a20x + a21x * t2 + a22x * t2 * t2 + a23x * t2 * t2 * t2;
        ry[i] = a20y + a21y * t2 + a22y * t2 * t2 + a23y * t2 * t2 * t2;
      }
    }
  // Case 2 Via Points
  }else{

    float d_q0x = vp1x - curr_x;
    float d_q1x = vp2x - vp1x;
    float d_q2x = trg_x - vp2x;

    float d_q0y = vp1y - curr_y;
    float d_q1y = vp2y - vp1y;
    float d_q2y = trg_y - vp2y;

    // Segment 1: current to via point 1
    float a10x = curr_x;
    float a11x = 0;
    float a12x = (d_q2x - 3 * d_q1x + 11 * d_q0x) / (5 * tf * tf);
    float a13x = (-d_q2x + 3 * d_q1x - 6 * d_q0x) / (5 * tf * tf * tf);

    float a10y = curr_y;
    float a11y = 0;
    float a12y = (d_q2y - 3 * d_q1y + 11 * d_q0y) / (5 * tf * tf);
    float a13y = (-d_q2y + 3 * d_q1y - 6 * d_q0y) / (5 * tf * tf * tf);

    // Segment 2: via point 1 to via point 2
    float a20x = vp1x;
    float a21x = (-d_q2x + 3 * d_q1x + 4 * d_q0x) / (5 * tf);
    float a22x = (-2 * d_q2x + 6 * d_q1x - 7 * d_q0x) / (5 * tf * tf);
    float a23x = (3 * d_q2x - 4 * d_q1x + 3 * d_q0x) / (5 * tf * tf * tf);

    float a20y = vp1y;
    float a21y = (-d_q2y + 3 * d_q1y + 4 * d_q0y) / (5 * tf);
    float a22y = (-2 * d_q2y + 6 * d_q1y - 7 * d_q0y) / (5 * tf * tf);
    float a23y = (3 * d_q2y - 4 * d_q1y + 3 * d_q0y) / (5 * tf * tf * tf);

    // Segment 3: via point 2 to target
    float a30x = vp2x;
    float a31x = (4 * d_q2x + 3 * d_q1x - 1 * d_q0x) / (5 * tf);
    float a32x = (7 * d_q2x - 6 * d_q1x + 2 * d_q0x) / (5 * tf * tf);
    float a33x = (-6 * d_q2x + 3 * d_q1x - 1 * d_q0x) / (5 * tf * tf * tf);

    float a30y = vp2y;
    float a31y = (4 * d_q2y + 3 * d_q1y - 1 * d_q0y) / (5 * tf);
    float a32y = (7 * d_q2y - 6 * d_q1y + 2 * d_q0y) / (5 * tf * tf);
    float a33y = (-6 * d_q2y + 3 * d_q1y - 1 * d_q0y) / (5 * tf * tf * tf);

    //Compute trajectory and store it in r1 array
    for (int i = 0; i <= vec_length; i++) {
      float t = (3 * tf * i) / vec_length;
      if (t <= tf) {
        rx[i] = a10x + a11x * t + a12x * t * t + a13x * t * t * t;
        ry[i] = a10y + a11y * t + a12y * t * t + a13y * t * t * t;
      } else if (t > tf && t <= 2 * tf) {
        float t2 = t - tf;
        rx[i] = a20x + a21x * t2 + a22x * t2 * t2 + a23x * t2 * t2 * t2;
        ry[i] = a20y + a21y * t2 + a22y * t2 * t2 + a23y * t2 * t2 * t2;
      } else {
        float t3 = t - 2 * tf;
        rx[i] = a30x + a31x * t3 + a32x * t3 * t3 + a33x * t3 * t3 * t3;
        ry[i] = a30y + a31y * t3 + a32y * t3 * t3 + a33y * t3 * t3 * t3;
      }
    }

  }
  tf = 2 * tf;
}

void cartesianPathWithViaPoints(float r1[], int tot_steps, float trg, float curr, float tf, float vp1, float vp2) {
  //Second segment: if 1 via-point, then vp1 to x_f ; if 2 via-points, then vp1 to vp2
  //Third segment(only if 2 via-points): vp2 to x_f
  if (vp2 == 0) {
    //vp1 to x_f
    float a10 = curr;
    float a11 = 0;
    float a12 = (12 * vp1 - 3 * trg - 9 * curr) / (4 * sq(tf));
    float a13 = (-8 * vp1 + 3 * trg + 5 * curr) / (4 * tf * tf * tf);
    
    float a20 = vp1;
    float a21 = (3 * trg - 3 * curr) / (4 * tf);
    float a22 = (-12 * vp1 + 6 * trg + 6 * curr) / (4 * sq(tf));
    float a23 = (8 * vp1 - 5 * trg - 3 * curr) / (4 * tf * tf * tf);

    //Compute trajectory and store it in r1 array
    for (int i = 0; i <= tot_steps; i++) {
      float t = (2 * tf * i) / tot_steps;
      if (t <= tf) {
        r1[i] = a10 + a11 * t + a12 * t * t + a13 * t * t * t;
      } else {
        float t2 = t - tf;
        r1[i] = a20 + a21 * t2 + a22 * t2 * t2 + a23 * t2 * t2 * t2;
      }
    }

    tf = 2 * tf;

  } else {

    float d_q0 = vp1 - curr;
    float d_q1 = vp2 - vp1;
    float d_q2 = trg - vp2;

    float a10 = curr;
    float a11 = 0;
    float a12 = (d_q2 - 3 * d_q1 + 11 * d_q0) / (5 * tf * tf);
    float a13 = (-d_q2 + 3 * d_q1 - 6 * d_q0) / (5 * tf * tf * tf);

    float a20 = vp1;
    float a21 = (-d_q2 + 3 * d_q1 + 4 * d_q0) / (5 * tf);
    float a22 = (-2 * d_q2 + 6 * d_q1 - 7 * d_q0) / (5 * tf * tf);
    float a23 = (3 * d_q2 - 4 * d_q1 + 3 * d_q0) / (5 * tf * tf * tf);

    float a30 = vp2;
    float a31 = (4 * d_q2 + 3 * d_q1 - 1 * d_q0) / (5 * tf);
    float a32 = (7 * d_q2 - 6 * d_q1 + 2 * d_q0) / (5 * tf * tf);
    float a33 = (-6 * d_q2 + 3 * d_q1 - 1 * d_q0) / (5 * tf * tf * tf);

    //Compute trajectory and store it in r1 array
    for (int i = 0; i <= tot_steps; i++) {
      float t = (3 * tf * i) / tot_steps;
      if (t <= tf) {
        r1[i] = a10 + a11 * t + a12 * t * t + a13 * t * t * t;
      } else if (t > tf && t <= 2 * tf) {
        float t2 = t - tf;
        r1[i] = a20 + a21 * t2 + a22 * t2 * t2 + a23 * t2 * t2 * t2;
      } else {
        float t3 = t - 2 * tf;
        r1[i] = a30 + a31 * t3 + a32 * t3 * t3 + a33 * t3 * t3 * t3;
      }
    }
    tf = 2 * tf;
  }
}

//----------------------------------------------------Angular Trajectories------------------------------------------------//
//Takes r1 and r2 as arrays containing the cartesian path. Then calculates the angular trajectories for this path
//and overwrites r1 and r2 arrays

/** Converts between cartesian coordinates and joint angle coordinates
 * @param r1 aray of X-coordinate of the cartesian path. It is converted into the array of th1-coordinates in the joint space
 * @param r2 aray of Y-coordinate of the cartesian path. It is converted into the array of th2-coordinates in the joint space
 * @param vec_length number of elements of the trajectory array
*/
void _xyToAngularTrajectories(float *r1, float *r2, uint16_t vec_length) {
  for (uint16_t i = 0; i < vec_length; i++) {
    //_inverseKinematics(r1[i], r2[i], r1[i], r2[i]);
    float L = 200;                                                  // Arm segment length (mm)
    float cQk = (sq(r1[i]) + sq(r2[i]) - 2 * sq(L)) / (2 * sq(L));  //Compute sin and cos of elbow joint
    float sQk = -sqrt(1 - sq(cQk));
    r1[i] = atan2(r2[i], r1[i]) - atan2(L * sQk, L + L * cQk);  //r1 and r2 now contain the values of the angular trajectories
    r2[i] = atan2(sQk, cQk);                                    //r1 is theta1 ('shoulder joint') and r2 is theta2 ('elbow joint')

  }
}

//--------------------------------------------------Trajectory Digitalization----------------------------------------------//
//INPUT: angular trajectory as an array of floats where each element contains the angle at a certain point of the trajectory
//OUTPUT: stores the number of steps to go from one point of the trajectory to the next as an array of integers
//The output array contains the number of steps that need to be performed at every time interval (ti)

/** Converts a trajectory from angular coordinates to number of steps
 * @param r_in angular trajectory as an array of floats where each element contains the angle at a certain point of the trajectory
 * @param r_out stores the number of steps to go from one point of the trajectory to the next as an array of integers
 * @param vec_length number of elements of the trajectory array
 * @param min_step angle change per step
 * @param checkSingularity true only for revolute joints. Not used for linear joints.
*/
void _trajDigitization(const float *r_in, volatile int16_t *r_out, uint16_t vec_length, double min_step, bool checkSingularity) {
  double curr_angle = r_in[0];  //Keep track of the current angular position
  //Serial.print("Motor Steps: "); Serial.println(mot_steps);
  for (uint16_t i = 0; i < vec_length; i++) {
    //Calculate the nummber of steps to go from r_in [i] to r_in [i - 1] as a float value
    if (checkSingularity) {
      if (abs(r_in[i] - curr_angle) >= 3.14) {
        if (curr_angle > 0)
          curr_angle = curr_angle - 2 * PI;
        else
          curr_angle = curr_angle + 2 * PI;
      }
    }
    float th_steps = (r_in[i] - curr_angle) / min_step;
    uint16_t quo = (uint16_t)th_steps;
    float rem = th_steps - (uint16_t)th_steps;
    //Compute que quotient (number of full-steps) and remainder of the previous division
    //quotient_remainder(th1_steps, quo, rem);

    if (th_steps > 0.5) {    //when the number of steps is positive
      if (rem > 0.5) {       //if the remainder is > 0.5
        r_out[i] = quo + 1;  //add 1 step to the full-steps number
      } else {
        r_out[i] = quo;  //else, apply the full-steps number
      }
    } else if (th_steps < -0.5) {  //when the number of steps is negative
      if (rem < -0.5) {            //if the remainder is < -0.5
        r_out[i] = quo - 1;        //subtract 1 step to the full-steps number
      } else {
        r_out[i] = quo;  //else, apply the full-steps number
      }
    } else {
      r_out[i] = 0;  //if the number of steps is <0.5 and >-0.5, set steps to 0
    }
    curr_angle += r_out[i] * min_step;  //Update current position
  }
}

/** Convert between number of acceleration steps into rad/s2
 * @param freq target frequency in Hz at the end of acceleration
 * @param acc_steps number of acceleration steps
 * @param step_size step size in mm
 * @return acceleration in rad/s2
*/
float _convert_AccSteps_2_AccRads2(int freq, uint32_t acc_steps, float step_size) {
  float v_0 = (float(freq) / 10) * step_size; // Calculate initial, final and average velocities
  float v_f = freq * step_size;
  float v_avg = (v_f + v_0) / 2;
  float travel = acc_steps * step_size;       // Travel (rad) at the end of acceleration phase
  float acc_time = travel / v_avg;            // Time taken for the motor to reach the end of the acceleration phase
  float acc = (v_f - v_0) / acc_time;         // Acceleration value (rad/s2)
  return acc;
}




