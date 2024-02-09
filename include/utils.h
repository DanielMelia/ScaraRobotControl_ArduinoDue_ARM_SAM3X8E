#include "Arduino.h"

struct PositionXYZR
{
    float X;
    float Y;
    float Z;
    float R;
};

bool getBit(uint8_t byte, uint8_t position);
//void quotient_remainder(float num, uint16_t &quo, float &rem);

// ---- Trajectory Calculation Functions ---- //

void _xyCartesianPath(float *rx, float *ry, float curr_x, float curr_y, float trg_x, float trg_y, float tf, uint16_t vec_length);
void _xyCartesianPathWithViaPoints(float *rx, float *ry, float curr_x, float curr_y, float trg_x, float trg_y, float vp1x, float vp2x, float vp1y, float vp2y, float tf, uint16_t vec_length);
void _zCartesianPath(float *r, float curr, float trg, float tf, uint16_t vec_length);
void _th3AngularTrajectory(const float *r1, const float *r2, float *r_out, float curr, float trg, float tf, uint16_t tot_steps);
//void _inverseKinematics(float x, float y, float &th1, float &th2);
//void cartesianPathWithViaPoints(float r1[], int tot_steps, float trg, float curr, float tf, float vp1, float vp2);
void _xyToAngularTrajectories(float *r1, float *r2, uint16_t vec_length);
void _trajDigitization(const float *r_in, volatile int16_t *r_out, uint16_t vec_length, double min_step, bool checkSingularity);
float _convert_AccSteps_2_AccRads2(int freq, uint32_t acc_steps, float step_size);

// ---- Timer Initialization Functions ---- //