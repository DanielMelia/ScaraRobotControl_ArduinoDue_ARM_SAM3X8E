#ifndef ScaraDue_h
#define ScaraDue_h

#include "Arduino.h"
#include "Stepper.h"
#include "utils.h"

#define vec_length 100                // Length of the look-up vectors
#define freq_init 1000

// Other motor pins
#define TH1_Zhome 37  // Signal to send Oriental Motors to home position
#define TH2_Zhome 36
#define TH1_Moving 35 // Signal to read if Oriental Motors are moving
#define TH2_Moving 34
#define TH1_Stop 32  // Signal to send Oriental Motors to stop
#define TH2_Stop 33 
#define TH1_Ready 38 // Signal to read if Oriental Motors are ready to move
#define TH2_Ready 39

//------------------------------------------ CONTROL MODES -------------------------------------------------------------//
enum ControlMode{
	STP,    //STOP mode
	MAN,    //MANUAL mode: the user inputs directly the number of steps to move for each motor (homing not needed)
	HOM,    //HOME mode: the user sets the initial cartesian coordinates, so the program can compute the initial joint angles
    TRAJ,   //TRAJECTORY mode: the user inputs the target position and trajectory time. Then a trajectory is computed and executed (homing needed)
    VEL,    //VELOCITY mode: or speed control mode. Useful for Joystick control for example. Inputs are velocity and direction (homing needed)
    CDM     // CONTROLLED DESCENT MODE: for Z-axis only. Mode moves down at controlled speed while monitoring the vacuum (for pick-up or similar)
};

enum PinMode{
    Output,
    Input,
    Input_PullUp
};

class ScaraDue
{
    public:
        // Axes Objects
        Stepper Z;
        Stepper TH1;
        Stepper TH2;
        Stepper TH3;
        //Stepper steppers[4];
        ControlMode controlMode;
        bool isHomed() {return _isHomed;}
        
        // Set Stepper Motors Parameters
        void SetSteppersGPIOPort(Pio* GPIO_x);
        void SetAxis_Z(uint32_t PPR, float pitch, uint8_t pulse_pin, uint8_t dir_pin);
        void SetAxis_TH1(uint32_t PPR, uint8_t cw_pulse_pin, uint8_t ccw_pulse_pin);
        void SetAxis_TH2(uint32_t PPR, uint8_t cw_pulse_pin, uint8_t ccw_pulse_pin);
        void SetAxis_TH3(uint32_t PPR, uint8_t pulse_pin, uint8_t dir_pin);
        bool Init();

        // Home
        bool StartHomingProcedure(float home_x, float home_y, float home_z, float home_r);
        bool ExecuteHomingProcedure();

        // Setup Different Motion Control Modes
        bool Move_ManualMode(int32_t steps_z, int32_t steps_th1, int32_t steps_th2, int32_t steps_th3, uint32_t freq_hz);
        bool Move_TrajectoryMode(float trg_x, float trg_y, float trg_z, float trg_r, float t_xyr, float t_z);
        bool Move_TrajectoryMode_SingleAxis(float step_size, int axis, float t_time);
        bool setupDescendModeTrajectory(float lowest_z_mm, float vf_mms, float pressureThreshold, uint8_t descent_mode, float acc_dist_mm = -1);

        // Interrupt Service Routines - to be runned inside the Timer Interrupt Handler
        void MotorRun_Z();
        void MotorRun_TH1();
        void MotorRun_TH2();
        void MotorRun_TH3();

        // Get Feedback Functions
        void UpdateEndEffectorCartesianPosition();
        PositionXYZR GetCurrentPosition();
        bool checkPositionIsWithinWS(float x, float y, float z, float r);

    private:
        // ----------------------------------------------------- //
        // ---- Stepper Motors Pins Variables and Functions ---- //
        // ----------------------------------------------------- //

        Pio* _GPIO_STEPPERS;
        uint8_t _PLS_PIN_Z;
        uint8_t _DIR_PIN_Z;
        uint8_t _PLSCW_PIN_TH1;
        uint8_t _PLSCCW_PIN_TH1;
        uint8_t _PLSCW_PIN_TH2;
        uint8_t _PLSCCW_PIN_TH2;
        uint8_t _PLS_PIN_TH3;
        uint8_t _DIR_PIN_TH3;

        void _step_Z();
        void _setDirection_Z(bool dir);
        void _step_TH1();
        void _setDirection_TH1(bool dir);
        void _step_TH2();
        void _setDirection_TH2(bool dir);
        void _step_TH3();
        void _setDirection_TH3(bool dir);
        bool _initialiseMotorPins();

        void _toggleMotorPin(uint32_t pin_mask, volatile bool &pin_status);
        void _setMotorPin(uint32_t pin_mask);
        void _clearMotorPin(uint32_t pin_mask);

        // ---------------------------------------- //
        // ---- Timers Variables and Functions ---- //
        // ---------------------------------------- //

        uint32_t _prescaler;

        void _timerStart(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq);
        void _timerUpdate(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq);
        void _timerStop(Tc *tc, uint32_t channel, IRQn_Type irq);

        void _startZTimer();
        void _startTh1Timer();
        void _startTh2Timer();
        void _startTh3Timer();   

        void _startXYRTimers();
        void _initialiseAllTimers();
        
        //void SetupMotorsBeforeStart();
        void _stopTimerZ();
        void _stopTimerTH1();
        void _stopTimerTH2();
        void _stopTimerTH3();
        void _stopAllTimers();

        // -------------------------------- //
        // ---- SCARA Status Variables ---- //
        // -------------------------------- //
        
        bool _isHomed;
        volatile bool _isHomed_Z;
        volatile bool _isHomed_XY;
        volatile bool _isHomed_R;
        bool _homingStarted;
        bool _isMoving;
        bool _isMovingUp;
        bool _isMovingXYR;
        bool _isMovingZ;
        bool _stopVelMode;
        bool _joystickControl;

        bool _th1_isMoving;
        bool _th2_isMoving;
        void Check_Th1Th2_moving();

        // ----------------------------------------------------------------------- //
        // ---- Current End-Effector Position and Workspace Limits and Checks ---- //
        // ----------------------------------------------------------------------- //

        float _curr_x; // Current end-effector position
        float _curr_y;
        float _curr_z;
        float _curr_r;
        float _L1; // length of the upper-arm segmeng
        float _L2; // length of the lower-arm segment

        float _workSpace_XY_min_radius = 125;
        float _workSpace_XY_max_radius = 390;
        float _workSpace_Z_max = 100;
        float _workSpace_Z_min = 0;

        // Maximum allowed velocities per axis
        float _maxSpeed_XY = 300;
        float _maxSpeed_Z = 150;
        float _maxSpeed_R = 2; 

        bool _checkValidTargetPosition(float x, float y, float z, float r);
        bool _checkValidTrajectory(float x_f, float y_f);
        void _checkTrajectoryTimes(float trg_x, float trg_y, float trg_z, float trg_r, float &t_z, float &t_xy, float &t_r);
        
        // 8-bit register that indicates if the robot is moving in the positive or negative direction in any of the 4 axes
        uint8_t _velocityDirectionFlags;
        void _setVelocityDirectionFlags(float vx, float vy, float vz, float vr);
        void readVelocityDirectionFlags();

        // ------------------------------------------- //
        // ---- Controlled Descent Mode Variables ---- //
        // ------------------------------------------- //

        // Controlled descent Mode moves down at controlled speed while monitoring the vacuum
        uint8_t _descentMode; // 0 -> No controlled descend / 1 -> Controlled descend for pickup / 2 -> Controlled descend for jig load
        volatile uint8_t _descendMode_result; //0 --> Undefined ; 1 --> Success; 2 --> Fail
        //volatile uint32_t DescendMode_CurrentStepCount;
        //uint32_t DescendMode_TotalSteps;
        //uint32_t DescendMode_DecStart;
        
        float _lowest_z;

        // ----------------------------------------------------------------- //
        // ---- Trapezoidal Acceleration/Deceleration Profile Variables ---- //
        // ----------------------------------------------------------------- //

        uint32_t _c0;
        volatile uint32_t _prev_counter_value;
        volatile uint32_t _rest;
        uint32_t _accSteps;   // Number of motor steps to reach the end of acceleration. Calculated based on acceleration distance   
        uint32_t _decStart;    

        // ------------------------------------------- //
        // ---- Trajectory Control Mode Variables ---- //
        // ------------------------------------------- //

        // A 3rd Order Polynomial Path is pre-calculated before the motion starts. It generates a look-up vector for each axis to indicate how many steps does the
        // motor need to perform during each trajectory time interval. The timers perform the switching sequence for all motors

        // Trajectory time interval (trajectory Time / vector length). The timers' frequencies are updated at each time interval
        volatile float _ti_xyr, _ti_z;

        volatile int16_t _mZ_steps[vec_length];    //Vector containing steps per each time interval (z axis)
        volatile int16_t _mTH1_steps[vec_length];  //Vector containing steps per each time interval (th1)
        volatile int16_t _mTH2_steps[vec_length];  //Vector containing steps per each time interval (th2)
        volatile int16_t _mTH3_steps[vec_length];  //Vector containing steps per each time interval (th3)

        void _trajectoryCalculationZ(float tf, float tg_z);
        void _trajectoryCalculationXYR(float tf, float tg_x, float tg_y, float tg_r);

        // ----------------------------------- //
        // ---- Pressure Sensor Variables ---- //
        // ----------------------------------- //

        float _airPressure;
        float _refAirPressure;

};

extern ScaraDue SCARA;

#endif