#ifndef ScaraDue_h
#define ScaraDue_h

#include "Arduino.h"
#include "Stepper.h"
#include "utils.h"
#include "AMS.h"
#include "FastLED/FastLED.h"
#include "RobotConfig.h"

#define vec_length 100                // Length of the look-up vectors
#define freq_init 1000

// Other motor pins
//#define TH1_Zhome 37  // Signal to send Oriental Motors to home position
//#define TH2_Zhome 36
//#define TH1_Moving 35 // Signal to read if Oriental Motors are moving
//#define TH2_Moving 34
//#define TH1_Stop 32  // Signal to send Oriental Motors to stop
//#define TH2_Stop 33 
//#define TH1_Ready 38 // Signal to read if Oriental Motors are ready to move
//#define TH2_Ready 39

//------------------------------------------ CONTROL MODES -------------------------------------------------------------//
enum ControlMode{
	STP,    //STOP mode
	MAN,    //MANUAL mode: the user inputs directly the number of steps to move for each motor (homing not needed)
	HOM,    //HOME mode: the user sets the initial cartesian coordinates, so the program can compute the initial joint angles
    TRAJ,   //TRAJECTORY mode: the user inputs the target position and trajectory time. Then a trajectory is computed and executed (homing needed)
    VEL,    //VELOCITY mode: or speed control mode. Useful for Joystick control for example. Inputs are velocity and direction (homing needed)
    CDM     // CONTROLLED DESCENT MODE: for Z-axis only. Mode moves down at controlled speed while monitoring the vacuum (for pick-up or similar)
};



class ScaraDue
{
    public:

        // Public Read-Only Members
        bool isHomed() {return _isHomed;}
        bool isMoving() {return _isMoving;}

        // Axes Objects
        Stepper Z;
        Stepper TH1;
        Stepper TH2;
        Stepper TH3;

        // 

        bool Init();
        void MainLoopFunction(bool serialSend);        
        
        // Home

        bool StartHomingProcedure(float home_x, float home_y, float home_z, float home_r);    // Serial Command      
        void InitialiseRobotPosition(float x, float y, float z, float r);    // Serial Command

        // Setup Different Motion Control Modes
        
        bool Move_ManualMode(int32_t steps_z, int32_t steps_th1, int32_t steps_th2, int32_t steps_th3, uint32_t freq_hz);    // Serial Command
        bool Move_TrajectoryMode(float trg_x, float trg_y, float trg_z, float trg_r, float t_xyr, float t_z);    // Serial Command
        bool Move_TrajectoryMode_SingleAxis(float step_size, uint8_t axis, float t_time);    // Serial Command
        bool Move_ControlledDescendMode(float lowest_z_mm, float vf_mms, float pressureThreshold, uint8_t descent_mode, float acc_dist_mm = -1);    // Serial Command
        bool Move_VelocityMode(float vel_mms, float acc_time_sec, uint8_t axis);
        void Stop(){_stopAllTimers();}    // Serial Command

        // TODO: Need to implement Stop With Deceleration and Move Velocity Mode - will need timer 7 

        // Interrupt Service Routines - to be run inside the Timer Interrupt Handler

        void MotorRun_Z();
        void MotorRun_TH1();
        void MotorRun_TH2();
        void MotorRun_TH3();
        void Timer7();

        // Get Feedback Functions

        void UpdateEndEffectorCartesianPosition(); // main loop function
        PositionXYZR GetCurrentPosition();
        bool checkPositionIsWithinWS(float x, float y, float z, float r); // main loop function
        void Check_Th1Th2_isMoving(); // main loop function
        void Check_Th1Th2_isReady(); // main loop function
        float readAirPressure(); // main loop function
        uint8_t readValves(); // main loop function

        // Auxiliary Components: pressure sensor, valves, LEDs...

        void SetBackLightBrightness(uint8_t panel_number, uint8_t brightness, CRGB::HTMLColorCode color);
        void SetBackLightBrightness(uint8_t panel_number, uint8_t brightness, uint8_t color);   //  Serial Command
        void RingLEDControl(bool state);    //  Serial Command
        void vacuum_on();   //  Serial Command
        void vacuum_off();  //  Serial Command
        void puff();
        void StartReleaseWthPuff(float offTime_ms, float puffTime_ms);  // Serial Command   

        void DEV_testAuxiliaryPins();
        void DEV_testStepPins();
        void DEV_testPressureSensor();


        // Set Stepper Motors Parameters
        //void SetSteppersGPIOPort(Pio* GPIO_x);
        //void SetAxis_Z(uint32_t PPR, float pitch, uint8_t pulse_pin, uint8_t dir_pin);
        //void SetAxis_TH1(uint32_t PPR, uint8_t cw_pulse_pin, uint8_t ccw_pulse_pin);
        //void SetAxis_TH2(uint32_t PPR, uint8_t cw_pulse_pin, uint8_t ccw_pulse_pin);
        //void SetAxis_TH3(uint32_t PPR, uint8_t pulse_pin, uint8_t dir_pin);
        //void SetIOSignalsPins_TH1(uint8_t home_pin, uint8_t stop_pin, uint8_t isMoving_pin, uint8_t isReady_pin);
        //void SetIOSignalsPins_TH2(uint8_t home_pin, uint8_t stop_pin, uint8_t isMoving_pin, uint8_t isReady_pin);
        //void SetLimitSwitchesPins(uint8_t z_limitswitch_pin, uint8_t th3_limitswitch_pin);

    private:

        bool _executeHomingProcedure(); // main loop function
        void _releaseWthPuff(); // main loop function
        void _initAMS();
        void _initLEDArray();
        void _initialiseAxesParameters();   // init function
        bool _initialiseMotorPins();   // init function
        void _setIOSignalsPins_TH1();   // init function
        void _setIOSignalsPins_TH2();   // init function
        void _setLimitSwitchesPins();   // init function
        void _setAuxiliaryComponents();   // init function

        ControlMode _controlMode;

        CRGB colors[6];
        struct CRGB *backLitPanels[NUM_PANELS];
        //CRGB raw_leds[NUM_LEDS];
        // Pressure Sensor: AMS 5812-0150-D , differential/relative, 0 ...1034 mbar, 0 ...15 psi
        AMS dPress; //define the sensor's instance with the sensor's family, sensor's I2C address as well as its specified minimum and maximum pressure
        // (5812, 0x78,0,1034)
        // ----------------------------------------------------- //
        // ---- Stepper Motors Pins Variables and Functions ---- //
        // ----------------------------------------------------- //

        //Pio* _GPIO_STEPPERS;
        // uint8_t _PLS_PIN_Z;
        // uint8_t _DIR_PIN_Z;
        // uint8_t _PLSCW_PIN_TH1;
        // uint8_t _PLSCCW_PIN_TH1;
        // uint8_t _PLSCW_PIN_TH2;
        // uint8_t _PLSCCW_PIN_TH2;
        // uint8_t _PLS_PIN_TH3;
        // uint8_t _DIR_PIN_TH3;

        // uint8_t _HOM_PIN_TH1;
        // uint8_t _HOM_PIN_TH2;
        // uint8_t _STP_PIN_TH1;
        // uint8_t _STP_PIN_TH2;
        // uint8_t _MOV_PIN_TH1;
        // uint8_t _MOV_PIN_TH2;
        // uint8_t _RDY_PIN_TH1;
        // uint8_t _RDY_PIN_TH2;

        // uint8_t _LimSw_PIN_Z;
        // uint8_t _LimSw_PIN_TH3;

        

        bool _LimSw_VALUE_Z;
        bool _LimSw_VALUE_TH3;

        void _step_Z();
        void _setDirection_Z(bool dir);
        void _step_TH1();
        void _setDirection_TH1(bool dir);
        void _step_TH2();
        void _setDirection_TH2(bool dir);
        void _step_TH3();
        void _setDirection_TH3(bool dir);
        

        void _toggleMotorPin(uint32_t pin_mask, volatile bool &pin_status);
        void _setMotorPin(uint32_t pin_mask);
        void _clearMotorPin(uint32_t pin_mask);

        void _homeStart_Z();
        void _homeStart_XY();
        void _homeStart_R();

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
        bool _xyHomePulseSent;
        bool _xyHomePulseCleared;
        bool _rHomingStarted;
        bool _isMoving;
        bool _isMovingUp;
        bool _isMovingXYR;
        bool _isMovingZ;

        bool _th1_isMoving;
        bool _th2_isMoving;
        bool _th1_isReady;
        bool _th2_isReady;       

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
        void _zLinearAccelerationSetup(float vf_mms, float acc_dist_mm); 

        // ----------------------------------------- //
        // ---- Velocity Control Mode Variables ---- //
        // ----------------------------------------- //

        bool _stopVelMode;
        //bool _joystickControl;
        uint8_t _velMode_rampCount;
        // _velModeAccSteps = 10;
        float v_X;
        float v_Y;
        uint32_t _t7_freq;

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
        float _prevAirPressure;
        float _pressureChange;
        float _pressureFilterGain;

        uint8_t _puffInit;
        uint8_t _puffEnd;
        bool _releaseWithPuff;
        uint8_t _releaseCount;

        bool _isPS24V_ok;
        bool _isPS48V_ok;

        uint8_t _valvesState; // This reads directly the state of the valves
        uint8_t _vacuumMode;  // This is the selected vacuum mode. Should match with _valvesState

        bool _valveStateChanged();
        bool _vacuumModeChanged();
        uint8_t _lastVS[5];
        uint8_t _lastVM[5];
        uint8_t _vsChanged_Idx;
        uint8_t _vmChanged_Idx;

};

extern ScaraDue SCARA;

#endif
