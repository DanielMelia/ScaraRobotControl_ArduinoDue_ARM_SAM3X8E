#ifndef Stepper_h
#define Stepper_h

#include "Arduino.h"

class Stepper
{
    public:
        bool isTimerOn() {return _isTimerOn;}
        bool isMoving() {return _isMoving;}
        bool isInit() {return _arePinsConfigured;}
        int32_t stepPosition() {return _posSteps;}
    private:
    // ---- status variables --- //
        volatile bool _isMoving;
        volatile bool _isTimerOn;
        bool _arePinsConfigured;
    // ---- physical parameters --- //
        uint32_t _ppr;
        float _pitch_mm;
        double _minStep;  // minimum motor step size (in radians/mm)
        float _maxPos;
        float _minPos; 
    // ---- motor main operational parameters ---- //
        volatile float _pos = 0;              // current motor absolute position (in radians OR mm)
        volatile uint8_t _dir;                 // motor direction (1,-1)
        volatile bool _pulseStatus = false;    // TRUE if PULSE pin is HIGH
        volatile uint32_t _freq;               // step frequency
        volatile int32_t _posSteps;           // current motor absolute position (in steps)

    // ---- trajectory control parameters ---- //
        uint32_t _targetSteps = 0;         // total number of steps to move during current action
        volatile uint32_t _currentSteps;   // count of steps performed during current action
        volatile uint8_t _i;               // trajectory time interval
        volatile uint32_t _remSteps;       // remaining steps during time interval


    // Stepper gives access-rights to ScaraDue to all its private members
    friend class ScaraDue;
};

#endif