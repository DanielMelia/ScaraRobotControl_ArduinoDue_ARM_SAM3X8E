#ifndef RobotConfig_h
#define RobotConfig_h

// ------------------------------------------------- //
// ----------- STEPPER MOTORS PINS ----------------- //
// ------------------------------------------------- //

// GPIO PORT C
// - C.1 (Pin 33)
// - C.2 (Pin 34) -> Th1 Stop Pin
// - C.3 (Pin 35) -> Th2 Stop Pin
// - C.4 (Pin 36) -> Th1 Read Pin
// - C.5 (Pin 37) -> Th2 Ready Pin
// - C.6 (Pin 38) -> Th1 Moving Pin
// - C.7 (Pin 39) -> Th2 Moving Pin
// - C.8 (Pin 40) -> Th1 Home Pin
// - C.9 (Pin 41) -> Th2 Home Pin

// - C.19 (Pin 44)  -> Th3 Pulse
// - C.18 (Pin 45)  -> Th3 Direction
// - C.17 (Pin 46)  -> Z Pulse
// - C.16 (Pin 47)  -> Z Direction
// - C.15 (Pin 48)  -> Th2 ClockWise
// - C.14 (Pin 49)  -> Th2 Counter ClockWise
// - C.13 ((Pin 50) -> Th1 ClockWise
// - C.12 (Pin 51)  -> Th1 Counter ClockWise

// - C.21 (Pin 9)  -> 
// - C.22 (Pin 8)  -> 
// - C.23 (Pin 7)  -> 
// - C.24 (Pin 6)  -> 
// - C.25 (Pin 5)  -> 
// - C.26 (Pin 4)  -> 

// - C.28 (Pin 3)  -> 
// - C.29 (Pin 10)  -> 

// Stepper Motors PORT
#define GPIO_PORT_STEPPERS PIOC

// Pulse-Direction Pins
#define PIN_TH3_PULSE     19  // Pin 44
#define PIN_TH3_DIR       18  // Pin 45
#define PIN_Z_PULSE       17  // Pin 46
#define PIN_Z_DIR         16  // Pin 47
#define PIN_TH2_CW        15  // Pin 48
#define PIN_TH2_CCW       14  // Pin 49
#define PIN_TH1_CW        13  // Pin 50
#define PIN_TH1_CCW       12  // Pin 51

// Th1 and Th2 IO Signals Pins: This are Oriental Motors. Drivers have a IO Signal connector which can be configured to provide different feedback/control signals
#define PIN_TH1_HOME      5   // Pin 37 -> OUTPUT: Send pulse to this pin to home motor
#define PIN_TH1_MOVING    3   // Pin 35 -> INPUT:  Check if the motor is moving
#define PIN_TH1_READY     6   // Pin 38 -> INPUT:  Check if the motor is ready to receive pulse
#define PIN_TH1_STOP      28  // Pin 3  -> OUTPUT: Send pulse to this pin to stop motor
#define PIN_TH2_HOME      4   // Pin 36
#define PIN_TH2_MOVING    2   // Pin 34
#define PIN_TH2_READY     7   // Pin 39
#define PIN_TH2_STOP      1   // Pin 33  

// Limit Switches Pins
#define PIN_Z_LimSw       8   // Pin 40
#define PIN_TH3_LimSw     9   // Pin 41

// ------------------------------------------------- //
// ------- AUXILIARY COMPONENTS PINS --------------- //
// ------------------------------------------------- //

// Auxiliary Components Ports
#define GPIO_PORT_AUXILIARY PIOD

// Auxiliary Components Pins
#define PIN_VALVE_1       2   // Pin 27
#define PIN_VALVE_2       1   // Pin 26
#define PIN_LED_ARRAY     0   // Pin 25 -> This is related to _LED_ARRAY_PIN. If the port pin number is changed, the Arduino pin number should be changed as well
#define PIN_RING_LED      9   // Pin 30
#define PIN_PS24V_OK      3   // Pin 28
#define PIN_PS48V_OK      6   // Pin 29

// LED panels variables
#define _LED_ARRAY_PIN 25   // This is the Arduino Pin (Not the port pin number)
#define NUM_LEDS 320
#define NUM_LEDS_PANEL 64
#define NUM_PANELS 5
// If the LED panel configuration is changed, there might be changed needed inside _initLEDArray() as well

// ------------------------------------------------- //
// -------------- OTHER PARAMETERS ----------------- //
// ------------------------------------------------- //

#define Z_PITCH 5.0 // Pitch of Z axis in mm
#define Z_PPR 200
#define XY_PPR 200
#define R_PPR 200
#define SAMPLE_TIME 100

#endif