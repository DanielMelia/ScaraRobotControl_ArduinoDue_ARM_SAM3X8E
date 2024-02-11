#include <Arduino.h>
#include "ScaraDue.h"

//ScaraDue SCARA;

// GPIO PORT C
// - C.6 (Pin 38)
// - C.8 (Pin 40)
// - C.19 (Pin 44)  -> Th3 Pulse
// - C.17 (Pin 46)  -> Z Pulse
// - C.15 (Pin 48)  -> Th2 ClockWise
// - C.13 ((Pin 50) -> Th1 ClockWise
// - C.12 (Pin 51)  -> Th1 Counter ClockWise
// - C.14 (Pin 49)  -> Th2 Counter ClockWise
// - C.16 (Pin 47)  -> Z Direction
// - C.18 (Pin 45)  -> Th3 Direction
// - C.9 (Pin 41)
// - C.7 (Pin 39)
// - C.4 (Pin 36)
// - C.2 (Pin 34)
// - C.1 (Pin 33)
// - C.3 (Pin 35)
// - C.5 (Pin 37)

// 9 free pins (th1_home, th1_moving, th1_stop, th1_ready, th2_home, th2_moving, th2_stop, th2_ready)

// Initialise variables
static char firstChar;  // first char received indicates the command code
static const u_int8_t BUFFER_SIZE = 30; // max command length
static char buf[BUFFER_SIZE];   // incoming serial data
static const u_int8_t MAX_ARGS = 9; // max number of arguments for a command
static float args[MAX_ARGS];  // arguments array
static uint32_t sample_time = 100; // Main loop 'cycle time' (delay at the end of the loop)

void readSerialCommand();
uint8_t splitCommand(char *ptr, const char *delimiter1, const char *delimiter2, char *cmdCode , float *cmdArgs);
bool executeCommand(const char *cmdCode, float *cmdArgs, uint8_t argCount);

int main(void){
  Serial.begin(115200);
  SCARA.SetSteppersGPIOPort(PIOC);
  SCARA.SetAxis_Z(200, 5.0, 17, 16);  // PIN NUMBERS - pls: 46, dir: 47, ppr: 5000
  SCARA.SetAxis_TH1(200, 13, 12);   // PIN NUMBERS - cw : 50, ccw: 51, ppr:100000
  SCARA.SetAxis_TH2(200, 15, 14);   // PIN NUMBERS - cw : 48, ccw: 49, ppr:100000
  SCARA.SetAxis_TH3(200, 19, 18);    // PIN NUMBERS - pls: 44, dir: 45, ppr:25000
  if (!SCARA.Init())
    Serial.println("SCARA INIT FAIL");
  delay(1000);

  //SCARA.Move_TrajectoryMode(10, 10, 0, 0, 0.8, 0);

  SCARA.Move_ManualMode(20, 0, 0, 0, 50);
  //delay(3000); // here we have to wait until previous move finishes (controlMode = STP)

  while(SCARA.Z.isTimerOn() || SCARA.TH1.isTimerOn() || SCARA.TH2.isTimerOn() || SCARA.TH3.isTimerOn()){
    delay(200);
  }

  SCARA.Move_ManualMode(-20, 0, 0, 0, 50);

  while(1){
    uint32_t start = millis(); // read start time

    // Read any incoming serial command
    readSerialCommand();

    // Read sensors and digital inputs status
  

    uint32_t finish = millis();  // read finish time
    //Calculate cycle time duration
    uint32_t duration = finish - start;
    /*To obtain a fixed sample time, the cycle time duration is subtracted   
    from the sample time, and the difference is added to the code as a delay*/
    uint32_t t_delay = sample_time - duration;
    if (t_delay > 0) {delay(t_delay);}
  }

}



/** Split incoming serial command and extract command code and command arguments
 * @param ptr pointer to incoming serial data
 * @param delimiter1 delimiter character between command code and command parameters
 * @param delimiter2 delimiter character between each command argument
 * @param cmdCode pointer to command code character
 * @param cmdArgs pointer to array of command arguments
 * @return number of command arguments
*/
uint8_t splitCommand(char *ptr, const char *delimiter1, const char *delimiter2, char *cmdCode , float *cmdArgs){ //   
  uint8_t argCount = 0;
  *cmdCode = '\0';

  // separate first char from arguments
  char * token = strtok (ptr,delimiter1);
  if (token != NULL){
    size_t len = strlen(token);
    if (len != 1){
      printf("wrong first char input\r\n");
      return 0;
    }     
    //printf ("%s\n",token);
    //printf("f_ch length: %d\n", strlen(token));
    *cmdCode = ptr[0];
  }
  else{
    return 0;
  }

  // split individual arguments
  token = strtok (NULL,delimiter2); 
  while (token != NULL)
  {
    cmdArgs[argCount] = strtof(token, NULL);
    argCount++;
    //printf ("%s\n",token);
    token = strtok (NULL, ";");
  }  
  return argCount;
}

/** Takes command code and arguments and executes coresponding StepperControl action
 * @param cmdCode pointer to Command Code character
 * @param cmdArgs pointer to Command Arguments array
 * @param argCount number of arguments
 * @return true is action is executed successfully
*/
bool executeCommand(const char *cmdCode, float *cmdArgs, uint8_t argCount){ // size_t argsSize
  switch(*cmdCode){
    case 'A':
      //ambientLightControl((int)sub_command[0],(bool)sub_command[1]);
      break;
    // CLOSE VALVES (VACUUM ON)
    case 'C': //C
      break;
    // DESCEND FOR PICKUP    
    case 'D': //D:vel;lowest_z;ref_pressure;mode;acc_dist;   mode = 1 for pickup / mode = 2 for jig load
      if(argCount < 4 || SCARA.controlMode != STP || !SCARA.isHomed())
        return false;
      if(argCount == 4)
        SCARA.setupDescendModeTrajectory(cmdArgs[1], cmdArgs[0], cmdArgs[2], (uint8_t)cmdArgs[3]);  
      else if(argCount == 5)
        SCARA.setupDescendModeTrajectory(cmdArgs[1], cmdArgs[0], cmdArgs[2], (uint8_t)cmdArgs[3], cmdArgs[4]);  
      break;
    case 'E': //C
      break;
    case 'F': //C
      break;
    // STEP INCREMENT MOTION  
    case 'G':  //G:step_size;axis;time;  //move a small increment in one axis
      if(argCount < 3 || SCARA.controlMode != STP || !SCARA.isHomed())
        return false;
      SCARA.Move_TrajectoryMode_SingleAxis(cmdArgs[0], (uint8_t)cmdArgs[1], cmdArgs[2]);
      // else{
      //   PositionXYZR currPos = SCARA.GetCurrentPosition();
      //   if((int8_t)cmdArgs[1]==0)      
      //     SCARA.Move_TrajectoryMode(currPos.X, currPos.Y, currPos.Z + cmdArgs[0], currPos.R, 0, cmdArgs[2]);
      //   else if((int8_t)cmdArgs[1]==1)      
      //     SCARA.Move_TrajectoryMode(currPos.X + cmdArgs[0], currPos.Y, currPos.Z, currPos.R, cmdArgs[2], 0);      
      //   else if((int8_t)cmdArgs[1]==2)      
      //     SCARA.Move_TrajectoryMode(currPos.X, currPos.Y + cmdArgs[0], currPos.Z, currPos.R, cmdArgs[2], 0);  
      //   else if((int8_t)cmdArgs[1]==3)      
      //     SCARA.Move_TrajectoryMode(currPos.X, currPos.Y, currPos.Z, currPos.R + cmdArgs[0], cmdArgs[2], 0);  
      // }      
      break; 
    case 'H': //C
      break;   
    case 'I': //C
      break;
    case 'J': //C
      break;
    case 'L': //C
      break;
    // MANUAL MODE CONTROL   
    case 'M': //'M:Z;TH1;TH2;TH3;Freq;' commands are used to move a specific number of steps
      if(argCount < 5 || SCARA.controlMode != STP || !SCARA.isHomed())
        return false;
      SCARA.Move_ManualMode((int32_t)cmdArgs[0], (int32_t)cmdArgs[1], (int32_t)cmdArgs[2], (int32_t)cmdArgs[3], (int32_t)cmdArgs[4]);      
      break;
    case 'O': //C
      break; 
    case 'R': //C
      break;                                 
    case 'S': //C
      break;
    // TRAJECTORY MODE CONTROL   
    case 'T': //T:xxx;yyy;zzz;rrr;t;t_z;
      if(argCount < 6 || SCARA.controlMode != STP)
        return false;  
      SCARA.Move_TrajectoryMode(cmdArgs[0], cmdArgs[1], cmdArgs[2], cmdArgs[3], cmdArgs[4], cmdArgs[5]); 
         
      break;
    case 'V': //C
      break;

    case 'Z': //C
      break;
        }
  return true;
}

void readSerialCommand(){
    // check if data is available
  if (Serial.available()) {
    // read the incoming bytes:
    u_int8_t rlen = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
    if(rlen > 0){
      uint8_t argsNum = splitCommand(buf, ":", ";", &firstChar , args); //   
      executeCommand(&firstChar, args, argsNum); //MAX_ARGS
      // Clear whole buffer
      memset(buf, 0, BUFFER_SIZE);
      // Initialise arguments array
      for (int i = 0; i < MAX_ARGS; i++)
        args[i]=-999;
    }
  }  
}
