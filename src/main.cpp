#include <Arduino.h>
#include "ScaraDue.h"

// Initialise variables
static char firstChar;  // first char received indicates the command code
static const u_int8_t BUFFER_SIZE = 30; // max command length
static char buf[BUFFER_SIZE];   // incoming serial data
static const u_int8_t MAX_ARGS = 9; // max number of arguments for a command
static float args[MAX_ARGS];  // arguments array
bool _serialSend;
//static uint32_t sample_time = 100; // Main loop 'cycle time' (delay at the end of the loop)

bool readSerialCommand();
bool executeCommand(const char *cmdCode, float *cmdArgs, uint8_t argCount);
uint8_t splitCommand(char *ptr, const char *delimiter1, const char *delimiter2, char *cmdCode , float *cmdArgs);

int main(void){

  // put your setup code here, to run once:
  Serial.begin(115200);

  if (!SCARA.Init())
    Serial.println("SCARA INIT FAIL");

  while(1){

    uint32_t start = millis(); // read start time

    // Read any incoming serial command
    bool readCmdOK = readSerialCommand();

    SCARA.MainLoopFunction(_serialSend);

    if(_serialSend){


    }

    uint32_t finish = millis();  // read finish time
    //Calculate cycle time duration
    uint32_t duration = finish - start;
    /*To obtain a fixed sample time, the cycle time duration is subtracted   
    from the sample time, and the difference is added to the code as a delay*/
    int32_t t_delay = SAMPLE_TIME - duration;
    if (t_delay > 0)
      delay((uint32_t)t_delay);

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
      SCARA.RingLEDControl((bool)cmdArgs[0]);
      break;
    // CLOSE VALVES (VACUUM ON)
    case 'C': //C
      SCARA.vacuum_on();
      break;
    // DESCEND FOR PICKUP    
    case 'D': //D:vel;lowest_z;ref_pressure;mode;acc_dist;   mode = 1 for pickup / mode = 2 for jig load
      if(argCount < 4 || SCARA.isMoving() || !SCARA.isHomed())
        return false;
      if(argCount == 4)
        SCARA.Move_ControlledDescendMode(cmdArgs[1], cmdArgs[0], cmdArgs[2], (uint8_t)cmdArgs[3]);  
      else if(argCount == 5)
        SCARA.Move_ControlledDescendMode(cmdArgs[1], cmdArgs[0], cmdArgs[2], (uint8_t)cmdArgs[3], cmdArgs[4]);  
      break;
    // EMERGENCY STOP   
    case 'E': //Stop All Timers
      SCARA.Stop();
      break;
    case 'F': //C




      break;
    // STEP INCREMENT MOTION  
    case 'G':  //G:step_size;axis;time;  //move a small increment in one axis
      if(argCount < 3 || SCARA.isMoving()|| !SCARA.isHomed())
        return false;
      SCARA.Move_TrajectoryMode_SingleAxis(cmdArgs[0], (uint8_t)cmdArgs[1], cmdArgs[2]);    
      break; 
    // SOFTWARE HOME  
    case 'H': // H:xxx;yyy;zzz;rrr; Initialise end-effector position
      if(argCount < 4 || SCARA.isMoving())
        return false;
        SCARA.InitialiseRobotPosition(cmdArgs[0], cmdArgs[1], cmdArgs[2], cmdArgs[3]);
      break;   
    // INITIALISE SERIAL TRANSMISSION   
    case 'I': //I:0/1; 1-> Initialise Serial Transmission, 0->Stop Serial Transmission
      if(cmdArgs[0]>0){
        _serialSend = true;
      }else{
        _serialSend = false;
      }
      break;
    case 'J': //C




      break;
    // BACKLIGHT CONTROL  
    case 'L': //L:brightness;color;
      if(argCount < 3)
        return false;
      SCARA.SetBackLightBrightness((uint8_t)cmdArgs[0], (uint8_t)cmdArgs[1], (uint8_t)cmdArgs[2]);
      //SCARA.SetBackLightBrightness((uint8_t)cmdArgs[0], (uint8_t)cmdArgs[1], (CRGB::HTMLColorCode)cmdArgs[2]);
      break;
    // MANUAL MODE CONTROL   
    case 'M': //'M:Z;TH1;TH2;TH3;Freq;' commands are used to move a specific number of steps
      if(argCount < 5 || SCARA.isMoving())
        return false;
      SCARA.Move_ManualMode((int32_t)cmdArgs[0], (int32_t)cmdArgs[1], (int32_t)cmdArgs[2], (int32_t)cmdArgs[3], (int32_t)cmdArgs[4]);      
      break;
    // OPEN VALVE (Vacuum Off)    
    case 'O': //
      SCARA.vacuum_off();
      break; 
    // RELEASE WITH PUFF - Initialise Sequency  
    case 'R': //R:off-time;puff-time;   // Set puff delay and puff time
      if(argCount < 2)
        return false;
      SCARA.StartReleaseWthPuff((uint8_t)cmdArgs[0], (uint8_t)cmdArgs[1]);
      break;    
    // STOP WITH DECELERATION - Stop Speed Mode Control with Deceleration                               
    case 'S': //C




      break;
    // TRAJECTORY MODE CONTROL   
    case 'T': //T:xxx;yyy;zzz;rrr;t;t_z;
      if(argCount < 6 || SCARA.isMoving() || !SCARA.isHomed())
        return false;  
      SCARA.Move_TrajectoryMode(cmdArgs[0], cmdArgs[1], cmdArgs[2], cmdArgs[3], cmdArgs[4], cmdArgs[5]);      
      break;
    case 'V': //C




      break;
    // HARDWARE HOME - Using limit switches and stepper driver home mode  
    case 'Z': //Z:xxx;yyy;zzz;rrr;
      if(argCount < 4 || SCARA.isMoving())
        return false;    
      SCARA.StartHomingProcedure(cmdArgs[0], cmdArgs[1], cmdArgs[2], cmdArgs[3]);    
      break;
        }
  return true;
}

bool readSerialCommand(){
    // check if data is available
  if (Serial.available()) {
    // read the incoming bytes:
    u_int8_t rlen = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
    if(rlen > 0){
      uint8_t argsNum = splitCommand(buf, ":", ";", &firstChar , args); //   
      if (!executeCommand(&firstChar, args, argsNum)){ //MAX_ARGS
        return false;
      } 
      // Clear whole buffer
      memset(buf, 0, BUFFER_SIZE);
      // Initialise arguments array
      for (int i = 0; i < MAX_ARGS; i++)
        args[i]=-999;
    }
  }  
  return true;
}
