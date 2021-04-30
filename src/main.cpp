#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <wiringPi.h>

#include <unistd.h> //usleep

#include "leg.h"
#include "base.h"
#include "hexapod.h"
#include "joystick.h"
#include "movement.h"
#include "trajectory.h"
#include "motor.h"

#define REACT_DELAY 5
#define ros_on false

using namespace hexapod;

int main(int argc, char **argv) // without joystrick [[[sudo chmod a+rw /dev/ttyUSB0]]]
{
  std::cout << "main start" << std::endl;
  int joystick_connection = joystick_setup();

  ///////////////////////////////dxl setting/////////////////
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL; // Communication result

  uint8_t dxl_error = 0;            // Dynamixel error
  int32_t dxl_present_position = 0; // Present position

  bool dxl_addparam_result = false; // addParam result
  bool dxl_getdata_result = false;  // GetParam result
  uint8_t param_goal_position[4];

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return 0;
  }
  ////////////////////////////////////////////////////////

  HexapodClass hexapod;
  hexapod.initialized = false;
  float AllAngles[18];

  bool flag = hexapod.init(portHandler, packetHandler); //set port
  if (!flag)
  {
    return 0;
  }

  int mode = 0; //0:standby,1:shift,2:rotating
  int shiftOn = true;
  bool motionToggle = false;
  bool L1Pressed = false;
  bool PressedSet[6]; //x o tri sqr L1 R1
  for (int i = 0; i < 6; i++)
  {
    PressedSet[i] = false;
  }

  while (joystick_connection)
  {

    auto t0 = millis();
    joystruct JS = joystick_loop();
    if (!hexapod.initialized)
    {
      mode = modeAssign(JS, PressedSet, shiftOn, motionToggle);
    }

    Locations location;

    if (mode == 1 || mode == 2 || mode == 3 || mode == 0)
    {
      hexapod.processMovement(portHandler, packetHandler, mode, REACT_DELAY, JS, AllAngles, ros_on);
    }
    else if (mode == 11 || mode == 12)
    {
      if (mode == 11)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_2, AllAngles, ros_on);
      }
      else if (mode == 12)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_1, AllAngles, ros_on);
      }
      else if (mode == 13)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_3, AllAngles, ros_on);
      }
      else if (mode == 14)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_4, AllAngles, ros_on);
      }
      else if (mode == 15)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_5, AllAngles, ros_on);
      }
      else if (mode == 16)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_6, AllAngles, ros_on);
      }
      else if (mode == 17)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_7, AllAngles, ros_on);
      }
      else if (mode == 18)
      {
        hexapod.scenarioMovement(portHandler, packetHandler, Gesture_8, AllAngles, ros_on);
      }
    }

    auto spent = millis() - t0;

    if (spent < REACT_DELAY)
    {
      usleep((REACT_DELAY - spent) * 1000);
    }
    else
    {
    }
  }

  joystick_close();

  return 0;
}

// int main(int argc, char **argv){
//   for (int i =0;i<20;i++){
//     shiftingStep_generator(i);
//   }
// }