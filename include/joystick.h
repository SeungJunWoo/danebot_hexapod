#pragma once

////////////////joystick headers
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"
///////////////joystick headers end

#include <math.h>
#include <cmath>

using namespace std;

namespace{
  int joy_fd(-1), num_of_axis(0), num_of_buttons(0);
  char name_of_joystick[80];
  vector<char> joy_button;
  vector<int> joy_axis;
}

struct joystruct
{
    int Lx, Ly, Rx, Ry;
    int L1, R1;
    int BottonX, BottonO, BottonTriangle, BottonSquare;
};

int joystick_setup();
joystruct joystick_loop();
void joystick_close();
int joystick_direction(joystruct JS);
int modeAssign(joystruct JS, bool pressedSet[6], int& shiftOn, bool &motionToggle);
