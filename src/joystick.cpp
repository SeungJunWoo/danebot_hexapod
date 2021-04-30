#include "joystick.h"

int joystick_setup()
{

  if ((joy_fd = open(JOY_DEV, O_RDONLY)) < 0)
  {
    cerr << "Failed to open " << JOY_DEV << endl;
    return 0;
  }

  ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

  joy_button.resize(num_of_buttons, 0);
  joy_axis.resize(num_of_axis, 0);

  cout << "Joystick: " << name_of_joystick << endl
       << "  axis: " << num_of_axis << endl
       << "  buttons: " << num_of_buttons << endl;

  fcntl(joy_fd, F_SETFL, O_NONBLOCK); // using non-blocking mode
  return 1;
}

joystruct joystick_loop()
{
  joystruct joystruct;
  js_event js;

  read(joy_fd, &js, sizeof(js_event));

  switch (js.type & ~JS_EVENT_INIT)
  {
  case JS_EVENT_AXIS:
    if ((int)js.number >= joy_axis.size())
    {
      cerr << "err:" << (int)js.number << endl;
    }
    joy_axis[(int)js.number] = js.value;
    break;
  case JS_EVENT_BUTTON:
    if ((int)js.number >= joy_button.size())
    {
      cerr << "err:" << (int)js.number << endl;
    }
    joy_button[(int)js.number] = js.value;
    break;
  }

  // cout << "L3_b: " << joy_button[8] << "  L3_h: " << joy_axis[0] << "  L3_v: " << joy_axis[1] << endl;
  // cout << "R3_b: " << joy_button[9] << "  R3_h: " << joy_axis[3] << "  R3_v: " << joy_axis[4] << endl;
  // cout << endl;

  joystruct.Lx = joy_axis[0];
  joystruct.Ly = joy_axis[1];
  joystruct.Rx = joy_axis[3];
  joystruct.Ry = joy_axis[4];

  joystruct.BottonX = (int)joy_button[0];
  joystruct.BottonO = (int)joy_button[1];
  joystruct.BottonTriangle = (int)joy_button[2];
  joystruct.BottonSquare = (int)joy_button[3];
  joystruct.L1 = (int)joy_button[4];
  joystruct.R1 = (int)joy_button[5];

  return joystruct;
}

void joystick_close()
{
  close(joy_fd);
}

int joystick_direction(joystruct JS){
  float rad= atan2(-JS.Ly, JS.Lx);
  int direction;
  // cout<<"rad: "<<rad<<endl;

  if ((rad >= M_PI*3.0/8.0) && (rad < M_PI*5.0/8.0)){
    direction=0;
  }
  else if ((rad >= M_PI*5.0/8.0) && (rad < M_PI*7.0/8.0)){
    direction=1;//
  }
  else if ((rad >= -M_PI*7.0/8.0) && (rad < -M_PI*5.0/8.0)){
    direction=3;
  }
  else if ((rad >= -M_PI*5.0/8.0) && (rad < -M_PI*3.0/8.0)){
    direction=4;
  }
  else if ((rad >= -M_PI*3.0/8.0) && (rad < -M_PI*1.0/8.0)){
    direction=5;
  }
  else if ((rad >= -M_PI*1.0/8.0) && (rad < M_PI*1.0/8.0)){
    direction=6;
  }
  else if ((rad >= M_PI*1.0/8.0) && (rad < M_PI*3.0/8.0)){
    direction=7;
  }
  else{
    direction=2;
  }
  // cout<<"direction: "<<direction<<endl;

  return direction;
}

int modeAssign(joystruct JS, bool pressedSet[6], int &shiftOn, bool &motionToggle)
{
  int mode;

  //Toggle//
  if (JS.L1)
  {
    pressedSet[4] = true;//L1
  }
  else if (!JS.L1)
  {
    if (pressedSet[4] == true) //pressed->unpressed
    {
      if (shiftOn == 1)
      {
        shiftOn = 2;
      }
      else
      {
        shiftOn = 1;
      }
      
    }
    pressedSet[4] = false;
  }

  if (JS.R1)
  {
    pressedSet[5] = true;//R1
  }
  else if (!JS.R1)
  {
    if (pressedSet[5] == true) //pressed->unpressed
    {
      if (motionToggle == true)
      {
        motionToggle = false;
      }
      else
      {
        motionToggle = true;
      }
    }
    pressedSet[5] = false;
  }

  //walking mode//
  if (JS.Lx == 0 && JS.Ly == 0)
  {
    mode = 0;
  }
  else
  {
    if (shiftOn == 1)
    {
      mode = 1;
    }
    else
    {
      mode = 2;
    }
    
  }

  //motion//
  if (mode==0 && JS.BottonX)
  {
    pressedSet[0] = true;//x
  }
  else if (!JS.BottonX)
  {
    if (pressedSet[0] == true) //pressed->unpressed
    {
      if (motionToggle)
      {
        mode =15;
      }
      else
      {
        mode = 11;
      }
    }
    pressedSet[0] = false;
  }

  if (mode==0 && JS.BottonO)
  {
    pressedSet[1] = true;//o
  }
  else if (!JS.BottonO)
  {
    if (pressedSet[1] == true) //pressed->unpressed
    {
      if (motionToggle)
      {
        mode =16;
      }
      else
      {
        mode = 12;
      }
    }
    pressedSet[1] = false;
  }

  if (mode==0 && JS.BottonTriangle)
  {
    pressedSet[2] = true;//tri
  }
  else if (!JS.BottonTriangle)
  {
    if (pressedSet[2] == true) //pressed->unpressed
    {
      if (motionToggle)
      {
        mode =17;
      }
      else
      {
        mode = 13;
      }
    }
    pressedSet[2] = false;
  }

  if (mode==0 && JS.BottonSquare)
  {
    pressedSet[3] = true;//sqr
  }
  else if (!JS.BottonSquare)
  {
    if (pressedSet[3] == true) //pressed->unpressed
    {
      if (motionToggle)
      {
        mode =18;
      }
      else
      {
        mode = 14;
      }
    }
    pressedSet[3] = false;
  }

  return mode;
}
