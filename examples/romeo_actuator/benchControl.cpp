#include <iostream>
#include <fstream>
#include <stdint.h>
#include <unistd.h>

#include <math.h>

#include "ddpsolver.hh"
#include "romeosimpleactuator.hh"
#include "costfunctionromeoactuator.hh"

#include <fcntl.h>
#include <termios.h>

#include <sys/time.h>


using namespace std;
using namespace Eigen;

struct timeval current_time, last_time;

uint8_t crc, send, received;
uint16_t posTarget;
uint8_t requiredJointposH;
uint8_t requiredJointposL;
uint8_t requiredSpeedH;
uint8_t requiredSpeedL;
uint8_t requiredCurrentH;
uint8_t requiredCurrentL;
uint8_t requiredPwmH;
uint8_t requiredPwmL;
uint8_t motorH;
uint8_t motorL;
int8_t motorTurn;
uint8_t jointH;
uint8_t jointL;
uint8_t currentH;
uint8_t currentL;
uint8_t pwmH;
uint8_t pwmL;
uint8_t crc1;
uint8_t crc2;
uint8_t crcCalc1;
uint8_t crcCalc2;

int32_t motorPosInt;
uint16_t jointPosInt;

double motorPos;
double jointPos;
double motorPosOld;
double jointPosOld;
double motorVel;
double jointVel;
double diffAngle;
double diff_time;

uint8_t crc8(uint8_t crc, uint8_t crc_data)
{
  uint8_t i;
  i = (crc_data ^ crc) & 0xff;
  crc = 0;
  if (i & 1)
    crc ^= 0x5e;
  if (i & 2)
    crc ^= 0xbc;
  if (i & 4)
    crc ^= 0x61;
  if (i & 8)
    crc ^= 0xc2;
  if (i & 0x10)
    crc ^= 0x9d;
  if (i & 0x20)
    crc ^= 0x23;
  if (i & 0x40)
    crc ^= 0x46;
  if (i & 0x80)
    crc ^= 0x8c;
  return (crc);
}


DDPSolver<double, 4, 1>::stateVec_t getStateFromSerial(int& ser)
{
  DDPSolver<double, 4, 1>::stateVec_t x;

  crc = 0;
  send = 0x47;
  write(ser, &send, 1);
  send = 0x06;
  crc = crc8(crc, send);
  write(ser, &send, 1);

  posTarget = 1 << 15; // betwen 650 and 1650

  send = (posTarget >> 8) & 0xFF;
  crc = crc8(crc, send);
  write(ser, &send, 1);
  send = posTarget & 0xFF;
  crc = crc8(crc, send);
  write(ser, &send, 1);
  send = crc;
  write(ser, &send, 1);

  last_time = current_time;
  gettimeofday(&current_time, NULL);
  diff_time = ((current_time.tv_sec - last_time.tv_sec) +
               (current_time.tv_usec - last_time.tv_usec) / 1000000.0);

  do
  {
    read(ser, &received, 1);
  }
  while (received != 0x47);

  read(ser, &requiredJointposH, 1);
  read(ser, &requiredJointposL, 1);
  read(ser, &requiredSpeedH, 1);
  read(ser, &requiredSpeedL, 1);
  read(ser, &requiredCurrentH, 1);
  read(ser, &requiredCurrentL, 1);
  read(ser, &requiredPwmH, 1);
  read(ser, &requiredPwmL, 1);


  read(ser, &motorH, 1);
  read(ser, &motorL, 1);
  read(ser, &motorTurn, 1);
  read(ser, &jointH, 1);
  read(ser, &jointL, 1);
  read(ser, &currentH, 1);
  read(ser, &currentL, 1);
  read(ser, &pwmH, 1);
  read(ser, &pwmL, 1);
  read(ser, &crc1, 1);
  read(ser, &crc2, 1);

  crcCalc1 = 0;
  crcCalc1 = crc8(crcCalc1, motorH);
  crcCalc1 = crc8(crcCalc1, motorL);
  crcCalc1 = crc8(crcCalc1, motorTurn);
  crcCalc1 = crc8(crcCalc1, jointH);
  crcCalc1 = crc8(crcCalc1, jointL);

  crcCalc2 = 0;
  crcCalc2 = crc8(crcCalc2, currentH);
  crcCalc2 = crc8(crcCalc2, currentL);
  crcCalc2 = crc8(crcCalc2, pwmH);
  crcCalc2 = crc8(crcCalc2, pwmL);

  motorPosOld = motorPos;
  jointPosOld = jointPos;

  motorPosInt = (4096 * motorTurn) + (motorH << 8) | motorL;
  jointPosInt = (jointH << 8) | jointL;
  motorPos = (double)motorPosInt * M_PI / 2048.0;
  jointPos = (double)jointPosInt * M_PI / 2048.0;
  motorVel = (motorPos - motorPosOld)/*/diff_time*/;
  jointVel = (jointPos - jointPosOld)/*/diff_time*/;
  diffAngle = motorPos - jointPos * 97.68;

  x << jointPos, motorVel, jointVel, diffAngle;

  return x;
}


DDPSolver<double, 4, 1>::stateVec_t sendCurrentCommand(int& ser,
    DDPSolver<double, 4, 1>::commandVec_t u)
{
  DDPSolver<double, 4, 1>::stateVec_t x;
  uint16_t currentDes;
  currentDes = (uint16_t) ((u[0]) * ((4096 * 20 * 0.005) / 3.0) + 2048);

  /*cout << currentDes << endl;
  cout << "-" << endl;*/

  crc = 0;
  send = 0x47;
  write(ser, &send, 1);
  send = 0x04;
  crc = crc8(crc, send);
  write(ser, &send, 1);

  send = (currentDes >> 8) & 0xFF;
  crc = crc8(crc, send);
  write(ser, &send, 1);
  send = currentDes & 0xFF;
  crc = crc8(crc, send);
  write(ser, &send, 1);
  send = crc;
  write(ser, &send, 1);

  last_time = current_time;
  gettimeofday(&current_time, NULL);
  diff_time = ((current_time.tv_sec - last_time.tv_sec) +
               (current_time.tv_usec - last_time.tv_usec) / 1000000.0);

  do
  {
    read(ser, &received, 1);
  }
  while (received != 0x47);

  read(ser, &motorH, 1);
  read(ser, &motorL, 1);
  read(ser, &motorTurn, 1);
  read(ser, &jointH, 1);
  read(ser, &jointL, 1);
  read(ser, &currentH, 1);
  read(ser, &currentL, 1);
  read(ser, &pwmH, 1);
  read(ser, &pwmL, 1);
  read(ser, &crc1, 1);
  read(ser, &crc2, 1);

  crcCalc1 = 0;
  crcCalc1 = crc8(crcCalc1, motorH);
  crcCalc1 = crc8(crcCalc1, motorL);
  crcCalc1 = crc8(crcCalc1, motorTurn);
  crcCalc1 = crc8(crcCalc1, jointH);
  crcCalc1 = crc8(crcCalc1, jointL);

  crcCalc2 = 0;
  crcCalc2 = crc8(crcCalc2, currentH);
  crcCalc2 = crc8(crcCalc2, currentL);
  crcCalc2 = crc8(crcCalc2, pwmH);
  crcCalc2 = crc8(crcCalc2, pwmL);

  motorPosOld = motorPos;
  jointPosOld = jointPos;

  motorPosInt = (4096 * motorTurn) + (motorH << 8) | motorL;
  jointPosInt = (jointH << 8) | jointL;
  motorPos = (double)motorPosInt * M_PI / 2048.0;
  jointPos = (double)jointPosInt * M_PI / 2048.0;
  motorVel = (motorPos - motorPosOld)/*/diff_time*/;
  jointVel = (jointPos - jointPosOld)/*/diff_time*/;
  diffAngle = motorPos - jointPos * 97.68;

  x << jointPos, motorVel, jointVel, diffAngle;

  return x;
}


int main()
{
  struct timeval tbegin, tend;
  double texec = 0.0;

  bool flag = 1;
  int i;

  int ser = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( ser, &tty ) != 0 )
  {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(
                errno) << std::endl;
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B1000000);
  cfsetispeed (&tty, (speed_t)B1000000);

  /* Setting other Port Stuff */
  /*config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                   INLCR | PARMRK | INPCK | ISTRIP | IXON);
  config.c_oflag = 0;*/
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_cc[VMIN]   =  1;                  // read doesn't block
  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( ser, TCIFLUSH );
  if ( tcsetattr ( ser, TCSANOW, &tty ) != 0)
  {
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  ofstream fichier("resultsBench.csv", ios::out | ios::trunc);
  fichier << "tau,tauDot,q,qDot,u" << endl;
  DDPSolver<double, 4, 1>::stateVec_t x, x_offset, xinit, xDes;
  DDPSolver<double, 4, 1>::commandVec_t u;

  cout << "begin" << endl;

  /* init */
  xinit = getStateFromSerial(ser);
  xinit = getStateFromSerial(ser);
  xinit = getStateFromSerial(ser);

  x_offset = getStateFromSerial(ser);
  //x_offset[0] = 0.0;
  x_offset[1] = 0.0;
  //x_offset[2] = 0.0;
  x_offset[3] = 0.0;
  x = getStateFromSerial(ser);

  xinit << x - x_offset;
  xDes << 0.0, 0.0, 0.0, 0.0;

  unsigned int T = 50;
  double dt = 1e-3;
  unsigned int iterMax = 100;
  double stopCrit = 1e-5;
  DDPSolver<double, 4, 1>::stateVecTab_t xList;
  DDPSolver<double, 4, 1>::commandVecTab_t uList;
  DDPSolver<double, 4, 1>::traj lastTraj;

  RomeoSimpleActuator romeoActuatorModel(dt);
  CostFunctionRomeoActuator costRomeoActuator;
  DDPSolver<double, 4, 1> testSolverRomeoActuator(romeoActuatorModel,
      costRomeoActuator, DISABLE_FULLDDP, ENABLE_QPBOX);
  testSolverRomeoActuator.FirstInitSolver(xinit, xDes, T, dt, iterMax, stopCrit);
  while (1)
  {
    gettimeofday(&tbegin, NULL);
    i++;
    if (i == 3000)
    {
      if (flag)
      {
        xDes << 0.0, 0.0, 0.0, 0.0;
        flag = 0;
      }
      else
      {
        xDes << 0.0, 0.0, 0.0, 0.0;
        flag = 1;
      }
      i = 0;
    }
    testSolverRomeoActuator.initSolver(xinit, xDes);
    testSolverRomeoActuator.solveTrajectory();

    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();

    u = lastTraj.uList[0];

    x = sendCurrentCommand(ser, -u);

    /*xList = lastTraj.xList;
    uList = lastTraj.uList;
    unsigned int iter = lastTraj.iter;*/

    xinit = x - x_offset;

    gettimeofday(&tend, NULL);
    texec = ((double) (1000 * (tend.tv_sec - tbegin.tv_sec) + ((
                         tend.tv_usec - tbegin.tv_usec) / 1000))) / 1000.;

    //system("clear");
    //cout << xinit[0] << " \t " << xinit[1] << " \t " << xinit[2] << " \t " << xinit[3] << endl;
    system("clear");
    printf("%f\n", u[0]);
    printf("%4f\t%4f\t%4f\t%4f\n", xinit[0], xinit[1], xinit[2], xinit[3]);

    printf("%f\n", ((tend.tv_sec - tbegin.tv_sec) + (tend.tv_usec - tbegin.tv_usec)
                    / 1000000.0));
  }

  return 0;

}
