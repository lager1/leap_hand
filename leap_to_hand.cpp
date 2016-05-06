/*
 *
 * Author: lager
 *
 * This program is intended to be controlling robotic arm Lynxmotion AL5D through the leap motion sensor
 * This program must be run as user root! 
 *
 */
/* ----------------------------------------------------------------------------------------- */
/*
 * ranges of individual servos:
 * 0: 550  - 2500  | 1500 | BASE       
 * 1: 1000 - 2000  | 1600 | SHOULDER    
 * 2: 800  - 1900  | 1400 | ELBOW      
 * 3: 600  - 2500  | 1500 | WRIST      
 * 4: 500  - 2350  | 1350 | WRIST_ROTATE
 * 5: 800  - 1900  | 1400 | GRIPPER     
 *
 */
/* ----------------------------------------------------------------------------------------- */
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>    // POSIX terminal control definitions
#include <cstdio>
#include <cstring>
#include "Leap.h"
/* ----------------------------------------------------------------------------------------- */
using namespace Leap;
/* ----------------------------------------------------------------------------------------- */
typedef enum {
  X,
  Y,
  Z
} dimension;
/* ----------------------------------------------------------------------------------------- */
typedef enum {
  MIN,
  CENTER,
  MAX,
  RANGE_SIZE    // just to know the size of enum
} range;
/* ----------------------------------------------------------------------------------------- */
typedef enum {
  BASE,         // 0
  SHOULDER,     // 1
  ELBOW,        // 2
  WRIST,        // 3
  WRIST_ROTATE, // 4
  GRIPPER       // 5
} servos;
/* ----------------------------------------------------------------------------------------- */
class Robot_arm {
  public:
    Robot_arm();
    ~Robot_arm();
    void move(int * new_pos);                               // move the arm
    int check_ranges(const int * ranges, int position);
    void set_to_mid();
    int init();
    void send_cmd(const char * cmd);                        // send command
    void set_outfile(int fd);                               // set output file
    static const int num_servos = 6;                        // number of servos
    int * get_pos();                                        // return current position
    static const int ranges[num_servos][RANGE_SIZE];        // min and max ranges of all servos
    // servo move constants
    static const int gripper_offset = 220;                  // offset which equals whole range of motion for gripper
                                                            // divided by 5 fingers
    static constexpr double elbow_offset = 2.2;             // whole range is 1100
                                                            // scanned range max is roughly 530
                                                            // scanned range min is roughly 30
    static const int base_offset = 850;                     // whole range is 1950
                                                            // scanned range max is 1.0
                                                            // scanned range min is 1.0
    static const int wrist_rotate_offset = 10;              // whole range is 1850
                                                            // scanned range max is roughly 90.0
                                                            // scanned range min is roughly -100.0


    static constexpr double wrist_offset = 6.5;             // whole range is 1900
                                                            // scanned range max is roughly 130.0
                                                            // scanned range min is roughly -170.0


    static const int shoulder_offset = 2;                   // whole range is 1000
                                                            // scanned range max is roughly 420
                                                            // scanned range min is roughly 150


  private:
    int USB;                                                // USB file descriptor
    static const int bufsize = 256;                         // command buffer size
    int position[num_servos];                               // current position of aech servo
    unsigned char cmd[bufsize];                             // command to send to robotic arm
    unsigned char end[20];                                   // end of each command
    int outfile;                                            // output file
};
/* ----------------------------------------------------------------------------------------- */
const int Robot_arm::ranges[Robot_arm::num_servos][RANGE_SIZE] = {  // MIN, CENTER, MAX
  {550,  1500, 2500},                                         // BASE
  {1000, 1600, 2000},                                         // SHOULDER
  {800,  1400, 1900},                                         // ELBOW
  {600,  1500, 2500},                                         // WRIST
  {500,  1350, 2350},                                         // WRIST_ROTATE
  {800,  1400, 1900},                                         // GRIPPER
};
/* ----------------------------------------------------------------------------------------- */
void Robot_arm::set_to_mid()
{
  // set all servos to mid position
  unsigned char cmd[] = "#0P1500S200#1P1600S200#2P1400S200#3P1500S200#4P1450S200#5P1400S250\r";     // set to mid slowly
  write(USB, cmd, strlen((char *)cmd));
}
/* ----------------------------------------------------------------------------------------- */
Robot_arm::Robot_arm()
:
  position {     // "center" position for each servo
    1500,
    1600,
    1400,
    1500,
    1350,
    1400  
  },
  end {'T', '2', '0', '0', '\r', '\0'},     // 200 ms to complete whole move
  outfile(0)
{
  memset(cmd, 0, bufsize);
}
/* ----------------------------------------------------------------------------------------- */
Robot_arm::~Robot_arm()
{ 
  close(USB); 

  if(outfile)
    close(outfile);
}
/* ----------------------------------------------------------------------------------------- */
int Robot_arm::init()
{
  /* Open File Descriptor */
  USB = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK | O_NDELAY);

  // Error Handling
  if(USB < 0) {
    std::cerr << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << std::endl;
    return 1;
  }

  // Configure Port
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Error Handling
  if(tcgetattr(USB, &tty) != 0) {
    std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    return 1;
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, B9600);
  cfsetispeed (&tty, B9600);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;
  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_lflag     =   0;                  // no signaling chars, no echo, no canonical processing
  tty.c_oflag     =   0;                  // no remapping, no delays
  tty.c_cc[VMIN]  =   0;                  // read doesn't block
  tty.c_cc[VTIME] =   5;                  // 0.5 seconds read timeout

  tty.c_cflag     |=  CREAD | CLOCAL;                  // turn on READ & ignore ctrl lines
  tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);         // turn off s/w flow ctrl
  tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  tty.c_oflag     &=  ~OPOST;                          // make raw

  /* Flush Port, then applies attributes */
  tcflush(USB, TCIFLUSH);

  if(tcsetattr(USB, TCSANOW, &tty) != 0) {
    std::cerr << "Error " << errno << " from tcsetattr" << std::endl;
    return 1;
  }

  set_to_mid();

  return 0;
}
/* ----------------------------------------------------------------------------------------- */
int Robot_arm::check_ranges(const int * ranges, int position)
{
  if(position > ranges[MAX] || position < ranges[MIN])
    return 1;     // position of servo i is minimal or maximal

  return 0;     // no error occured
}
/* ----------------------------------------------------------------------------------------- */
void Robot_arm::move(int * new_pos)
{
  int i;

  for(i = 0; i < num_servos; i++) {
    if(position[i] != new_pos[i]) {       // detected move change
      if(check_ranges(ranges[i], new_pos[i]) == 0) {  // moved in safe ranges
        position[i] = new_pos[i];         // save the new position
      }

      sprintf((char *)(cmd + (strlen((char *)cmd))), "#%dP%d", i, position[i]);
    }
  }

  if(strlen((char *)cmd) > 0) {               // command is not empty
    strcat((char *)cmd, (char *)end);         // append end of the string to command
    write(USB, cmd, strlen((char *)cmd));     // send command to hand

    if(outfile)     // write to outfile if open
      write(outfile, cmd, strlen((char *)cmd));
  }

  memset(cmd, 0, bufsize);                  // clear buffer
}
/* ----------------------------------------------------------------------------------------- */
int * Robot_arm::get_pos()
{
  return position;  
}
/* ----------------------------------------------------------------------------------------- */
void Robot_arm::send_cmd(const char * cmd)
{
  write(USB, cmd, strlen((char *)cmd));
}
/* ----------------------------------------------------------------------------------------- */
void Robot_arm::set_outfile(int fd)
{
  outfile = fd; 
}
/* ----------------------------------------------------------------------------------------- */
class CListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
    void set_robot_output(int fd);
    void set_filter(double filter);
  private:
    Robot_arm rarm;
    int new_pos[Robot_arm::num_servos];
    const int timeout = 40000;
    const int filter_const = 100;
    double filter;
};
/* ----------------------------------------------------------------------------------------- */
void CListener::onInit(const Controller& controller) {
  int i, * tmp = rarm.get_pos();
  for(i = 0; i < Robot_arm::num_servos; i++)
    new_pos[i] = tmp[i];        // fill new position by current one
  
  if(rarm.init()) {
    std::cerr << "Failed to initialize usb connection to robotic hand" << std::endl;
    exit(1);
  }
  
  std::cout << "Initialized" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onConnect(const Controller& controller) {
  std::cout << "Leap Motion sensor Connected" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onDisconnect(const Controller& controller) {
  std::cout << "Leap Motion sensor Disconnected" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onFrame(const Controller& controller) {
  usleep(timeout);        // timeout to move the arm close to real time

  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();

  if(frame.hands().count() == 0)    // do nothing if no hand is involved
    return;

  if(frame.hands().count() > 1) {     // do nothing if more than one hand is involved
    //std::cout << "More than one hand detected, returning to center position" << std::endl;
    //rarm.set_to_mid();
    return;
  }
  
  // ------------------------ MAIN LOGIC
  // ------------------------
  // GRIPPER
  new_pos[GRIPPER] = Robot_arm::ranges[GRIPPER][MAX] - frame.fingers().extended().count() * Robot_arm::gripper_offset;  // move GRIPPER in safe ranges
  //std::cout <<  "new_pos[GRIPPER]: " << new_pos[GRIPPER] << std::endl;

  HandList hands = frame.hands();
  for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    // Get the first hand
    const Hand hand = *hl;
    Arm arm = hand.arm();
    const Vector direction = hand.direction();
    const Vector normal = hand.palmNormal();

    // ------------------------------
    // ELBOW
    new_pos[ELBOW] = Robot_arm::ranges[ELBOW][MAX] - int(Robot_arm::elbow_offset * hand.palmPosition()[Y]);
    //std::cout <<  "new_pos[ELBOW]: " << new_pos[ELBOW] << std::endl;


    // ------------------------------
    // BASE
    if(arm.direction()[X] < 0)
      new_pos[BASE] = Robot_arm::ranges[BASE][CENTER] - int(Robot_arm::base_offset * fabs(arm.direction()[X]));
    else
      new_pos[BASE] = Robot_arm::ranges[BASE][CENTER] + int(Robot_arm::base_offset * fabs(arm.direction()[X]));
    //std::cout <<  "new_pos[BASE]: " << new_pos[BASE] << std::endl;
 

    // ------------------------------
    // WRIST ROTATE
    if(direction.pitch() * RAD_TO_DEG < 0)
      new_pos[WRIST_ROTATE] = Robot_arm::ranges[WRIST_ROTATE][CENTER] - int(Robot_arm::wrist_rotate_offset * fabs(direction.pitch() * RAD_TO_DEG));
    else
      new_pos[WRIST_ROTATE] = Robot_arm::ranges[WRIST_ROTATE][CENTER] + int(Robot_arm::wrist_rotate_offset * fabs(direction.pitch() * RAD_TO_DEG));
    //std::cout <<  "new_pos[WRIST_ROTATE]: " << new_pos[WRIST_ROTATE] << std::endl;


    // ------------------------------
    // WRIST
    if(normal.roll() * RAD_TO_DEG < 0)
      new_pos[WRIST] = Robot_arm::ranges[WRIST][CENTER] + int(Robot_arm::wrist_offset * fabs(normal.roll() * RAD_TO_DEG));
    else
      new_pos[WRIST] = Robot_arm::ranges[WRIST][CENTER] - int(Robot_arm::wrist_offset * fabs(normal.roll() * RAD_TO_DEG));
    //std::cout <<  "new_pos[WRIST]: " << new_pos[WRIST] << std::endl;


    // ------------------------------
    // SHOULDER
    new_pos[SHOULDER] = Robot_arm::ranges[SHOULDER][MIN] + int(Robot_arm::shoulder_offset * arm.elbowPosition()[Z]);
    //std::cout <<  "new_pos[SHOULDER]: " << new_pos[SHOULDER] << std::endl;
  }

  // filtration
  if(filter != 0) {
    int * tmp_pos = rarm.get_pos();

    for(int i = 0; i < Robot_arm::num_servos; i++) {
      if(fabs((tmp_pos[i] - new_pos[i]) / (double)filter_const) > filter) {      // move only if greater than filter
        rarm.move(new_pos);
        break;
      }
    }
    return;
  }

  rarm.move(new_pos);

}
/* ----------------------------------------------------------------------------------------- */
void CListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onDeviceChange(const Controller& controller) {
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onServiceConnect(const Controller& controller) {
}
/* ----------------------------------------------------------------------------------------- */
void CListener::onServiceDisconnect(const Controller& controller) {
}
/* ----------------------------------------------------------------------------------------- */
void CListener::set_robot_output(int fd)
{
  rarm.set_outfile(fd);
}
/* ----------------------------------------------------------------------------------------- */
void CListener::set_filter(double f)
{
  filter = f;
}
/* ----------------------------------------------------------------------------------------- */
void print_help()
{
  std::cout << "leap_to_hand [-h] [-o output_file] [-i input_file] [-f filter_rate ]" << std::endl;
  std::cout << "filter rate is decimal number from 0 to 1" << std::endl;
}
/* ----------------------------------------------------------------------------------------- */
int check_file(int * infile, const char * name)
{
  // check file exists and is readable
  if(access(name, F_OK | R_OK) != 0) {
    std::cerr << "Error " << errno << " from access: " << strerror(errno) << std::endl;
    return 1;
  }

  *infile = open(name, O_RDWR);
 
  if(*infile == -1) {
    std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
    return 1;  
  }

  return 0;
}
/* ----------------------------------------------------------------------------------------- */
int create_file(int * outfile, const char * name)
{
  // check file can be created and written
  *outfile = open(name, O_RDWR | O_CREAT | O_TRUNC,  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

  if(*outfile == -1) {
    std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
    return 1;  
  }

  return 0;
}
/* ----------------------------------------------------------------------------------------- */
void parse_argv(int argc, char ** argv, int * infile, int * outfile, double * filter)
{
  int c;

  while ((c = getopt (argc, argv, "ho:i:f:")) != -1)
    switch (c) {
      case 'h':
        print_help();
        return;
      case 'o':
        if(create_file(outfile, optarg))
          exit(1);
        return;
      case 'i':
        if(check_file(infile, optarg))
          exit(1);
        return;
      case 'f':
        *filter = strtod(optarg, NULL);

        if(*filter < 0 || *filter > 1) {
          std::cout << "filter value out of range" << std::endl;
          print_help();
          exit(1);
        }
        break;
      default:
        print_help();
        exit(1);
    }
}
/* ----------------------------------------------------------------------------------------- */
void play(int infile)
{
  Robot_arm rarm;
  size_t bufsize = 256;
  ssize_t ret = 0;
  char * cmd = NULL;
  FILE * in = fdopen(infile, "r");

  if(rarm.init()) {
    std::cerr << "Failed to initialize usb connection to robotic hand" << std::endl;
    exit(1);
  }
  
  rarm.set_to_mid();
  sleep(3);

  ret = getdelim(&cmd, &bufsize, '\r', in);

  while(ret != EOF) {
    rarm.send_cmd(cmd);
    ret = getdelim(&cmd, &bufsize, '\r', in);
  }

  fclose(in);
  close(infile);
}
/* ----------------------------------------------------------------------------------------- */
int main(int argc, char ** argv)
{
  int infile = 0, outfile = 0;
  double filter = 0;
  parse_argv(argc, argv, &infile, &outfile, &filter);

  if(infile != 0) {
    play(infile);
    return 0;
  }
 
  // Create a sample listener and controller
  CListener listener;
  Controller controller;
 
  if(outfile != 0)
    listener.set_robot_output(outfile);

  listener.set_filter(filter);

  // Have the sample listener receive events from the controller
  controller.addListener(listener);

  // Keep this process running until Enter is pressed
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}



