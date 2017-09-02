# leap\_hand
Simple program to move robotic hand Lynxmotion AL5D with human hands through Leap Motion sensor.

### downloading and conmpilation
C++11 compatible compiler (g++ 4.9.2 or newer) is needed to compile this project.
Download and compile by:
```
git clone https://github.com/lager1/leap_hand.git
cd leap_hand
make
```
### usage
Program needs to be run with superuser permissions.
For correct usage, Leap motioon sensor and robotic hand should be connected to your computer, afterwards the program could be run.
After launching the robotic hand is moved to central position and the sensor detects hand motion
until the enter key is pressed.
Program has built-in help which can be displayed by -h option.

# demo
![demo](https://github.com/lager1/leap_hand/blob/master/demo.gif "demo")

# demo on youtube
[demo](https://youtu.be/rlCg_acRVgg)
