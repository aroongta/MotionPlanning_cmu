//
//  robotAnimation.hpp
//  motion_planning_sim
//
//  Created by Bryan Zhao on 11/20/18.
//  Copyright © 2018 Bryan Zhao. All rights reserved.
//

#ifndef robotAnimation_h
#define robotAnimation_h

#include <stdio.h>
#include "fssimplewindow.h"
#include "math.h"
#include <string>
#include "ysglfontdata.h"
#include <iostream>
#include <vector>

using namespace std;

class Robot{
public:
    Robot(); // default constructor
    int pollDevice(int input); // polls device from outside world
    bool isPaused; // allows user to pause path
    bool isMoving; // animation state
    void drawRobot(int centerX, int centerY); // draws robot in window
    void drawRotor(int cx, int cy, int rad); // draws rotors of quad
    void drawCircle(int cx, int cy, int rad, bool fill);
    double theta; // angle of rotor blades
    void dispMenu(int locX, int locY, int width, int height);
    void displayText(const string message, int width, int height); // displays text on screen
    void interpolatePath(vector<double> xCoords, vector<double> yCoords);
    int currAnimatedX; // current location for animation
    int currAnimatedY;
};
#endif /* robotAnimation_h */
