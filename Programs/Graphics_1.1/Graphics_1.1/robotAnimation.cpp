//  robotAnimation.cpp
//  motion_planning_sim
//
//  Created by Bryan Zhao on 11/20/18.

#define PI 3.1415926535
#define DEG2RAD PI/180.0

//#include "robotAnimation.h"
// default constructor
#include "Graphics.h"
Robot::Robot()
{
    //isMoving = false; // tracks animation movement
    isPaused = false;
    theta = 0; // rotation angle of rotors
	currentX = 0;
	currentY = 0;
}

// draw circles
void Robot::drawCircle(int cx, int cy, int rad, bool fill)
{
    glEnable(GL_BLEND); // set up alpha blending, need GL_BLEND
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    int i; // loop var
    double angle, x, y;
    
    if (fill)
        glBegin(GL_POLYGON);
    else
        glBegin(GL_LINE_LOOP);
    
    for (i = 0; i < 64; i++) {
        angle = (double)i * PI / 32.0;
        x = (double)cx + cos(angle)*(double)rad;
        y = (double)cy + sin(angle)*(double)rad;
        glVertex2d(x, y);
    }
    glEnd();
    glDisable(GL_BLEND);
}

// draws an ellipse (if needed)
void drawEllipse(int cx, int cy, float radiusX, float radiusY)
{
    int i; // loop var
    glBegin(GL_POLYGON);
    for(i=0;i<360;i++)
    {
        float rad = i*DEG2RAD;
        glVertex2f(cx + cos(rad)*radiusX, cy + sin(rad)*radiusY);
    }
    glEnd();
}

// draws rotors of quad
void Robot::drawRotor(int cx, int cy, int rad)
{
    drawCircle(cx, cy, rad, 0);
    glBegin(GL_LINES);
    glVertex2i(cx, cy);
    glVertex2i(cx + (rad)*cos(theta * PI / 180), cy - (rad)*sin(theta * PI / 180));
    glVertex2i(cx, cy);
    glVertex2i(cx - (rad)*cos(theta * PI / 180), cy + (rad)*sin(theta * PI / 180));
    glVertex2i(cx, cy);
    glVertex2i(cx + (rad)*cos(theta * PI / 180 + PI/2), cy - (rad)*sin(theta * PI / 180 + PI/2));
    glVertex2i(cx, cy);
    glVertex2i(cx - (rad)*cos(theta * PI / 180 + PI/2), cy + (rad)*sin(theta * PI / 180 + PI/2));
    glVertex2i(cx, cy);
    glVertex2i(cx + (rad)*cos(theta * PI / 180 + PI/4), cy - (rad)*sin(theta * PI / 180 + PI/4));
    glVertex2i(cx, cy);
    glVertex2i(cx - (rad)*cos(theta * PI / 180 + PI/4), cy + (rad)*sin(theta * PI / 180 + PI/4));
    glVertex2i(cx, cy);
    glVertex2i(cx + (rad)*cos(theta * PI / 180 - PI/4), cy - (rad)*sin(theta * PI / 180 - PI/4));
    glVertex2i(cx, cy);
    glVertex2i(cx - (rad)*cos(theta * PI / 180 - PI/4), cy + (rad)*sin(theta * PI / 180 - PI/4));
    glEnd();
}


// draws robot animation
void Robot::drawRobot()
{
	// draw center body
    double x = currentX, y = currentY; // typecast for glVertex2d
    glLineWidth(1.0);
    glColor3ub(90, 90, 90);
    glBegin(GL_QUADS);
    glVertex2d(x-5, y-5);
    glVertex2d(x+5, y-5);
    glVertex2d(x+5, y+5);
    glVertex2d(x-5, y+5);
    glEnd();
    
    // draw rotors of quadcopter
    double rad = 6; // radius of rotor
    int spacing = 10;
    int left_xc = x - spacing, left_yc = y;
    int right_xc = x + spacing, right_yc = y;
    int top_xc = x, top_yc = y - spacing;
    int bot_xc = x, bot_yc = y + spacing;
    
    // left rotor
    glLineWidth(1.5);
    drawRotor(left_xc, left_yc, rad);
    drawRotor(right_xc, right_yc, rad);
    drawRotor(top_xc, top_yc, rad);
    drawRotor(bot_xc, bot_yc, rad);
    
    theta+=6;
}

// function to animate robot
bool Robot::animateRobot(int &i,vector<double> x,vector<double> y)
{
	drawRobot();
	if (currentX < 0 || currentX >= 500 || currentY < 0 || currentY >= 500)
	{//Resetting animation of path changes from start
		i = 0;
		currentX = x[i];
		currentY = y[i];
	}
	if (i == x.size() - 1)	//the robot has reached end goal
		return false;
	if (i < x.size() - 1)
	{
		currentX += (x[i + 1] - x[i]) / 5.0;
		currentY += (y[i + 1] - y[i]) / 5.0;
		if (currentX==x[i+1] && currentY==y[i+1])
			i++;
	}
	return true;
}

// generates a smoother path for the robot to follow during animation by interpolation
void Robot::interpolatePath(vector<double> xPathCoords, vector<double> yPathCoords)
{
    // copy and reverse coordinates (since path coords contain end to start)
    vector<double> reverseXPathCoords (xPathCoords.rbegin(), xPathCoords.rend());
    vector<double> reverseYPathCoords (yPathCoords.rbegin(), yPathCoords.rend());
    vector<double> animateXCoords;
    vector<double> animateYCoords;
    
    // to be filled in, add in interpolated path by generating points
    // between the reversePathCoords, and then track where the robot is during
    // animation via the "currAnimatedX" and "currAnimatedY" points
    
    /*
    if (isMoving) {
        // implement something
        }
    }
     */
}
