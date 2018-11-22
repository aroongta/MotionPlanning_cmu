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
#include <iostream>
#include "math.h"
#include <string>
#include "ysglfontdata.h"
#include <iostream>
#include <vector>
#include <list>

using namespace std;

struct node
{    //Data structure for each node of the computational graph
	double x, y; // coordinates of node
	// node* neighbors[4]; // list of pointers for neighbours
	node* parent; // pointer to the parent node
	float fCost; // f(n) = g(n) + h(n)
	float gMovementCost; // g(n)
	bool isObstacle; // whether the node is an obstacle or not
	bool isVisited; // to store if the node is visited or not
	int distance; // to store distance from the start
	vector<node*> vecNeighbors; // list of connections to neighbors
};
class Graphics
{
public:
	vector<double> pathCoordsX;    //list of x coordinates for the optimal path
	vector<double> pathCoordsY;    //list of y coordinates for the optimal path
public:
	Graphics(int winX, int winY); // constructor
	void Draw_Circle(double cx, double cy, int rad); // draws circles
	void Draw_Nodes(int winX, int winY); // draws nodes for the computational graph
	void Draw_GridLines(int winX, int winY); // draws grid of computational graph
	void setPath(double x, double y); // fills path coordinates in the member variable
	void Draw_Path(); // draws path
	void Draw_Path(node* end); // draws path based on node's coordinates
	void Draw_StartEnd(); // animates start and end points
	void Set_StartEnd(int winX, int winY); // allows user to set start and end points
	void draw_obstacles();
	//node **nodes=new node*[625]; // array of all nodes
	node *nodes=nullptr;
	node* start = nullptr; // starting point
	node* end = nullptr; // desired destination
	void setObstacles();
	int coords_Convert(int cX, int cY, int gridSize, int nX); //To convert the coordinates to array indices
	double heuristic(node* first, node* second); // heuristic function
	bool computeAStar(int winX, int winY); // run a-star algorithm
};
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
    void animateRobot(vector <double> x, vector <double> y);
};
#endif /* robotAnimation_h */
