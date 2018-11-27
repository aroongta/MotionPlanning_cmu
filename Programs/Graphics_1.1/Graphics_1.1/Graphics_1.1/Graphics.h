#pragma once
// Definitions of nide struct, Graphics Class and Robot Class
//Motion Planning Simulator
//
// 24780: Engineering Computation Project
//Created by: Ashish Roongta and Bryan Zhao on 11/19/2018
//  Copyright © 2018A shish Roongta. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <list>
#include "fssimplewindow.h"
#include "math.h"
#include <string>
#include "ysglfontdata.h"
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
//Data structure for each node of the computational graph
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
	/*node()
	{//default constructor
		x = y = 0.0;
		parent = nullptr;
		fCost = 100000;
		gMovementCost =100000;
		isObstacle = false;
		isVisited = false;
		distance = 0;
	}*/
};
class Graphics
{
private:
	// removed vector of pathCoords, placed into public (for implementing animation)
public:
	int mlp;	//to stor the indicator for motion planning algorithm chosen
	vector<double> pathCoordsX;    //list of x coordinates for the optimal path
	vector<double> pathCoordsY;    //list of y coordinates for the optimal path
	node* nodes = nullptr; // array of all nodes
	node* start = nullptr; // starting point
	node* end = nullptr; // desired destination
	bool RobotMoving;
	Graphics() {};	//Default constructor
	Graphics(int winX, int winY); // constructor
	void Reset();	//Function to reset all values
	void ClearPath();	//Fucntion to clear the path
	void Draw_Circle(double cx, double cy, int rad); // draws circles
	void Draw_Nodes(int winX, int winY); // draws nodes for the computational graph
	void Draw_GridLines(int winX, int winY); // draws grid of computational graph
	void setPath(double x, double y,bool rev); // fills path coordinates in the member variable
	void Draw_Path(); // draws path
	void Draw_Path(node* end); // draws path based on node's coordinates
	void Draw_StartEndObs(); // animates start and end points, obstacles if any
	void Set_StartEndObs(int winX, int winY); // allows user to set start and end points, obstacles if any
	int coords_Convert(int cX, int cY, int gridSize, int nX); // to convert the coordinates to array indices
	double heuristic(node* first, node* second, bool isDijkstras); // heuristic function
	bool computeAStar(int winX, int winY, bool isDijkstras); // run a-star algorithm
	void dispMenu(int locX, int locY, int width, int height);
	void displayText(const string message, int width, int height); // displays text on screen
};
//Class for Robot animation
class Robot{
public:
	Robot(); // default constructor
	int pollDevice(int input); // polls device from outside world
	bool isPaused; // allows user to pause path
	//bool isMoving; // animation state
	void drawRobot(); // draws robot in window
	void drawRotor(int cx, int cy, int rad); // draws rotors of quad
	void drawCircle(int cx, int cy, int rad, bool fill);
	double theta; // angle of rotor blades
	void interpolatePath(vector<double> xCoords, vector<double> yCoords);
	double  currentX; // current location for animation
	double currentY;
	bool animateRobot(int &i,vector<double> x, vector<double> y);	//Funciton to animate Robot
};