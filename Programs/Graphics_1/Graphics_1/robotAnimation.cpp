//  robotAnimation.cpp
//  motion_planning_sim
//
//  Created by Bryan Zhao on 11/20/18.

#define PI 3.1415926535
#define DEG2RAD PI/180.0

#include "robotAnimation.h"
//To convert the coordinates to array indices
int Graphics::coords_Convert(int cX, int cY, int gridSize, int nX)
{
	return (nX * int(cX / gridSize) + int(cY / gridSize));
}
// draws circles at each specified position
void Graphics::Draw_Circle(double cx, double cy, int rad = 2)
{
	//const double PI = 3.1415927;
	glBegin(GL_POLYGON);
	glColor3ub(100, 100, 100);
	int i;
	for (i = 0; i < 64; i++) {//to draw head of the pedestrian
		double angle = (double)i * PI / 32.0;
		double x = (double)cx + cos(angle)*(double)rad;
		double y = (double)cy + sin(angle)*(double)rad;
		glVertex2d(x, y);
	}
	glEnd();
}

// draws nodes, represented by circles, on the computational graph
void Graphics::Draw_Nodes(int winX, int winY)
{
	for (int i = 10; i <= winX - 10; i += 20)
	{
		for (int j = 10; j <= winY - 10; j += 20) {
			Draw_Circle(i, j);
		}
	}
}

// draws grid lines on the computational graph
void Graphics::Draw_GridLines(int winX, int winY)
{
	glLineWidth(1.0);
	glColor3ub(100, 100, 100);
	//glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINES);
	for (int i = 0; i <= winX; i += 20)
	{ // to plot vertical grid lines
		glVertex2i(i, 0);
		glVertex2i(i, winY);
	}
	for (int j = 0; j <= winY; j += 20)
	{ // to plot horizontal gridlines
		glVertex2i(0, j);
		glVertex2i(winX, j);
	}
	// glDisable(GL_LINE_STIPPLE);
	glEnd();
}

// stores coordinates of computed path in the member variable
void Graphics::setPath(double x, double y)
{
	pathCoordsX.push_back(x);
	pathCoordsY.push_back(y);
}

// draws the computed path - output from algorithm
void Graphics::Draw_Path()
{
	glColor3ub(0, 150, 0);
	glLineWidth(4.0);
	glBegin(GL_LINE_STRIP);
	// glBegin(GL_QUADS);
	for (int i = 0; i < pathCoordsX.size(); i++)
	{
		glVertex2d(pathCoordsX[i], pathCoordsY[i]);
	}
	glEnd();
}
//Function to draw obstacles
Graphics::Graphics(int winX, int winY)
{
	int gridSize = 20;    //size of the grid-cells
	int nX = winX / gridSize, nY = winY / gridSize;
	cout << nX * nY << '\n';
	int it = 0; // iterator
	nodes = new node[nX*nY]; // 500 / 20
	// initialize nodes
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20) {
			it = coords_Convert(x, y, gridSize, nX);
			//nodes[it] = new node;
			nodes[it].x = x;
			
			// cout << it<< "\t"; // DEBUG
			nodes[it].y = y;
			nodes[it].isObstacle = false;
			nodes[it].parent = nullptr;
			nodes[it].isVisited = false;
		}
	}

	// create connections between 4 neighboring nodes
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20)
		{
			it = coords_Convert(x, y, gridSize, nX);
			if (y >= 30) {// so you don't look past top row of nodes
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y - gridSize, gridSize, nX)]); // N
			}
			if (y <= winY - 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y + gridSize, gridSize, nX)]); // S
			}
			if (x >= 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x - gridSize, y, gridSize, nX)]); // W
			}
			if (x <= winX - 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x + gridSize, y, gridSize, nX)]); // E
			}
		}
	}

	// manually set start and end nodes as a precaution
	start = &nodes[3];
	end = &nodes[33];
	cout << "w1.start's position: " << nodes[3].x << " " << nodes[3].y << endl; // DEBUG
	cout << "w1.end's position: " << nodes[33].x << " " << nodes[33].y << endl; // DEBUG
}
void Graphics::setObstacles()
{
	for (int b = 5; b < 625; b += 5)
		nodes[b].isObstacle = true;
}
void Graphics::draw_obstacles()
{
	glBegin(GL_QUADS);
	glColor3ub(200, 0, 0);	//black color for obstcles
		for (int i = 0; i < 625; i++)
	{
		if (nodes[i].isObstacle)
		{
			cout << "obstacle found" << endl;
			glVertex2d(nodes[i].x - 10, nodes[i].y - 10);
			glVertex2d(nodes[i].x + 10, nodes[i].y - 10);
			glVertex2d(nodes[i].x + 10, nodes[i].y + 10);
			glVertex2d(nodes[i].x - 10, nodes[i].y + 10);
		}
	}
	glEnd();
}
double Graphics::heuristic(node* first, node* second)
{
	double distance = sqrt((first->x - second->x)*(first->x - second->x) + (first->y - second->y)*(first->y - second->y));
	return distance; // euclidean distance, but change to manhattan distance if needed
}

// runs a star search algorithm
bool Graphics::computeAStar(int winX, int winY)
{
	int gridSize = 20; //size of the grid-cells
	int nX = winX / gridSize, nY = winY / gridSize;
	int it = 0; // iterator

	// reset all nodes' movement cost and heuristic values
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20) {
			it = coords_Convert(x, y, gridSize, nX);
			// cout << it << "\t"; // DEBUG
			nodes[it].isVisited = false;
			nodes[it].fCost = 100000; // treated as infinity
			nodes[it].gMovementCost = 100000;
			nodes[it].parent = nullptr;
			nodes[it].isObstacle = false;
		}
	}

	// initialize starting conditions
	node *current = start; // should be nullptr at first
	start->gMovementCost = 0.0; // f(n) is initialized to 0
	start->fCost = heuristic(start, end);

	// add start node to open list - this will ensure it gets tested
	// as the algorithm progresses, newly discovered nodes get added to this
	// list, and will themselves be tested later
	list<node*> openList;
	openList.push_back(start);

	// if the not tested list contains nodes, there may be better paths
	// which have not yet been explored
	// however, we will also stop searching when we reach the target

	while (!openList.empty() && current != end)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
	{
		// sort nodes in open list by global goal in ascending order
		openList.sort([](const node* lhs, const node* rhs) { return lhs->fCost < rhs->fCost; });
		// printf("entered loop\n"); // DEBUG
		// front of openList is potentially the lowest distance node (after sorting); our
		// list may also contain nodes that have already been visited, so remove these
		while (openList.front()->isVisited && !openList.empty()) {
			// cout << "openList.front() has already been visited, popping off\n"; // DEBUG
			openList.pop_front(); // remove it
		}

		// if no valid nodes left to test
		if (openList.empty()) {
			printf("openList empty, breaking out\n");
			break;
		}

		if (current == end) {
			cout << "current is end... not breaking out?\n";
			break;
		}

		// continue search
		current = openList.front(); // remove nodes that have already been visited
		current->isVisited = true; // this node has now been visited, put in closed list
		openList.pop_front();

		// current = openList.front(); // set current node to front of list in frontier

		// check each of this node's neighbors (in the closed list)
		for (auto nodeNeighbor : current->vecNeighbors) {
			// only if the neighbour is not visited and is not an obstacle, add it to open list to test later
			if (!nodeNeighbor->isVisited && nodeNeighbor->isObstacle == false)
				openList.push_back(nodeNeighbor);

			// calculate the neighbor's potential lowest parent distance
			float possiblyLower = current->gMovementCost + heuristic(current, nodeNeighbor);

			// if choosing path through this node is lower distance than current neighbor's,
			// update the neighbor to use this node as the source/origin, and set its distance scores as necessary
			if (possiblyLower < nodeNeighbor->gMovementCost)
			{
				// cout << "is parent being set?\n"; // DEBUG
				nodeNeighbor->parent = current;
				nodeNeighbor->gMovementCost = possiblyLower;

				// update the neighbour's score since best path length to the neighbor being tested has changed
				nodeNeighbor->fCost = nodeNeighbor->gMovementCost + heuristic(nodeNeighbor, end); // f(n) = g(n) + h(n)
			}
		}
	}
	return true;
}

// draws computed path by tracing back to the start point from end point via parents
void Graphics::Draw_Path(node *end)
{
	if (end != nullptr) // just for safety
	{
		node *p = end; // starting point of "back-propagated path"
		while (p != nullptr) // begin tracing back via parents
		{
			// cout << "p's parent is not nullptr\n"; // DEBUG
			setPath(p->x, p->y);
			// cout << "Stored path, p->x and p->y:" << p->x << " and " << p->y << endl; // DEBUG
			p = p->parent; // set next node to this node's parent (trace back)
		}
	}
	Draw_Path(); // draws path that is stored from setPath (in pathCoords)
}

void Graphics::Draw_StartEnd() // draws start and end points
{
	int gridSize = 20;
	if (start != nullptr) {
		glBegin(GL_QUADS);
		glColor3ub(0, 50, 255); // start goal is blue!
		int x = start->x;
		int y = start->y;
		glVertex2i(x - gridSize / 2, y - gridSize / 2);
		glVertex2i(x + gridSize / 2, y - gridSize / 2);
		glVertex2i(x + gridSize / 2, y + gridSize / 2);
		glVertex2i(x - gridSize / 2, y + gridSize / 2);
		glEnd();
	}
	if (end != nullptr) {
		glBegin(GL_QUADS);
		glColor3ub(255, 50, 0); // end goal is red!
		int xEnd = end->x;
		int yEnd = end->y;
		glVertex2i(xEnd - gridSize / 2, yEnd - gridSize / 2);
		glVertex2i(xEnd + gridSize / 2, yEnd - gridSize / 2);
		glVertex2i(xEnd + gridSize / 2, yEnd + gridSize / 2);
		glVertex2i(xEnd - gridSize / 2, yEnd + gridSize / 2);
		glEnd();
	}
}

// allows user to set start and end points manually
void Graphics::Set_StartEnd(int winX, int winY)
{
	// clear computed path coordinates
	while (!pathCoordsX.empty()) {
		pathCoordsX.pop_back();
	}
	while (!pathCoordsY.empty()) {
		pathCoordsY.pop_back();
	}

	auto key = FsInkey();
	int gridSize = 20;
	int mouseEvent, leftButton, middleButton, rightButton, locX, locY; // mouse movement info
	int selectedNodeX, selectedNodeY; // translation to nodes in graph
	mouseEvent = FsGetMouseEvent(leftButton, middleButton, rightButton, locX, locY);

	if ((10 <= locX && locX <= winX - 10) && (10 <= locY && locY <= winY - 10)) {
		// 139 % 20 = 19, 139-19 = 120 + 10 = 130
		selectedNodeX = locX - (locX % 20) - 10;
		selectedNodeY = locY - (locY % 20) + 10;

		// set start and end locs
		if (key == FSKEY_S)
			start = &nodes[coords_Convert(selectedNodeX, selectedNodeY, gridSize, winX / gridSize)];
		if (key == FSKEY_E)
			end = &nodes[coords_Convert(selectedNodeX, selectedNodeY, gridSize, winX / gridSize)];
	}
}

// default constructor
Robot::Robot()
{
    isMoving = false; // tracks animation movement
    isPaused = false;
    theta = 0; // rotation angle of rotors
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
void Robot::drawRobot(int locX, int locY)
{
    // draw center body
    double x = locX, y = locY; // typecast for glVertex2d
    glLineWidth(1.0);
    glColor3ub(90, 90, 90);
    glBegin(GL_QUADS);
    glVertex2d(x-5, y-5);
    glVertex2d(x+5, y-5);
    glVertex2d(x+5, y+5);
    glVertex2d(x-5, y+5);
    glEnd();
    
    // draw rotors of quadcopter
    double rad = 9; // radius of rotor
    int spacing = 12;
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
    
    theta+=4;
}

// displays animation menu
void Robot::dispMenu(int x, int y, int width, int height)
{
    int xSpacing = width/2.0;
    int ySpacing = height/2.0;
    glColor3ub(0, 255, 255);
    glBegin(GL_QUADS);
    glVertex2d(x-xSpacing, y-ySpacing);
    glVertex2d(x+xSpacing, y-ySpacing);
    glVertex2d(x+xSpacing, y+ySpacing);
    glVertex2d(x-xSpacing, y+ySpacing);
    glEnd();
    
    displayText("Animate robot.", x-50, y+5);
    
    // now, poll for mouse coordinates
    int mouseEvent, leftButton, middleButton, rightButton, locX, locY; // mouse movement info
    mouseEvent = FsGetMouseEvent(leftButton, middleButton, rightButton, locX, locY);
    
    if ((x-xSpacing <= locX && locX <= x+xSpacing) && (y-ySpacing <= locY && locY <= y+ySpacing)) {
        
        // set animation movement
        if(mouseEvent == FSMOUSEEVENT_LBUTTONDOWN) {
            isMoving = true;
            cout << "animate robot has been clicked!";
        }
        if(mouseEvent == FSMOUSEEVENT_LBUTTONUP)
            isMoving = false;
    }
}

// display text on menu
void Robot::displayText(const string message, int x, int y)
{
    glColor3ub(0, 0, 255);
    glRasterPos2i(x, y);  // sets position
    YsGlDrawFontBitmap8x8(message.c_str());
}

// animates motion
void Robot::animateRobot(vector <double> x, vector <double> y)
{
	int i,j;
	if (isMoving) {
        // implement something
		for (i = 0; i < x.size()-1; i++)
		{
			for (j = 1; j <= 10; j++)
			{
				
				drawRobot(x[i] + j*(x[i] - x[i + 1]) / 10,y[i] + j*(y[i] - y[i + 1]) / 10);
				FsSleep(25);

			}
		}
    }
}
