//All functions for the Graphics class
//Motion_Planning Simulator
//Created by Ashish Roongta on 11/21/2018

#include "Graphics.h"
// to convert the coordinates to array indices
int Graphics::coords_Convert(int cX, int cY, int gridSize, int nX)
{
	return (nX * int(cX / gridSize) + int(cY / gridSize));
}

// draws circles at each specified position
void Graphics::Draw_Circle(double cx, double cy, int rad = 2)
{
	const double PI = 3.1415927;
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

// stores coordinates of computed path in the member variables
void Graphics::setPath(double x, double y,bool rev)
{
	//set bool to true if you motion planning algorithm
	//outputs path from goal to start
	if (rev)
	{
		pathCoordsX.insert(pathCoordsX.begin(), x);
		pathCoordsY.insert(pathCoordsY.begin(), y);

	}
	if (!rev)
	{
		pathCoordsX.push_back(x);
		pathCoordsY.push_back(y);
	}
}

// draws the computed path - output from algorithm
void Graphics::Draw_Path()
{
	if (pathCoordsX.empty())
	{//To check if a possible path was found by the algorithm
		//cout << "No Path Possible";
		return;
	}
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
void Graphics::Reset()
{
	int winX = 500, winY = 500;
	int gridSize = 20; //size of the grid-cells
	int nX = winX / gridSize, nY = winY / gridSize;
	int it = 0; // iterator
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20) {
			it = coords_Convert(x, y, gridSize, nX);
			nodes[it].x = x;
			// cout << it<< "\t"; // DEBUG
			nodes[it].y = y;
			//nodes[it].isObstacle = false;
			nodes[it].parent = nullptr;
			nodes[it].isVisited = false;
			nodes[it].fCost = 100000; //treated as infinity
			nodes[it].gMovementCost = 100000;
			nodes[it].parent = nullptr;
		}
	}
}
// constructor of graphics object to initialize nodes
Graphics::Graphics(int winX, int winY)
{
	mlp = 0;
	int gridSize = 20; //size of the grid-cells
	int nX = winX / gridSize, nY = winY / gridSize;
	// cout << nX * nY << '\n'; // DEBUG
	int it = 0; // iterator
	nodes = new node[nX*nY]; // 500 / 20
	RobotMoving = false;
	// initialize nodes
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20) {
			it = coords_Convert(x, y, gridSize, nX);
			nodes[it].x = x;
			// cout << it<< "\t"; // DEBUG
			nodes[it].y = y;
			nodes[it].isObstacle = false;
			nodes[it].parent = nullptr;
			nodes[it].isVisited = false;
			nodes[it].fCost = 100000; //treated as infinity
			nodes[it].gMovementCost = 100000;
			nodes[it].parent = nullptr;
		}
	}

	// create connections between 4 neighboring nodes
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20)
		{
			it = coords_Convert(x, y, gridSize, nX);
			if (y >= 30) { // so you don't look past top row of nodes
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y - gridSize, gridSize, nX)]); // north neighbors
			}
			if (y <= winY - 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y + gridSize, gridSize, nX)]); // south neighbors
			}
			if (x >= 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x - gridSize, y, gridSize, nX)]); // west neighbors
			}
			if (x <= winX - 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x + gridSize, y, gridSize, nX)]); // east neighbors
			}
			// also create diagonal connections
			/*if (x >= 30 && y >= 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x - gridSize, y - gridSize, gridSize, nX)]); // northwest neighbors
			}
			if (x <= winX - 30 && y <= winY - 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x + gridSize, y + gridSize, gridSize, nX)]); // southeast neighbors
			}
			if (x >= 30 && y <= winY - 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x - gridSize, y + gridSize, gridSize, nX)]); // southwest neighbors
			}
			if (x <= winX - 30 && y >= 30) {
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x + gridSize, y - gridSize, gridSize, nX)]); // southeast neighbors
			}*/
		}
	}

	// manually set start and end nodes for safety
	start = &nodes[3];
	end = &nodes[33];
	cout << "w1.start's position: " << nodes[3].x << " " << nodes[3].y << endl; // DEBUG
	cout << "w1.end's position: " << nodes[33].x << " " << nodes[33].y << endl; // DEBUG
}

double Graphics::heuristic(node* first, node* second, bool isDijkstras)
{
	if (!isDijkstras) {
		double distance = sqrt((first->x - second->x)*(first->x - second->x) + (first->y - second->y)*(first->y - second->y));
		return distance; // euclidean distance, but change to manhattan distance if needed
	}
	else {
		return 0; // dijkstra's has no heuristic
	}
}
// runs a star search algorithm and create links between start and end point with shortest path
bool Graphics::computeAStar(int winX, int winY, bool isDijkstras)
{
	int gridSize = 20; //size of the grid-cells
	int nX = winX / gridSize, nY = winY / gridSize;
	int it = 0; // iterator
	Reset(); //calling function to reset values of all nodes

	// reset all nodes' movement cost and heuristic values
	/*for (int x=10; x<=winX-10; x+=20) {
		for (int y=10; y<=winY-10; y+=20) {
			it = coords_Convert(x, y, gridSize, nX);
			// cout << it << "\t"; // DEBUG
			nodes[it].isVisited = false;
			nodes[it].fCost = 100000; // treated as infinity
			nodes[it].gMovementCost = 100000;
			nodes[it].parent = nullptr;
		}
	}*/

	// initialize starting conditions
	node *current = start; // should be nullptr at first
	start->gMovementCost = 0.0; // f(n) is initialized to 0
	start->fCost = heuristic(start, end, isDijkstras);

	// add start node to open list -> includes nodes to be tested
	// (newly discovered nodes get added to this list)
	list<node*> openList;
	openList.push_back(start);

	// if open list contains nodes, keep exploring for optimal path
	// however, stop searching when we reach the target
	while (!openList.empty() && current != end)
	{
		// sort nodes in open list by global goal/fCost in ascending order
		openList.sort([](const node* lhs, const node* rhs) { return lhs->fCost < rhs->fCost; });
		// printf("entered loop\n"); // DEBUG

		// front of openList is potentially the lowest distance node (after sorting), but
		// list may also contain nodes that have already been visited, so remove these
		while (!openList.empty() && openList.front()->isVisited) {
			// cout << "openList.front() has already been visited, popping off\n"; // DEBUG
			openList.pop_front(); // remove it
		}

		if (current == end || openList.empty()) { // if you have reached end node, just break out
			cout << "current is end...\n"; // DEBUG
			break;
		}

		// else, continue search
		current = openList.front(); // look at front of sorted list
		current->isVisited = true; // this node has now been visited, so put in closed list/pop off
		openList.pop_front();

		// check each of this node's neighbors (in the closed list)
		for (auto nodeNeighbor : current->vecNeighbors) {
			// only if the neighbour is not visited and is not an obstacle, add it to open list to test later
			if (!nodeNeighbor->isVisited && nodeNeighbor->isObstacle == false)
				openList.push_back(nodeNeighbor);

			// calculate the neighbor's potential lowest parent distance
			float possiblyLower = current->gMovementCost + heuristic(current, nodeNeighbor, isDijkstras);

			// if choosing path through this node is lower distance than current neighbor's,
			// update the neighbor to use this node as the source/origin, and set distance scores as necessary
			if (possiblyLower < nodeNeighbor->gMovementCost)
			{
				nodeNeighbor->parent = current; // set new parent
				nodeNeighbor->gMovementCost = possiblyLower; // update local g cost

				// update the neighbour's score since best path length to the neighbor being tested has changed
				nodeNeighbor->fCost = nodeNeighbor->gMovementCost + heuristic(nodeNeighbor, end, isDijkstras); // f(n) = g(n) + h(n)
			}
		}
	}
	//giving the computed path to the member variables
	if (end != nullptr)
	{
		node *p = end; // starting point of "back-propagated path"
		while (p != nullptr) // begin tracing back via parents
		{
			setPath(p->x, p->y, 1);
			// cout << "Stored path, p->x and p->y:" << p->x << " and " << p->y << endl; // DEBUG
			p = p->parent; // set next node to this node's parent (trace back)
		}
	}
	return true; // end of a-star search
}

// draws computed path by tracing back to the start point from end point via parents
void Graphics::Draw_Path(node *end)
{
	if (end != nullptr)
	{
		node *p = end; // starting point of "back-propagated path"
		while (p != nullptr) // begin tracing back via parents
		{
			setPath(p->x, p->y,1);
			// cout << "Stored path, p->x and p->y:" << p->x << " and " << p->y << endl; // DEBUG
			p = p->parent; // set next node to this node's parent (trace back)
		}
	}
	Draw_Path(); // draws path that is stored from setPath (in pathCoords)
}

// added Draw_StartEndObs() to animate squares with starting point (key S), end point (key E),
// and obstacles (key O)
void Graphics::Draw_StartEndObs() // draws start and end points, obstacles if any
{
	int gridSize = 20, winX = 500, winY = 500;
	int it; // iterator

	// draw starting square
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

	// ending square
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

	// draw obstacles, if any
	for (int i = 10; i <= winX - 10; i += 20) {
		for (int j = 10; j <= winY - 10; j += 20) {
			it = coords_Convert(i, j, gridSize, winX / gridSize);
			if (nodes[it].isObstacle) {
				glBegin(GL_QUADS);
				glColor3ub(80, 80, 80); // obstacles are grey
				glVertex2i(i - gridSize / 2, j - gridSize / 2);
				glVertex2i(i + gridSize / 2, j - gridSize / 2);
				glVertex2i(i + gridSize / 2, j + gridSize / 2);
				glVertex2i(i - gridSize / 2, j + gridSize / 2);
				glEnd();
			}
		}
	}
}
//function to clear the path member variables
void Graphics::ClearPath()
{
	// clear computed path coordinates
	while (!pathCoordsX.empty()) {
		pathCoordsX.pop_back();
	}
	while (!pathCoordsY.empty()) {
		pathCoordsY.pop_back();
	}

}
// added Set_StartEndObs(), allows user to set start and end points manually
void Graphics::Set_StartEndObs(int winX, int winY)
{
	// clear computed path coordinates
	ClearPath();
	auto key = FsInkey();
	int gridSize = 20;
	int mouseEvent, leftButton, middleButton, rightButton, locX, locY; // mouse movement info
	int selectedNodeX, selectedNodeY; // translation to nodes in graph
	mouseEvent = FsGetMouseEvent(leftButton, middleButton, rightButton, locX, locY);
	int iter;
	if ((10 <= locX && locX <= winX - 10) && (10 <= locY && locY <= winY - 10)) {
		selectedNodeX = locX - (locX % 20) + 10;
		selectedNodeY = locY - (locY % 20) + 10;
		iter = coords_Convert(selectedNodeX, selectedNodeY, gridSize, winX / gridSize);
		// set start and end locs & obstacles
		if (key == FSKEY_S)
			if(nodes[iter].isObstacle==false)
				start = &nodes[iter];
		if (key == FSKEY_E)
			if(nodes[iter].isObstacle==false)
				end = &nodes[iter];
		if (key == FSKEY_O) {
			if (start != &nodes[iter] && end != &nodes[iter])
				nodes[iter].isObstacle = true;
		}
		if (key == FSKEY_P)
			if(nodes[iter].isObstacle==true)	//checking if that node is designated as an obstacle 
				nodes[iter].isObstacle = false;
	}
}
void Graphics::dispMenu(int x, int y, int width, int height)
{
	int xSpacing = width / 2.0;
	int ySpacing = height / 2.0;
	glColor3ub(0, 255, 255);
	glBegin(GL_QUADS);
	int i = 4*ySpacing;
	//For animate robot button
	glVertex2d(x - xSpacing, y - ySpacing);
	glVertex2d(x + xSpacing, y - ySpacing);
	glVertex2d(x + xSpacing, y + ySpacing);
	glVertex2d(x - xSpacing, y + ySpacing);
	//For A* motion planning algorithm
	glVertex2d(x - xSpacing, y - ySpacing+i);
	glVertex2d(x + xSpacing, y - ySpacing+i);
	glVertex2d(x + xSpacing, y + ySpacing+i);
	glVertex2d(x - xSpacing, y + ySpacing+i);

	//For Dijkstra's MLP button
	glVertex2d(x - xSpacing, y - ySpacing + 2 * i);
	glVertex2d(x + xSpacing, y - ySpacing + 2 * i);
	glVertex2d(x + xSpacing, y + ySpacing + 2 * i);
	glVertex2d(x - xSpacing, y + ySpacing + 2 * i);

	//For RRT MLP button
	glVertex2d(x - xSpacing, y - ySpacing + 3 * i);
	glVertex2d(x + xSpacing, y - ySpacing + 3 * i);
	glVertex2d(x + xSpacing, y + ySpacing + 3 * i);
	glVertex2d(x - xSpacing, y + ySpacing + 3 * i);

	//For D* MLP button
	glVertex2d(x - xSpacing, y - ySpacing + 4 * i);
	glVertex2d(x + xSpacing, y - ySpacing + 4 * i);
	glVertex2d(x + xSpacing, y + ySpacing + 4 * i);
	glVertex2d(x - xSpacing, y + ySpacing + 4 * i);

	glEnd();

	displayText("Animate robot.", x - 50, y + 5);
	displayText("A* Algorithm", x - 50, y + 5+ i);
	displayText("Dijkstra's.", x - 50, y + 5+ 2*i);
	displayText("RRT algoritm.", x - 50, y + 5+3*i);
	displayText("D* Lite", x - 50, y + 5 + 4*i);

	// now, poll for mouse coordinates
	int mouseEvent, leftButton, middleButton, rightButton, locX, locY; // mouse movement info
	mouseEvent = FsGetMouseEvent(leftButton, middleButton, rightButton, locX, locY);
	//0:animate robot
	//1:A* MLP
	//2:Dijkstra's
	//3:RRT
	//4:D* Lite

	for (int u = 0; u <= 4; u++)
	{
		if ((x - xSpacing <= locX && locX <= x + xSpacing) && (y - ySpacing+u*i <= locY && locY <= y + ySpacing+u*i)) {
			// set animation movement
			if (leftButton)
			{
				if (u == 0)
				{
					RobotMoving = true;
					cout << "animate robot has been clicked!";
				}
				else
				{
					//cout << "Clicked:" << u << endl;;
					mlp = u;
					break;
				}
			}
		}
	}
}

// display text on menu
void Graphics::displayText(const string message, int x, int y)
{
	glColor3ub(0, 0, 255);
	glRasterPos2i(x, y);  // sets position
	YsGlDrawFontBitmap8x8(message.c_str());
}
