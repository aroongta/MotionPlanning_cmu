#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include <iostream> 
#include <stdlib.h>
#include <string>
#include <vector> //Vector to store coordinates

using namespace std;
struct node
{	//Data structure for each node of the computational graph
	double x, y; // coordinates of node
	// node* neighbors[4]; // list of pointers for neighbours
	node* parent; // pointer to the parent node
	float globalGoal;
	float localGoal;
	bool isObstacle; // whether the node is an obstacle or not
	bool isVisited; // to store if the node is visited or not
	int distance; // to store distance from the start
	vector<node*> vecNeighbors; // list of connections to neighbors
};

class Graphics
{
private:
	vector<double> pathCoordsX;	//list of x coordinates for the optimal path
	vector<double> pathCoordsY;	//list of y coordinates for the optimal path
public:
	Graphics(int winX, int winY); // constructor
	void Draw_Circle(double cx, double cy, int rad); // draws circles
	void Draw_Nodes(int winX, int winY); // draws nodes for the computational graph
	void Draw_GridLines(int winX, int winY); // draws grid of computational graph
	void setPath(double x, double y); // fills path coordinates in the member variable
	void Draw_Path(); // draws path
	void Draw_Path(node* end); // draws path based on node's coordinates

	node* nodes = nullptr; // array of all nodes
	node* start = nullptr; // starting point
	node* end = nullptr; // desired destination
	int coords_Convert(int cX, int cY, int gridSize, int nX); //To convert the coordinates to array indices
	double heuristic(node* first, node* second); // heuristic function
	bool computeAStar(int winX, int winY); // run a-star algorithm
};
//To convert the coordinates to array indices
int Graphics::coords_Convert(int cX, int cY, int gridSize, int nX)
{
	return nX * int(cX / gridSize) + int(cY / gridSize);
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
		for (int j = 10; j <= winY - 10; j += 20)
			Draw_Circle(i, j);
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
	glColor3ub(0, 250, 50);
	glLineWidth(4.0);
	glBegin(GL_LINE_STRIP);
	// glBegin(GL_QUADS);
	for (int i = 0; i < pathCoordsX.size(); i++)
	{
		glVertex2d(pathCoordsX[i], pathCoordsY[i]);
		//glVertex2d(pathCoordsX[i]-10, pathCoordsY[i]-10);
		//glVertex2d(pathCoordsX[i]-10, pathCoordsY[i]+10);
		//glVertex2d(pathCoordsX[i]+10, pathCoordsY[i]+10);
		//glVertex2d(pathCoordsX[i]+10, pathCoordsY[i]-10);
	}
	glEnd();
}

Graphics::Graphics(int winX, int winY)
{
	int gridSize = 20;	//size of the grid-cells
	int nX = winX / gridSize, nY = winY / gridSize;
	cout << nX * nY << '\n';
	int it = 0;
	nodes = new node[nX*nY]; // 500 / 20
	// initialize nodes
	for (int x = 10; x <= winX - 10; x += 20) {
		for (int y = 10; y <= winY - 10; y += 20) {
			nodes[int(x / 20) * 20 + int(y / 20)].x = x;
			it = coords_Convert(x, y, gridSize, nX);
			//printf("y*winX/gridSize + x index %d", y*winX/gridSize + x);
			//cout << it<< "\t";
			//cout << nX * int(x / gridSize) + int(y / gridSize) <<"\t"<<it<< "\t";
			nodes[it].y = y;
			nodes[it].isObstacle = false;
			nodes[it].parent = nullptr;
			nodes[it].isVisited = false;
		}
	}

	// create connections between 4 neighboring nodes
	for (int x = 10; x < winX - 10; x += 20) {
		for (int y = 10; y < winY - 10; y += 20)
		{
			it = coords_Convert(x, y, gridSize, nX);
			if (y >= 30) // so you don't look past top row of nodes
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y - gridSize, gridSize, nX)]);
			//nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[int(x/20)*20 + int((y-1)/20)]); // N neighbor
			if (y<int(x / 20) * 20 + int(y / 20) - 1)
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y + gridSize, gridSize, nX)]);
			//nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[(y + 1) * winX/gridSize + (x + 0)]); // S neighbor
			if (x >= 30)
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x - gridSize, y, gridSize, nX)]);
			//nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[(y + 0) * winX/gridSize + (x - 1)]); // W neighbor
			if (x < winX / gridSize - 1)
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x + gridSize, y, gridSize, nX)]);
			//nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[(y + 0) * winX/gridSize + (x + 1)]);
		}
	}
}

int main(void)
{
	int winX = 500, winY = 500;
	Graphics w1(winX, winY); // graphics window
	FsOpenWindow(16, 16, MAXWIDTH, MAXHEIGHT, 1);

	for (int i = 10; i < 400; i += 20)
		w1.setPath(i, 30);
	for (int i = 30; i < 400; i += 20)
		w1.setPath(390, i);

	while (FsInkey() != FSKEY_ESC)
	{
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
		FsPollDevice();
		w1.Draw_GridLines(winX,winY);
		w1.Draw_Nodes(winX, winY);
		w1.Draw_Path();
		FsSwapBuffers();
		FsSleep(25);
	}
}
