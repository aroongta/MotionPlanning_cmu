#define MAXWIDTH 800
#define MAXHEIGHT 600

#include "fssimplewindow.h"
// #include "ysglfontdata.h"
#include <iostream> 
#include <stdlib.h>
#include <string>
#include <vector> // vector to store coordinates
#include "math.h"
#include <list> // lists for searching through open list and closed list

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
	int coords_Convert(int cX, int cY,int gridSize, int nX); //To convert the coordinates to array indices
    double heuristic(node* first, node* second); // heuristic function
    bool computeAStar(int winX, int winY); // run a-star algorithm
};
//To convert the coordinates to array indices
int Graphics::coords_Convert(int cX, int cY, int gridSize, int nX)
{
	return nX * int(cX / gridSize) + int(cY / gridSize);
}
// draws circles at each specified position
void Graphics::Draw_Circle(double cx, double cy, int rad=2)
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
void Graphics::Draw_Nodes(int winX,int winY)
{
	for (int i=10; i <= winX-10;i+=20)
	{ 
		for (int j=10; j <= winY-10;j+=20)
			Draw_Circle(i, j);
	}
}

// draws grid lines on the computational graph
void Graphics::Draw_GridLines(int winX,int winY)
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
	for (int i=0; i < pathCoordsX.size(); i++)
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
    for (int x=10; x<=winX-10; x+=20) {
        for (int y=10; y<=winY-10; y+=20) {
            nodes[int(x/20)*20 + int(y/20)].x = x;
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
    for (int x=10; x < winX-10; x+=20) {
        for (int y=10; y < winY-10; y+=20)
        {
			it = coords_Convert(x, y, gridSize, nX);
            if(y>=30) // so you don't look past top row of nodes
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y - gridSize, gridSize, nX)]);
					  //nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[int(x/20)*20 + int((y-1)/20)]); // N neighbor
			if (y<int(x / 20) * 20 + int(y / 20) - 1)
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x, y + gridSize, gridSize, nX)]);
						//nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[(y + 1) * winX/gridSize + (x + 0)]); // S neighbor
			if (x >= 30)
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x - gridSize, y, gridSize, nX)]);
				//nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[(y + 0) * winX/gridSize + (x - 1)]); // W neighbor
            if(x<winX/gridSize-1)
				nodes[it].vecNeighbors.push_back(&nodes[coords_Convert(x + gridSize, y, gridSize, nX)]);
                //nodes[int(x/20)*20 + int(y/20)].vecNeighbors.push_back(&nodes[(y + 0) * winX/gridSize + (x + 1)]);
        }
    }
}

double Graphics::heuristic(node* first, node* second)
{
    double distance = sqrt((first->x - second->x)*(first->x - second->x) + (first->y - second->y)*(first->y - second->y));
    return distance;
}

// runs a star search algorithm
bool Graphics::computeAStar(int winX, int winY)
{
    int gridSize = 20;
    // reset all nodes' movement cost and heuristic values
    for (int x=0; x<winX/gridSize; x++) {
        for (int y=0; y<winY/gridSize; y++) {
            nodes[y*winX/gridSize + x].isVisited = false;
            nodes[y*winX/gridSize + x].globalGoal = 10000; // 10000 instead of INFINITY
            nodes[y*winX/gridSize + x].localGoal = 10000;
            nodes[y*winX/gridSize + x].parent = nullptr;
        }
    }
    
    // initialize starting conditions
    node *current = start; // should be nullptr at first
    start->localGoal = 0.0; // f(n) is initialized to 0
    start->globalGoal = heuristic(start, end);
    
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
        openList.sort([](const node* lhs, const node* rhs){ return lhs->globalGoal < rhs->globalGoal; } );
        
        // front of openList is potentially the lowest distance node (after sorting); our
        // list may also contain nodes that have already been visited, so remove these
        while(!openList.empty() && openList.front()->isVisited)
            openList.pop_front(); // remove nodes that have already been visited
        
        // if no valid nodes left to test
        if (openList.empty())
            break;
        
        // continue search
        current = openList.front(); // set current node to front of list in frontier
        current->isVisited = true; // this node has now been visited
        
        
        // check each of this node's neighbours
        for (auto nodeNeighbor : current->vecNeighbors)
        {
            // only if the neighbour is not visited and is not an obstacle, add it to open list to test later
            if (!nodeNeighbor->isVisited && nodeNeighbor->isObstacle == false)
                openList.push_back(nodeNeighbor);
            
            // calculate the neighbor's potential lowest parent distance
            float possiblyLower = current->localGoal + heuristic(current, nodeNeighbor);
            
            // if choosing path through this node is a lower distance than what
            // the neighbor currently has set, update the neighbor to use this node
            // as the source/origin, and set its distance scores as necessary
            if (possiblyLower < nodeNeighbor->localGoal)
            {
                nodeNeighbor->parent = current;
                nodeNeighbor->localGoal = possiblyLower;
                
                // update the neighbour's score since best path length to the neighbor being tested has changed
                nodeNeighbor->globalGoal = nodeNeighbor->localGoal + heuristic(nodeNeighbor, end); // f(n) = g(n) + h(n)
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
		cout << "end is not a null ptr" << endl;
        node *p = end;
        while (p->parent != nullptr) // begin tracing back via parents
		{
            setPath(p->x, p->y);
			cout << p->x << "\t" << p->y;
            //cout << "this is p->x and p->y:" << p->x*20 << " and " << p->y*20 << endl;
//            DrawLine(p->x*nNodeSize + nNodeSize / 2, p->y*nNodeSize + nNodeSize / 2,
//                     p->parent->x*nNodeSize + nNodeSize / 2, p->parent->y*nNodeSize + nNodeSize / 2, PIXEL_SOLID, FG_YELLOW);
            
            // Set next node to this node's parent
            p = p->parent;
        }
    }
    Draw_Path(); // draws path that is stored from setPath (in pathCoords)
}

int main(void)
{
	int winX = 500, winY = 500;
	Graphics w1(winX, winY); // graphics window
	FsOpenWindow(16, 16, MAXWIDTH, MAXHEIGHT,1);
    
    for (int i = 10; i < 400; i += 20)
        w1.setPath(i, 30);
    for (int i = 30; i < 400; i += 20)
        w1.setPath(390, i);
    
    // test case
    w1.start = &w1.nodes[3];
    cout << "w1.start's position:" << w1.nodes[3].x << " " << w1.nodes[3].y << endl;
	cout << "w1.end position:" << w1.nodes[33].x << " " << w1.nodes[33].y << endl;
    w1.end = &w1.nodes[33];
    w1.computeAStar(winX, winY); // compute a-star search for path

	while (FsInkey() != FSKEY_ESC)
	{
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // clear buffer
		FsPollDevice();
		w1.Draw_GridLines(winX,winY);
		w1.Draw_Nodes(winX, winY);
		w1.Draw_Path();
        //w1.Draw_Path(w1.end);
		FsSwapBuffers(); // if double buffer
			// glFlush(); // if single buffer

		FsSleep(25);
	}
}
