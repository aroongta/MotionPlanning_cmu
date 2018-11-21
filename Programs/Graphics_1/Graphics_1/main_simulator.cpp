#define MAXWIDTH 800
#define MAXHEIGHT 600

#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector> // vector to store coordinates
#include "math.h"
#include <list> // lists for searching through open list and closed list
#include "robotAnimation.h"


using namespace std;
/*
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

    Graphics(int winX, int winY); // constructor
    void Draw_Circle(double cx, double cy, int rad); // draws circles
    void Draw_Nodes(int winX, int winY); // draws nodes for the computational graph
    void Draw_GridLines(int winX, int winY); // draws grid of computational graph
    void setPath(double x, double y); // fills path coordinates in the member variable
    void Draw_Path(); // draws path
    void Draw_Path(node* end); // draws path based on node's coordinates
    void Draw_StartEnd(); // animates start and end points
    void Set_StartEnd(int winX, int winY); // allows user to set start and end points
    
    node* nodes = nullptr; // array of all nodes
    node* start = nullptr; // starting point
    node* end = nullptr; // desired destination
    
    int coords_Convert(int cX, int cY,int gridSize, int nX); //To convert the coordinates to array indices
    double heuristic(node* first, node* second); // heuristic function
    bool computeAStar(int winX, int winY); // run a-star algorithm
};
*/

int main(void)
{
    int winX = 500, winY = 500;
    Robot quadCopter;
    Graphics w1(winX, winY); // graphics window
    FsOpenWindow(16, 16, MAXWIDTH, MAXHEIGHT,1);
    
    /*
    for (int i = 10; i < 400; i += 20) // test case for setPath
        w1.setPath(i, 30);
    for (int i = 30; i < 400; i += 20)
        w1.setPath(390, i); */
    
    /* w1.start = &w1.nodes[40]; // test case for a-star
    w1.end = &w1.nodes[80];
    w1.Draw_StartEnd(); */
	w1.setObstacles();
	for (int b = 0; b < 625; b++)
		cout << w1.nodes[b].x<<"\t"<<w1.nodes[b].y<<"\t"<<w1.nodes[b].isObstacle<<endl;
    while (FsInkey() != FSKEY_ESC)
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // clear buffer
        FsPollDevice();
        w1.Draw_GridLines(winX,winY);
        w1.Draw_Nodes(winX, winY);
        // w1.Draw_Path();
        w1.Draw_StartEnd();
        w1.computeAStar(winX, winY); // compute a-star search for path
        w1.Set_StartEnd(winX, winY); // allows user to manually set start and end nodes
        w1.Draw_Path(w1.end);
        quadCopter.drawRobot(w1.start->x, w1.start->y);
		quadCopter.animateRobot(w1.pathCoordsX,w1.pathCoordsY);
        quadCopter.dispMenu(600, 100, 120, 20);
		w1.draw_obstacles();
        FsSwapBuffers(); // if double buffer
        FsSleep(25);
    }
}
