#include <iostream> 
#include <stdlib.h>
#include <string>
#include <vector> //Vector to store coordinates
#include <concurrent_priority_queue.h>
using namespace std;

struct node
{
	int x, y;
	node *parent;
	node* neighbors[4];
	double f;
	node();	//default constructor
	node(int x, int y);	//second constructor
};
node::node()	//default consturctore
{}
node::node(int nx, int ny) //second constructor for the struct node
{
	x = nx;
	y = ny;
	parent = nullptr;
	for (int i = 0; i < 4; i++)
		neighbors[i] = nullptr;
	double f = 1000;  //putting f to a very large value initially
}
class Dijkstras
{
private:
	node *grid[25*25];
public:
	void initialize(int startX, int startY, int goalX, int goalY);	//Function to initialize all nodes

};
void Dijkstras::initialize(int startX, int startY,int goalX,int goalY)	//Function to initialize all nodes
{
	int i, j;
	for (int i = 10; i <= 510; i += 20)
	{
		for (int j = 10; j <= 510; j += 20)
		{
			grid[25*int(i/20)+int(j/20)] = new node(i,j);
		}
	}
	
}