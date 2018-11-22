
#define MAXWIDTH 800
#define MAXHEIGHT 600
#include "Graphics.h"

using namespace std;
// run the main program
int main(void)
{
	int winX = 500, winY = 500, i = 0;
	Robot quadCopter;
	Graphics w1(winX, winY); // graphics window
	FsOpenWindow(16, 16, MAXWIDTH, MAXHEIGHT, 1);

	while (FsInkey() != FSKEY_ESC)
	{
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // clear buffer
		FsPollDevice();
		w1.Draw_GridLines(winX, winY);
		w1.Draw_Nodes(winX, winY);
		
		w1.Set_StartEndObs(winX, winY); // allows user to manually set start and end nodes
		w1.Draw_StartEndObs(); // draw start points, end points, obstacles
		w1.computeAStar(winX, winY); // compute a-star search for path
		w1.Draw_Path(w1.end); // highlight the shortest path
		if (w1.RobotMoving)
			w1.RobotMoving=quadCopter.animateRobot(i, w1.pathCoordsX, w1.pathCoordsY);
		else
		{
			quadCopter.currentX = w1.start->x;
			quadCopter.currentY = w1.start->y;
			quadCopter.drawRobot();	//draw robot at the starting point
			i = 0;
		}
		w1.dispMenu(600, 100, 120, 20); // display animation button
		quadCopter.interpolatePath(w1.pathCoordsX, w1.pathCoordsY); // interpolate path for animation (needs implementation)
		FsSwapBuffers(); // if double buffer
		FsSleep(25);
	}
	return 0; 
}
