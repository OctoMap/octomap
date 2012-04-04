#include <octovis/SelectionBox.h>

namespace octomap{

  SelectionBox::SelectionBox()
    : m_visible(false),
      minPt(0,0,0), maxPt(1,1,1)
  {


  }

  SelectionBox::~SelectionBox(){

  }

  void SelectionBox::draw() const{
	  glEnable(GL_LINE_SMOOTH);
	  glLineWidth(3.);
	  glColor3f(0.9,0.0, 0.0);
	  glBegin(GL_LINE_LOOP); // Start drawing a line primitive
	  glVertex3f(minPt.x, minPt.y, minPt.z);
	  glVertex3f(maxPt.x, minPt.y, minPt.z);
	  glVertex3f(maxPt.x, maxPt.y, minPt.z);
	  glVertex3f(minPt.x, maxPt.y, minPt.z);
	  glEnd();

	  glBegin(GL_LINE_LOOP);
	  glVertex3f(minPt.x, minPt.y, maxPt.z);
	  glVertex3f(maxPt.x, minPt.y, maxPt.z);
	  glVertex3f(maxPt.x, maxPt.y, maxPt.z);
	  glVertex3f(minPt.x, maxPt.y, maxPt.z);
//	  glVertex3f(-1.0f, -1.0f, 0.0f); // The bottom left corner
//	  glVertex3f(-1.0f, 1.0f, 0.0f); // The top left corner
//	  glVertex3f(1.0f, 1.0f, 0.0f); // The top right corner
//	  glVertex3f(1.0f, -1.0f, 0.0f); // The bottom right corner
	  glEnd();

	  glBegin(GL_LINES);
	  glVertex3f(minPt.x, minPt.y, minPt.z);
	  glVertex3f(minPt.x, minPt.y, maxPt.z);

	  glVertex3f(maxPt.x, minPt.y, minPt.z);
	  glVertex3f(maxPt.x, minPt.y, maxPt.z);

	  glVertex3f(maxPt.x, maxPt.y, minPt.z);
	  glVertex3f(maxPt.x, maxPt.y, maxPt.z);

	  glVertex3f(minPt.x, maxPt.y, minPt.z);
	  glVertex3f(minPt.x, maxPt.y, maxPt.z);
	  glEnd();


	  //glDisable(GL_LIGHTING);

	  glDisable(GL_LINE_SMOOTH);
	  glEnable(GL_LIGHTING);

  }


}
