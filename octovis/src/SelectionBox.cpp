#include <octovis/SelectionBox.h>

namespace octomap{

  SelectionBox::SelectionBox()
    : m_visible(false)
  {

	  m_frames.push_back(new qglviewer::ManipulatedFrame());
	  m_frames.push_back(new qglviewer::ManipulatedFrame());

	  frame(0)->setTranslation(0,0,0);
	  frame(1)->setTranslation(1,1,1);

	  qglviewer::LocalConstraint* XAxis = new qglviewer::LocalConstraint();
	  XAxis->setTranslationConstraint(qglviewer::AxisPlaneConstraint::FREE, qglviewer::Vec(1.0,0.0,0.0));
	  XAxis->setRotationConstraint   (qglviewer::AxisPlaneConstraint::FORBIDDEN, qglviewer::Vec(0.0,0.0,0.0));
	  frame(0)->setConstraint(XAxis);
	  frame(1)->setConstraint(XAxis);

	  //WorldConstraint* xyzConstraint = new WorldConstraint();


  }

  SelectionBox::~SelectionBox(){
	  delete m_frames[0];
	  delete m_frames[1];

  }

  void SelectionBox::draw(bool withNames) const{
	  qglviewer::Vec minPt = frame(0)->position();
	  qglviewer::Vec maxPt = frame(1)->position();

	  glEnable(GL_LINE_SMOOTH);
	  glLineWidth(2.);
	  glDisable(GL_LIGHTING);
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

	  glDisable(GL_LINE_SMOOTH);
	  glEnable(GL_LIGHTING);


	  // draw spheres in their frames:
	  GLUquadricObj* quadric=gluNewQuadric();
	  gluQuadricNormals(quadric, GLU_SMOOTH);
      glColor4f(1.0, 0.0, 0.0, 1.0);

      for (unsigned i = 0; i < m_frames.size(); ++i){
		  glPushMatrix();
		  glMultMatrixd(m_frames[i]->matrix());
		  if (withNames)
			  glPushName(i);
		  gluSphere(quadric, 0.04, 32, 32);
		  if (withNames)
			  glPopName();
		  glPopMatrix();
      }

	  gluDeleteQuadric(quadric);

  }

  void SelectionBox::getBBXMin(float& x, float& y, float& z) const {
	  x = frame(0)->position().x;
	  y = frame(0)->position().y;
	  z = frame(0)->position().z;

	  for (unsigned i = 1; i < m_frames.size(); ++i){
		x = std::min(x,frame(i)->position().x);
		y = std::min(y,frame(i)->position().y);
		z = std::min(z,frame(i)->position().z);
	  }
  }

  void SelectionBox::getBBXMax(float& x, float& y, float& z) const {
	  x = frame(0)->position().x;
	  y = frame(0)->position().y;
	  z = frame(0)->position().z;

	  for (unsigned i = 1; i < m_frames.size(); ++i){
		x = std::max(x,frame(i)->position().x);
		y = std::max(y,frame(i)->position().y);
		z = std::max(z,frame(i)->position().z);
	  }
  }


}
