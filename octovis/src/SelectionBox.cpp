#include <octovis/SelectionBox.h>

namespace octomap{

  SelectionBox::SelectionBox()
    : m_visible(false),
      m_minPt(0,0,0), m_maxPt(1,1,1),
      m_arrowLength(0.2)
  {



    for (unsigned i=0; i< 3; ++i){
      m_frames.push_back(new qglviewer::ManipulatedFrame());
    }

    for (unsigned i=0; i< 3; ++i){
      m_frames.push_back(new qglviewer::ManipulatedFrame());
    }

	  qglviewer::WorldConstraint* XAxis = new qglviewer::WorldConstraint();
	  XAxis->setTranslationConstraint(qglviewer::AxisPlaneConstraint::AXIS, qglviewer::Vec(1.0,0.0,0.0));
	  XAxis->setRotationConstraint   (qglviewer::AxisPlaneConstraint::FORBIDDEN, qglviewer::Vec(0.0,0.0,0.0));

	  qglviewer::WorldConstraint* YAxis = new qglviewer::WorldConstraint();
	  YAxis->setTranslationConstraint(qglviewer::AxisPlaneConstraint::AXIS, qglviewer::Vec(0.0,1.0,0.0));
	  YAxis->setRotationConstraint   (qglviewer::AxisPlaneConstraint::FORBIDDEN, qglviewer::Vec(0.0,0.0,0.0));

	  qglviewer::WorldConstraint* ZAxis = new qglviewer::WorldConstraint();
	  ZAxis->setTranslationConstraint(qglviewer::AxisPlaneConstraint::AXIS, qglviewer::Vec(0.0,0.0,1.0));
	  ZAxis->setRotationConstraint   (qglviewer::AxisPlaneConstraint::FORBIDDEN, qglviewer::Vec(0.0,0.0,0.0));


	  frame(0)->setConstraint(XAxis);
	  frame(1)->setConstraint(YAxis);
	  frame(2)->setConstraint(ZAxis);
	  frame(3)->setConstraint(XAxis);
	  frame(4)->setConstraint(YAxis);
	  frame(5)->setConstraint(ZAxis);



  }

  SelectionBox::~SelectionBox(){
	  delete m_frames[0];
	  delete m_frames[1];

  }

  void SelectionBox::draw(bool withNames){


   // set min/max new from grabbed frame:
	  for (unsigned i = 0; i < m_frames.size(); ++i){
	    if (frame(i)->grabsMouse()){
	      qglviewer::Vec f = frame(i)->position();

	      unsigned oi = i+3;
	      float corr = m_arrowLength/2.0;
	      if (i >= 3){
	        oi = i-3;
	        corr *= -1;
	      }

	      qglviewer::Vec fo = frame(oi)->position();

	      unsigned ci = i%3;
	      m_minPt[ci] = std::min(f[ci] - corr, fo[ci] + corr);
	      m_maxPt[ci] = std::max(f[ci] - corr, fo[ci] + corr);
	    }
	  }

	  // draw box:

	  glEnable(GL_LINE_SMOOTH);
	  glLineWidth(2.);
	  glDisable(GL_LIGHTING);
	  glColor3f(0.9,0.0, 0.0);
	  glBegin(GL_LINE_LOOP); // Start drawing a line primitive
	  glVertex3f(m_minPt.x, m_minPt.y, m_minPt.z);
	  glVertex3f(m_maxPt.x, m_minPt.y, m_minPt.z);
	  glVertex3f(m_maxPt.x, m_maxPt.y, m_minPt.z);
	  glVertex3f(m_minPt.x, m_maxPt.y, m_minPt.z);
	  glEnd();

	  glBegin(GL_LINE_LOOP);
	  glVertex3f(m_minPt.x, m_minPt.y, m_maxPt.z);
	  glVertex3f(m_maxPt.x, m_minPt.y, m_maxPt.z);
	  glVertex3f(m_maxPt.x, m_maxPt.y, m_maxPt.z);
	  glVertex3f(m_minPt.x, m_maxPt.y, m_maxPt.z);
//	  glVertex3f(-1.0f, -1.0f, 0.0f); // The bottom left corner
//	  glVertex3f(-1.0f, 1.0f, 0.0f); // The top left corner
//	  glVertex3f(1.0f, 1.0f, 0.0f); // The top right corner
//	  glVertex3f(1.0f, -1.0f, 0.0f); // The bottom right corner
	  glEnd();

	  glBegin(GL_LINES);
	  glVertex3f(m_minPt.x, m_minPt.y, m_minPt.z);
	  glVertex3f(m_minPt.x, m_minPt.y, m_maxPt.z);

	  glVertex3f(m_maxPt.x, m_minPt.y, m_minPt.z);
	  glVertex3f(m_maxPt.x, m_minPt.y, m_maxPt.z);

	  glVertex3f(m_maxPt.x, m_maxPt.y, m_minPt.z);
	  glVertex3f(m_maxPt.x, m_maxPt.y, m_maxPt.z);

	  glVertex3f(m_minPt.x, m_maxPt.y, m_minPt.z);
	  glVertex3f(m_minPt.x, m_maxPt.y, m_maxPt.z);
	  glEnd();

	  glDisable(GL_LINE_SMOOTH);
	  glEnable(GL_LIGHTING);


	  // correct all arrow frames:

	  for (unsigned i = 0; i < m_frames.size(); ++i){
	    qglviewer::Vec pt = m_minPt;
	    float corr = m_arrowLength/2;
	    if (i/3 == 1){
	      pt = m_maxPt;
	      corr *= -1;
	    }

	    pt[i%3] += corr;

	    frame(i)->setTranslation(pt);

	  }

	  // draw spheres in their frames:
//	  GLUquadricObj* quadric=gluNewQuadric();
//	  gluQuadricNormals(quadric, GLU_SMOOTH);
//      glColor4f(1.0, 0.0, 0.0, 1.0);

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_COLOR_MATERIAL);
	  for (unsigned i = 0; i < m_frames.size(); ++i){
	    glPushMatrix();
	    glMultMatrixd(m_frames[i]->matrix());
	    if (withNames)
	      glPushName(i);

	    float length = m_arrowLength;
	    if (frame(i)->grabsMouse())
	      length *= 2;

	    const float radius = length/20;

	    float color[4];
	    if (i%3 == 0){ // x
        color[0] = 1.0f;  color[1] = 0.7f;  color[2] = 0.7f;  color[3] = 1.0f;
        glPushMatrix();
        glRotatef(90.0, 0.0, 1.0, 0.0);

	    } else if (i%3 == 1){ // y
	      color[0] = 0.7f;  color[1] = 1.0f;  color[2] = 0.7f;  color[3] = 1.0f;
	      glPushMatrix();
	      glRotatef(-90.0, 1.0, 0.0, 0.0);

	    } else {  // z
	      glPushMatrix();
	      color[0] = 0.7f;  color[1] = 0.7f;  color[2] = 1.0f;  color[3] = 1.0f;
	    }
	    glTranslatef(0.0, 0.0, -length/2.0);
	    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	    QGLViewer::drawArrow(length, radius);
	    glPopMatrix();



	    if (withNames)
	      glPopName();

	    glPopMatrix();
	  }
	  if (colorMaterial)
	    glEnable(GL_COLOR_MATERIAL);
	  if (!lighting)
	    glDisable(GL_LIGHTING);


	  //gluDeleteQuadric(quadric);

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

  int SelectionBox::getGrabbedFrame() const {
    int frameid = -1;
    for (unsigned i = 0; i < m_frames.size(); ++i){
      if (frame(i)->grabsMouse()){
        frameid = i;
        break;
      }
    }

    return frameid;
  }


  void SelectionBox::drawAxis(float length) const
  {
	const float radius = length/20;

	GLboolean lighting, colorMaterial;
	glGetBooleanv(GL_LIGHTING, &lighting);
	glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

	glDisable(GL_COLOR_MATERIAL);

	float color[4];
	color[0] = 0.7f;  color[1] = 0.7f;  color[2] = 1.0f;  color[3] = 1.0f;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	QGLViewer::drawArrow(length, radius);

	color[0] = 1.0f;  color[1] = 0.7f;  color[2] = 0.7f;  color[3] = 1.0f;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	glPushMatrix();
	glRotatef(90.0, 0.0, 1.0, 0.0);
	QGLViewer::drawArrow(length, radius);
	glPopMatrix();

	color[0] = 0.7f;  color[1] = 1.0f;  color[2] = 0.7f;  color[3] = 1.0f;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	QGLViewer::drawArrow(length, radius);
	glPopMatrix();

	if (colorMaterial)
	  glEnable(GL_COLOR_MATERIAL);
	if (!lighting)
	  glDisable(GL_LIGHTING);
  }


}
