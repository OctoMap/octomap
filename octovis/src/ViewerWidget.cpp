/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <octovis/ViewerWidget.h>
#include <manipulatedCameraFrame.h>

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192E0
#endif

using namespace std;

namespace octomap {

ViewerWidget::ViewerWidget(QWidget* parent) :
        QGLViewer(parent), m_zMin(0.0),m_zMax(1.0) {

  m_printoutMode = false;
  m_heightColorMode = false;
  m_semantic_coloring = false;
  m_drawSelectionBox = false;
}

void ViewerWidget::init() {

  //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::AltModifier);
  //    setHandlerKeyboardModifiers(QGLViewer::FRAME, Qt::NoModifier);
  //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::ControlModifier);
  setMouseTracking(true);

  // Restore previous viewer state.
  restoreStateFromFile();

  // Make camera the default manipulated frame.
  setManipulatedFrame( camera()->frame() );
  // invert mousewheel (more like Blender)
  camera()->frame()->setWheelSensitivity(-1.0);


  // Light initialization:
  glEnable(GL_LIGHT0);

  float pos[4] = {-1.0, 1.0, 1.0, 0.0};
  // Directional light
  glLightfv(GL_LIGHT0, GL_POSITION, pos);

  // background color defaults to white
  this->setBackgroundColor( QColor(255,255,255) );
}

void ViewerWidget::resetView(){
  this->camera()->setOrientation((float) -M_PI_2, (float) M_PI_2);
  this->showEntireScene();
  update();
}


QString ViewerWidget::helpString() const{
  QString help = "<h2>Octomap 3D viewer</h2>";

  help +="The Octomap library implements a 3D occupancy grid mapping approach. "
      "It provides data structures and mapping algorithms. The map is implemented "
      "using an octree. 3D maps can be viewed an built using this 3D viewer."
      "<br/><br/>"
      "Octomap is available at https://octomap.github.io, and is actively "
      "maintained by Kai M. Wurm and Armin Hornung. This 3D viewer is based on "
      "libQGLViewer, available at http://www.libqglviewer.com/."
      "<br/><br/>"
      "Please refer to the \"Keyboard\" and \"Mouse\" tabs for instructions on "
      "how to use the viewer.";
  return help;
}

void ViewerWidget::enableHeightColorMode (bool enabled) {
  m_heightColorMode = enabled;
  for(std::vector<SceneObject*>::iterator it = m_sceneObjects.begin(); it != m_sceneObjects.end(); it++) {
    (*it)->enableHeightColorMode(enabled);
  }
  update();
}

void ViewerWidget::enablePrintoutMode(bool enabled) {
  m_printoutMode = enabled;
  for(std::vector<SceneObject*>::iterator it = m_sceneObjects.begin(); it != m_sceneObjects.end(); it++) {
    (*it)->enablePrintoutMode(enabled);
  }
  update();
}

void ViewerWidget::enableSemanticColoring (bool enabled) {
  m_semantic_coloring = enabled;
  for(std::vector<SceneObject*>::iterator it = m_sceneObjects.begin(); it != m_sceneObjects.end(); it++) {
    (*it)->enableSemanticColoring(enabled);
  }
  update();
}

void ViewerWidget::enableSelectionBox(bool enabled) {
  m_drawSelectionBox = enabled;
  update();
}


qglviewer::Quaternion ViewerWidget::poseToQGLQuaternion(const octomath::Pose6D& pose) {
  // copying octomap::Quaternion parameters to qglviewer::Quaternion does not work (reason unknown)
  // octomath::Quaternion quaternion = pose.rot().normalized();
  // return qglviewer::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.u());

  // Compute viewing direction and use code from libqglviewer's "look at" function
  octomath::Vector3 dir = pose.rot().rotate(octomath::Vector3(1.,0.,0.));
  qglviewer::Vec direction(dir.x(), dir.y(), dir.z());
  //qglviewer::Vec xAxis = direction ^ camera()->upVector();
  // useing 0, 0, 1 as upvector instead:
  qglviewer::Vec xAxis = direction ^ qglviewer::Vec(0.0, 0.0, 1.0);

  qglviewer::Quaternion q;
  q.setFromRotatedBasis(xAxis, xAxis^direction, -direction);
  return q;
}

void ViewerWidget::setCamPosition(double x, double y, double z, double lookX, double lookY, double lookZ){
  this->camera()->setOrientation(-M_PI/2., M_PI/2.);
  camera()->setPosition(qglviewer::Vec(x, y, z));
  camera()->lookAt(qglviewer::Vec(lookX, lookY, lookZ));
  camera()->setUpVector(qglviewer::Vec(0.0, 0.0, 1.0));
  update();
}

void ViewerWidget::setCamPose(const octomath::Pose6D& pose){
  octomath::Pose6D ahead = pose * octomath::Pose6D(octomath::Vector3(1,0,0), octomath::Quaternion());
  setCamPosition(pose.x(), pose.y(), pose.z(), ahead.x(), ahead.y(), ahead.z());
}

void ViewerWidget::jumpToCamFrame(int id, int frame) {
  qglviewer::KeyFrameInterpolator *kfi = camera()->keyFrameInterpolator(id);
  if(kfi && frame >= 0 && frame < kfi->numberOfKeyFrames()) {
    camera()->setPosition(kfi->keyFrame(frame).position());
    camera()->setOrientation(kfi->keyFrame(frame).orientation());
  } else {
    std::cerr << "Error: Could not jump to frame " << frame << " of " << kfi->numberOfKeyFrames() << std::endl;
  }
  update();
}

void ViewerWidget::deleteCameraPath(int id) {
  if(camera()->keyFrameInterpolator(id)) {
    disconnect(camera()->keyFrameInterpolator(id), SIGNAL(interpolated()), this, SLOT(update()));
    disconnect(camera()->keyFrameInterpolator(id), SIGNAL(interpolated()), this, SLOT(cameraPathInterpolated()));
    disconnect(camera()->keyFrameInterpolator(id), SIGNAL(endReached()), this, SLOT(cameraPathFinished()));
    camera()->deletePath(id);
  }
}

void ViewerWidget::appendToCameraPath(int id, const octomath::Pose6D& pose) {
  qglviewer::Vec position(pose.trans().x(), pose.trans().y(), pose.trans().z());
  qglviewer::Quaternion quaternion = poseToQGLQuaternion(pose);
  qglviewer::Frame frame(position, quaternion);
  if(!camera()->keyFrameInterpolator(id)) {
    camera()->setKeyFrameInterpolator(id, new qglviewer::KeyFrameInterpolator(camera()->frame()));
  }
  camera()->keyFrameInterpolator(id)->addKeyFrame(frame);
}

void ViewerWidget::removeFromCameraPath(int id, int frame) {
  qglviewer::KeyFrameInterpolator *old_kfi = camera()->keyFrameInterpolator(id);
  if(old_kfi) {
    qglviewer::KeyFrameInterpolator *new_kfi = new qglviewer::KeyFrameInterpolator(camera()->frame());
    for(int i = 0; i < old_kfi->numberOfKeyFrames(); i++) {
      if(i != frame) {
        new_kfi->addKeyFrame(old_kfi->keyFrame(i));
      }
    }
    deleteCameraPath(id);
    camera()->setKeyFrameInterpolator(id, new_kfi);
  }
}

void ViewerWidget::updateCameraPath(int id, int frame) {
  qglviewer::KeyFrameInterpolator *old_kfi = camera()->keyFrameInterpolator(id);
  if(old_kfi) {
    qglviewer::KeyFrameInterpolator *new_kfi = new qglviewer::KeyFrameInterpolator(camera()->frame());
    for(int i = 0; i < old_kfi->numberOfKeyFrames(); i++) {
      if(i != frame) {
        new_kfi->addKeyFrame(old_kfi->keyFrame(i));
      } else {
        new_kfi->addKeyFrame(*(camera()->frame()));
      }
    }
    deleteCameraPath(id);
    camera()->setKeyFrameInterpolator(id, new_kfi);
  }
}

void ViewerWidget::appendCurrentToCameraPath(int id) {
  int frame = 0;
  if(camera()->keyFrameInterpolator(id)) frame = camera()->keyFrameInterpolator(id)->numberOfKeyFrames();
  addCurrentToCameraPath(id, frame);
}

void ViewerWidget::addCurrentToCameraPath(int id, int frame) {
  qglviewer::KeyFrameInterpolator *old_kfi = camera()->keyFrameInterpolator(id);
  if(!old_kfi || frame >= old_kfi->numberOfKeyFrames()) {
    camera()->addKeyFrameToPath(id);
  } else {
    qglviewer::KeyFrameInterpolator *new_kfi = new qglviewer::KeyFrameInterpolator(camera()->frame());
    for(int i = 0; i < old_kfi->numberOfKeyFrames(); i++) {
      new_kfi->addKeyFrame(old_kfi->keyFrame(i));
      if(i == frame) {
        new_kfi->addKeyFrame(camera()->frame());
      }
    }
    deleteCameraPath(id);
    camera()->setKeyFrameInterpolator(id, new_kfi);
  }
}

void ViewerWidget::playCameraPath(int id, int start_frame) {
  qglviewer::KeyFrameInterpolator *kfi = camera()->keyFrameInterpolator(id);
  if(kfi && !kfi->interpolationIsStarted() && start_frame >= 0 && start_frame < kfi->numberOfKeyFrames()) {
    m_current_camera_path = id;
    m_current_camera_frame = start_frame;
    kfi->setInterpolationTime(kfi->keyFrameTime(start_frame));
    std::cout << "Playing path of length " << kfi->numberOfKeyFrames() << ", start time " << kfi->keyFrameTime(start_frame) << std::endl;
    connect(kfi, SIGNAL(interpolated()), this, SLOT(update()));
    connect(kfi, SIGNAL(interpolated()), this, SLOT(cameraPathInterpolated()));
    connect(kfi, SIGNAL(endReached()), this, SLOT(cameraPathFinished()));
    kfi->startInterpolation();
  }
}

void ViewerWidget::stopCameraPath(int id) {
  if(camera()->keyFrameInterpolator(id) && camera()->keyFrameInterpolator(id)->interpolationIsStarted()) {
    disconnect(camera()->keyFrameInterpolator(id), SIGNAL(interpolated()), this, SLOT(update()));
    disconnect(camera()->keyFrameInterpolator(id), SIGNAL(interpolated()), this, SLOT(cameraPathInterpolated()));
    disconnect(camera()->keyFrameInterpolator(id), SIGNAL(endReached()), this, SLOT(cameraPathFinished()));
    camera()->keyFrameInterpolator(id)->stopInterpolation();
  }
}

void ViewerWidget::cameraPathFinished() {
  if(camera()->keyFrameInterpolator(m_current_camera_path)) {
    disconnect(camera()->keyFrameInterpolator(m_current_camera_path), SIGNAL(interpolated()), this, SLOT(update()));
    disconnect(camera()->keyFrameInterpolator(m_current_camera_path), SIGNAL(interpolated()), this, SLOT(cameraPathInterpolated()));
    disconnect(camera()->keyFrameInterpolator(m_current_camera_path), SIGNAL(endReached()), this, SLOT(cameraPathFinished()));
    emit cameraPathStopped(m_current_camera_path);
  }
}

void ViewerWidget::cameraPathInterpolated() {
  qglviewer::KeyFrameInterpolator *kfi = camera()->keyFrameInterpolator(m_current_camera_path);
  if(kfi) {
    int current_frame = m_current_camera_frame;
    for(int i = m_current_camera_frame + 1; i < kfi->numberOfKeyFrames(); i++) {
      if(kfi->keyFrameTime(current_frame) <= kfi->interpolationTime()) current_frame = i;
      else break;
    }
    if(current_frame != m_current_camera_frame) {
      m_current_camera_frame = current_frame;
      emit cameraPathFrameChanged(m_current_camera_path, m_current_camera_frame);
    }
  }
}

void ViewerWidget::setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max){
  m_zMin = min[2];
  m_zMax = max[2];
  QGLViewer::setSceneBoundingBox(min, max);
}

void ViewerWidget::addSceneObject(SceneObject* obj){
  assert (obj);
  m_sceneObjects.push_back(obj);
  update();
}

void ViewerWidget::removeSceneObject(SceneObject* obj){
  assert(obj);
  for(std::vector<SceneObject*>::iterator it = m_sceneObjects.begin();
      it != m_sceneObjects.end();){
    if (*it == obj)
      it = m_sceneObjects.erase(it);
    else
      ++it;
  }
  update();
}

void ViewerWidget::clearAll(){
  // clear drawable objects:
  for(std::vector<SceneObject*>::iterator it = m_sceneObjects.begin(); it != m_sceneObjects.end(); it++){
    (*it)->clear();
  }
}

void ViewerWidget::draw(){

  // debugging: draw light in scene
  //drawLight(GL_LIGHT0);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_LIGHTING);
  if (m_printoutMode){
    glCullFace(GL_BACK);
  }

  // draw drawable objects:
  for(std::vector<SceneObject*>::iterator it = m_sceneObjects.begin();
      it != m_sceneObjects.end(); ++it){
    (*it)->draw();
  }

  if (m_drawSelectionBox){
    m_selectionBox.draw();

    if (m_selectionBox.getGrabbedFrame() >= 0){
      setMouseBinding(Qt::LeftButton, FRAME, TRANSLATE);
    } else {
      setMouseBinding(Qt::LeftButton, FRAME, ROTATE);
    }

  }

}

void ViewerWidget::drawWithNames(){

  if (m_drawSelectionBox)
    m_selectionBox.draw(true);
}

void ViewerWidget::postDraw(){

  // Reset model view matrix to world coordinates origin
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  camera()->loadModelViewMatrix();
  // TODO restore model loadProjectionMatrixStereo

  // Save OpenGL state
  glPushAttrib(GL_ALL_ATTRIB_BITS);

  glDisable(GL_COLOR_MATERIAL);

  if (gridIsDrawn()){
    glLineWidth(1.0);
    drawGrid(5.0, 10);
  }
  if (axisIsDrawn()){
    glLineWidth(2.0);
    drawAxis(1.0);
  }

  // Restore GL state
  glPopAttrib();
  glPopMatrix();

  m_drawAxis = axisIsDrawn();
  m_drawGrid = gridIsDrawn();
  setAxisIsDrawn(false);
  setGridIsDrawn(false);
  QGLViewer::postDraw();

  setAxisIsDrawn(m_drawAxis);
  setGridIsDrawn(m_drawGrid);
}

void ViewerWidget::postSelection(const QPoint&)
{

}



} // namespace

