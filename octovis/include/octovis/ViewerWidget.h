// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2011.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
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
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef VIEWERWIDGET_H_
#define VIEWERWIDGET_H_

#include "SceneObject.h"
#include "SelectionBox.h"
#include <octomap/octomap.h>
#include <qglviewer.h>

namespace octomap{

class ViewerWidget : public QGLViewer {
  Q_OBJECT

 public:

  ViewerWidget(QWidget* parent = NULL);
  void clearAll();

  /**
   * Adds an object to the scene that can be drawn
   *
   * @param obj SceneObject to be added
   */
  void addSceneObject(SceneObject* obj);

  /**
   * Removes a SceneObject from the list of drawable objects if
   * it has been added previously. Does nothing otherwise.
   *
   * @param obj SceneObject to be removed
   */
  void removeSceneObject(SceneObject* obj);

 public slots:
  void enablePrintoutMode (bool enabled = true);
  void enableHeightColorMode (bool enabled = true);
  void enableSemanticColoring (bool enabled = true);
  void enableSelectionBox (bool enabled = true);
  void setCamPosition(double x, double y, double z, double lookX, double lookY, double lookZ);
  void setCamPose(const octomath::Pose6D& pose);
  virtual void setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max);
  void deleteCameraPath(int id);
  void appendToCameraPath(int id, const octomath::Pose6D& pose);
  void appendCurrentToCameraPath(int id);
  void addCurrentToCameraPath(int id, int frame);
  void removeFromCameraPath(int id, int frame);
  void updateCameraPath(int id, int frame);
  void jumpToCamFrame(int id, int frame);
  void playCameraPath(int id, int start_frame);
  void stopCameraPath(int id);
  const SelectionBox& selectionBox() const { return m_selectionBox;}

  /**
   * Resets the 3D viewpoint to the initial value
   */
  void resetView();

private slots:
   void cameraPathFinished();
   void cameraPathInterpolated();

signals:
   void cameraPathStopped(int id);
   void cameraPathFrameChanged(int id, int current_camera_frame);

 protected:

  virtual void draw();
  virtual void drawWithNames();
  virtual void init();
  /**
   * Overloaded from QGLViewer. Draws own axis and grid in scale, then calls QGLViewer::postDraw().
   */
  virtual void postDraw();
  virtual void postSelection(const QPoint&);
  virtual QString helpString() const;

  qglviewer::Quaternion poseToQGLQuaternion(const octomath::Pose6D& pose);

  std::vector<SceneObject*> m_sceneObjects;
  SelectionBox m_selectionBox;

  bool m_printoutMode;
  bool m_heightColorMode;
  bool m_semantic_coloring;

  bool m_drawAxis; // actual state of axis (original overwritten)
  bool m_drawGrid; // actual state of grid (original overwritten)
  bool m_drawSelectionBox;

  double m_zMin;
  double m_zMax;

  int m_current_camera_path;
  int m_current_camera_frame;
};

}

#endif /* VIEWERWIDGET_H_ */
