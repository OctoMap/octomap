// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
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
#include <octomap/octomap.h>
#include <qglviewer.h>

namespace octomap{

class ViewerWidget : public QGLViewer {
  Q_OBJECT

 public:

  ViewerWidget(QWidget* parent = NULL);
  void clearOcTree();
  void clearOcTreeStructure();
  void clearAll();


  /**
   * Sets a new pointcloud, adjusts view and bounding box to contain all points
   *
   * @param _cloud
   */

  void setOcTreeVoxels(std::list<octomap::OcTreeVolume>& occupied_voxels,
		       std::list<octomap::OcTreeVolume>& freespace_voxels,
		       std::list<octomap::OcTreeVolume>& occupied_delta_voxels,
		       std::list<octomap::OcTreeVolume>& freespace_delta_voxels,
		       std::list<octomap::OcTreeVolume>& grid_voxels,
		       std::list<octomap::OcTreeVolume>& changed_free_voxels);

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

  void enableOcTree(bool enabled = true){m_drawOcTreeGrid = enabled; updateGL();};
  void enableOcTreeCells(bool enabled = true){m_drawOcTreeCells = enabled; updateGL();};
  void enableFreespace (bool enabled = true){ m_draw_freespace = enabled; updateGL();};
  void enableFreespaceDeltaOnly (bool enabled = true){ m_draw_freespaceDeltaOnly = enabled; updateGL();};
  void enablePrintoutMode (bool enabled = true){m_printoutMode = enabled; updateGL();};
  void enableHeightColorMode (bool enabled = true);
  void setCamPosition(double x, double y, double z, double lookX, double lookY, double lookZ);
  virtual void setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max);

  /**
   * Resets the 3D viewpoint to the initial value
   */
  void resetView();

 protected:

  virtual void draw();
  virtual void init();
  /**
   * Overloaded from QGLViewer. Draws own axis and grid in scale, then calls QGLViewer::postDraw().
   */
  virtual void postDraw();
  virtual QString helpString() const;
  void drawOctreeGrid();
  void drawOctreeCells() const;
  void drawFreespace() const;
  void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize, GLfloat* cubeColorArray = NULL) const;

  //! setup octree visualizations
  void initOctreeCubeVis (const std::list<octomap::OcTreeVolume>& occupied_voxels,
			  const std::list<octomap::OcTreeVolume>& freespace_voxels,
			  const std::list<octomap::OcTreeVolume>& occupied_delta_voxels,
			  const std::list<octomap::OcTreeVolume>& freespace_delta_voxels,
			  const std::list<octomap::OcTreeVolume>& changed_free_voxels);

  void generateCubes (const std::list<octomap::OcTreeVolume>& points, GLfloat** gl_array, GLfloat* gl_color_array = NULL);
  void initOctreeGridVis();

  void heightMapColor(double height, GLfloat* glArrayPos) const;

  std::vector<SceneObject*> m_sceneObjects;

  std::list<octomap::OcTreeVolume> m_grid_voxels;

  bool m_drawOcTreeCells;
  bool m_drawOcTreeGrid;
  bool m_printoutMode;
  bool m_heightColorMode;
  bool m_draw_freespace;
  bool m_draw_freespaceDeltaOnly;

  bool m_octree_grid_vis_initialized;
  bool m_drawAxis; // actual state of axis (original overwritten)
  bool m_drawGrid; // actual state of grid (original overwritten)

  //! OpenGL representation of Octree cells (cubes)

  GLfloat**     octree_occupied_cells_vertex_array;
  unsigned int octree_occupied_cells_vertex_size;
  GLfloat**     octree_freespace_cells_vertex_array;
  unsigned int octree_freespace_cells_vertex_size;
  GLfloat**     octree_occupied_delta_cells_vertex_array;
  unsigned int octree_occupied_delta_cells_vertex_size;
  GLfloat**     octree_freespace_delta_cells_vertex_array;
  unsigned int octree_freespace_delta_cells_vertex_size;
  GLfloat**     octree_freespace_changed_cells_vertex_array;
  unsigned int octree_freespace_changed_cells_vertex_size;

  //! Color array for occupied cells (height)
  GLfloat* octree_occupied_cells_color_array;
  GLfloat* octree_occupied_delta_cells_color_array;

  //! OpenGL representation of Octree (grid structure)
  GLfloat*     octree_grid_vertex_array;
  unsigned int octree_grid_vertex_size;

  double m_zMin;
  double m_zMax;

};

}

#endif /* VIEWERWIDGET_H_ */
