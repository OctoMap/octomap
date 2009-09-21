/*
 * ViewerWidget.h
 *
 *  Created on: May 28, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#ifndef VIEWERWIDGET_H_
#define VIEWERWIDGET_H_

#include <octomap.h>
#include <boost/shared_ptr.hpp>
#include <QGLViewer/qglviewer.h>

class ViewerWidget : public QGLViewer {
  Q_OBJECT

 public:

  ViewerWidget(QWidget* parent = NULL);
  void clearOcTree();
  void clearOcTreeStructure();
  void clearTrajectory();
  void clearPointcloud();
  void clearAll();


  /**
   * Sets a new pointcloud, adjusts view & boundingbox to contain all points
   *
   * @param _cloud
   */
  void setPointcloud(boost::shared_ptr<octomap::Pointcloud> _cloud);

  void setOcTreeVoxels(std::list<octomap::OcTreeVolume>& occupied_voxels,
		       std::list<octomap::OcTreeVolume>& freespace_voxels,
		       std::list<octomap::OcTreeVolume>& occupied_delta_voxels,
		       std::list<octomap::OcTreeVolume>& freespace_delta_voxels,
		       std::list<octomap::OcTreeVolume>& grid_voxels,
		       std::list<octomap::OcTreeVolume>& changed_free_voxels);

  void setTrajectory(std::vector<octomap::point3d>& traj);

  /*!
   * Assigns a new ScanGraph to the viewer. This will update the internal
   * PointCloud from the graph (by calling setPointcloud())
   *
   * @param _graph
   */
  void setScanGraph(boost::shared_ptr<octomap::ScanGraph> _graph);

 public slots:

  void enableOcTree(bool enabled = true){m_drawOcTreeGrid = enabled; updateGL();};
  void enableOcTreeCells(bool enabled = true){m_drawOcTreeCells = enabled; updateGL();};
  void enablePointcloud(bool enabled = true){m_drawPointcloud = enabled; updateGL();};
  void enableTrajectory (bool enabled = true){ m_show_trajectory = enabled; updateGL();};
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

 private:

  virtual void draw();
  virtual void init();
  void drawOctreeGrid();
  void drawOctreeCells() const;
  void drawFreespace() const;
  void drawPointcloud() const;
  void drawTrajectory() const;
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

  std::list<octomap::OcTreeVolume> m_grid_voxels;

  bool m_drawOcTreeCells;
  bool m_drawOcTreeGrid;
  bool m_drawPointcloud;
  bool m_printoutMode;
  bool m_heightColorMode;
  bool m_show_trajectory;
  bool m_draw_freespace;
  bool m_draw_freespaceDeltaOnly;

  bool m_octree_grid_vis_initialized;

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

  //! OpenGL representation of Pointcloud
  GLfloat*     pointcloud_points_array;
  unsigned int pointcloud_points_size;

  GLfloat* trajectory_vertex_array;
  GLfloat* trajectory_color_array;
  unsigned int trajectory_size;

  double m_zMin;
  double m_zMax;

};


#endif /* VIEWERWIDGET_H_ */
