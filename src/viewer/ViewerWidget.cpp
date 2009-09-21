/*
 * ViewerWidget.cpp
 *
 *  Created on: May 28, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#include "ViewerWidget.h"

using namespace std;


ViewerWidget::ViewerWidget(QWidget* parent) :
  QGLViewer(parent),
  octree_occupied_cells_vertex_size(0), octree_freespace_cells_vertex_size(0),
  octree_occupied_delta_cells_vertex_size(0), octree_freespace_delta_cells_vertex_size(0),
  octree_freespace_changed_cells_vertex_size(0),octree_grid_vertex_size(0),  pointcloud_points_size(0),
  trajectory_vertex_array(NULL), trajectory_color_array(NULL), trajectory_size(0),m_zMin(0.0),m_zMax(1.0)
{
	m_drawOcTreeCells = true;
	m_drawPointcloud = true;
	m_drawOcTreeGrid = false;
	m_octree_grid_vis_initialized = false;
	m_show_trajectory = false;
	m_draw_freespace = false;
	m_draw_freespaceDeltaOnly = false;
	m_printoutMode = false;
	m_heightColorMode = false;

	//connect(this, SIGNAL(cameraIsEditedChanged(bool)), this, SLOT(cameraEdited(bool)));


	// TODO: have colors in named arrays

}

void ViewerWidget::init() {
  // Restore previous viewer state.
  restoreStateFromFile();

  // Light initialization:
  glEnable(GL_LIGHT0);

  float pos[4] = {-1.0, -1.0, 1.0, 0.0};
  // Directional light
  glLightfv(GL_LIGHT0, GL_POSITION, pos);
}

void ViewerWidget::resetView(){
  this->showEntireScene();
  updateGL();

}

void ViewerWidget::enableHeightColorMode (bool enabled){
  m_heightColorMode = enabled;
}

void ViewerWidget::setCamPosition(double x, double y, double z, double lookX, double lookY, double lookZ){
  this->camera()->setOrientation(-M_PI/2., M_PI/2.);
  camera()->setPosition(qglviewer::Vec(x, y, z));
  camera()->lookAt(qglviewer::Vec(lookX, lookY, lookZ));
  updateGL();
}

void ViewerWidget::setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max){
  m_zMin = min[2];
  m_zMax = max[2];
  QGLViewer::setSceneBoundingBox(min, max);
}


void ViewerWidget::generateCubes(const std::list<octomap::OcTreeVolume>& voxels, GLfloat** gl_array, GLfloat* gl_color_array) {

  // generate the cubes, 6 quads each
  // min and max-values are computed on-the-fly
  unsigned int i = 0;
  unsigned int colorIdx = 0;
  double x,y,z;

  for (std::list<octomap::OcTreeVolume>::const_iterator it=voxels.begin(); it != voxels.end(); it++) {

    double half_cube_size = GLfloat(it->second /2.0);

    x = it->first.x();
    y = it->first.y();
    z = it->first.z();

    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parrallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    gl_array[0][i]   = x + half_cube_size;
    gl_array[0][i+1] = y + half_cube_size;
    gl_array[0][i+2] = z - half_cube_size;

    gl_array[1][i]   = x + half_cube_size;
    gl_array[1][i+1] = y - half_cube_size;
    gl_array[1][i+2] = z - half_cube_size;

    gl_array[2][i]   = x + half_cube_size;
    gl_array[2][i+1] = y + half_cube_size;
    gl_array[2][i+2] = z - half_cube_size;

    gl_array[3][i]   = x - half_cube_size;
    gl_array[3][i+1] = y + half_cube_size;
    gl_array[3][i+2] = z - half_cube_size;

    gl_array[4][i]   = x + half_cube_size;
    gl_array[4][i+1] = y + half_cube_size;
    gl_array[4][i+2] = z - half_cube_size;

    gl_array[5][i]   = x + half_cube_size;
    gl_array[5][i+1] = y + half_cube_size;
    gl_array[5][i+2] = z + half_cube_size;
    i+= 3;

    gl_array[0][i]   = x - half_cube_size;
    gl_array[0][i+1] = y + half_cube_size;
    gl_array[0][i+2] = z - half_cube_size;

    gl_array[1][i]   = x - half_cube_size;
    gl_array[1][i+1] = y - half_cube_size;
    gl_array[1][i+2] = z - half_cube_size;

    gl_array[2][i]   = x + half_cube_size;
    gl_array[2][i+1] = y + half_cube_size;
    gl_array[2][i+2] = z + half_cube_size;

    gl_array[3][i]   = x - half_cube_size;
    gl_array[3][i+1] = y + half_cube_size;
    gl_array[3][i+2] = z + half_cube_size;

    gl_array[4][i]   = x + half_cube_size;
    gl_array[4][i+1] = y - half_cube_size;
    gl_array[4][i+2] = z - half_cube_size;

    gl_array[5][i]   = x + half_cube_size;
    gl_array[5][i+1] = y - half_cube_size;
    gl_array[5][i+2] = z + half_cube_size;
    i+= 3;

    gl_array[0][i]   = x - half_cube_size;
    gl_array[0][i+1] = y + half_cube_size;
    gl_array[0][i+2] = z + half_cube_size;

    gl_array[1][i]   = x - half_cube_size;
    gl_array[1][i+1] = y - half_cube_size;
    gl_array[1][i+2] = z + half_cube_size;

    gl_array[2][i]   = x + half_cube_size;
    gl_array[2][i+1] = y - half_cube_size;
    gl_array[2][i+2] = z + half_cube_size;

    gl_array[3][i]   = x - half_cube_size;
    gl_array[3][i+1] = y - half_cube_size;
    gl_array[3][i+2] = z + half_cube_size;

    gl_array[4][i]   = x - half_cube_size;
    gl_array[4][i+1] = y - half_cube_size;
    gl_array[4][i+2] = z - half_cube_size;

    gl_array[5][i]   = x - half_cube_size;
    gl_array[5][i+1] = y - half_cube_size;
    gl_array[5][i+2] = z + half_cube_size;
    i+= 3;

    gl_array[0][i]   = x + half_cube_size;
    gl_array[0][i+1] = y + half_cube_size;
    gl_array[0][i+2] = z + half_cube_size;

    gl_array[1][i]   = x + half_cube_size;
    gl_array[1][i+1] = y - half_cube_size;
    gl_array[1][i+2] = z + half_cube_size;

    gl_array[2][i]   = x + half_cube_size;
    gl_array[2][i+1] = y - half_cube_size;
    gl_array[2][i+2] = z - half_cube_size;

    gl_array[3][i]   = x - half_cube_size;
    gl_array[3][i+1] = y - half_cube_size;
    gl_array[3][i+2] = z - half_cube_size;

    gl_array[4][i]   = x - half_cube_size;
    gl_array[4][i+1] = y + half_cube_size;
    gl_array[4][i+2] = z - half_cube_size;

    gl_array[5][i]   = x - half_cube_size;
    gl_array[5][i+1] = y + half_cube_size;
    gl_array[5][i+2] = z + half_cube_size;
    i+= 3;

    if (gl_color_array != NULL){
      // color for 4 vertices (same height)
      for (int k= 0; k < 4; ++k){
        heightMapColor(z, gl_color_array + colorIdx);
        colorIdx +=4;
      }
    }

  }
}

void ViewerWidget::heightMapColor(double h, GLfloat* glArrayPos) const{
  if (m_zMin >= m_zMax)
    h = 0.5;
  else{
    h = (1.0 - std::min(std::max((h-m_zMin)/ (m_zMax - m_zMin), 0.0), 1.0)) *0.8;
  }

  // blue -> red -> yellow
//  glArrayPos[0] = GLfloat(std::min(std::max(4*(height-0.25), 0.), 1.)); // red
//  glArrayPos[1] = GLfloat(std::min(std::max(4*fabs(height-0.5)-1., 0.), 1.)); // green
//  glArrayPos[2] = GLfloat(std::min(std::max(4*(0.75-height), 0.), 1.)); // blue
//  glArrayPos[3] = 0.5; // alpha

  // blend over HSV-values (more colors)
  double r, g, b;
  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:
      r = v; g = n; b = m;
      break;
    case 1:
      r = n; g = v; b = m;
      break;
    case 2:
      r = m; g = v; b = n;
      break;
    case 3:
      r = m; g = n; b = v;
      break;
    case 4:
      r = n; g = m; b = v;
      break;
    case 5:
      r = v; g = m; b = n;
      break;
    default:
      r = 1; g = 0.5; b = 0.5;
      break;
    }

    glArrayPos[0] = r;
    glArrayPos[1] = g;
    glArrayPos[2] = b;
    glArrayPos[3] = 1.0; // alpha
}


void ViewerWidget::initOctreeCubeVis (const std::list<octomap::OcTreeVolume>& occupied_voxels,
				      const std::list<octomap::OcTreeVolume>& freespace_voxels,
				      const std::list<octomap::OcTreeVolume>& occupied_delta_voxels,
				      const std::list<octomap::OcTreeVolume>& freespace_delta_voxels,
				      const std::list<octomap::OcTreeVolume>& changed_free_voxels) {

  clearOcTree();

  // allocate vertex arrays for cubes  -----------------
  octree_occupied_cells_vertex_size = occupied_voxels.size() * 4 * 3;
  octree_occupied_cells_vertex_array = new GLfloat* [6];

  octree_freespace_cells_vertex_size = freespace_voxels.size() * 4 * 3;
  octree_freespace_cells_vertex_array = new GLfloat* [6];

  if (m_draw_freespaceDeltaOnly){
    octree_freespace_delta_cells_vertex_size = changed_free_voxels.size() * 4 * 3;
  } else{
    octree_freespace_delta_cells_vertex_size = freespace_delta_voxels.size() * 4 * 3;
  }

  octree_occupied_delta_cells_vertex_size = occupied_delta_voxels.size() * 4 * 3;
  octree_occupied_delta_cells_vertex_array = new GLfloat* [6];
  octree_freespace_delta_cells_vertex_array = new GLfloat* [6];

  for (unsigned i = 0; i<6; ++i){
    octree_occupied_cells_vertex_array[i] = new GLfloat[octree_occupied_cells_vertex_size];
    octree_freespace_cells_vertex_array[i] = new GLfloat[octree_freespace_cells_vertex_size];
    octree_occupied_delta_cells_vertex_array[i] = new GLfloat[octree_occupied_delta_cells_vertex_size];
    octree_freespace_delta_cells_vertex_array[i] = new GLfloat[octree_freespace_delta_cells_vertex_size];
  }
  octree_occupied_cells_color_array = new GLfloat[occupied_voxels.size() * 4 *4];
  octree_occupied_delta_cells_color_array = new GLfloat[occupied_delta_voxels.size() * 4 *4];

  // ---------------------------------------------------

  generateCubes(occupied_voxels, octree_occupied_cells_vertex_array, octree_occupied_cells_color_array);
  generateCubes(freespace_voxels, octree_freespace_cells_vertex_array);

  generateCubes(occupied_delta_voxels, octree_occupied_delta_cells_vertex_array, octree_occupied_delta_cells_color_array);
  if (m_draw_freespaceDeltaOnly){
    generateCubes(changed_free_voxels, octree_freespace_delta_cells_vertex_array);
  } else{
    generateCubes(freespace_delta_voxels, octree_freespace_delta_cells_vertex_array);
  }
}

void ViewerWidget::setOcTreeVoxels(std::list<octomap::OcTreeVolume>& occupied_voxels,
				   std::list<octomap::OcTreeVolume>& freespace_voxels,
				   std::list<octomap::OcTreeVolume>& occupied_delta_voxels,
				   std::list<octomap::OcTreeVolume>& freespace_delta_voxels,
				   std::list<octomap::OcTreeVolume>& grid_voxels,
				   std::list<octomap::OcTreeVolume>& changed_free_voxels) {

  m_octree_grid_vis_initialized = false;

  // copy tree voxels
  // TODO maybe store them in ViewerGui for each type, then use reference
//   m_occupied_voxels = occupied_voxels;
//   m_freespace_voxels = freespace_voxels;
  m_grid_voxels = grid_voxels;

  initOctreeCubeVis(occupied_voxels, freespace_voxels, occupied_delta_voxels, freespace_delta_voxels, changed_free_voxels);
  initOctreeGridVis();

  updateGL();
}

void ViewerWidget::initOctreeGridVis() {

  if (m_octree_grid_vis_initialized) return;


  clearOcTreeStructure();
  // allocate arrays for octree grid visualization
  octree_grid_vertex_size = m_grid_voxels.size() * 12 * 2 * 3;
  octree_grid_vertex_array = new GLfloat[octree_grid_vertex_size];


  // generate the cubes, 12 lines each

  std::list<octomap::OcTreeVolume>::iterator it_rec;
  unsigned int i = 0;
  double x,y,z;
  for (it_rec=m_grid_voxels.begin(); it_rec != m_grid_voxels.end(); it_rec++) {

    x = it_rec->first.x();
    y = it_rec->first.y();
    z = it_rec->first.z();

    double half_voxel_size = it_rec->second / 2.0;

    // ----
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    // ----
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    // ----
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z - half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x - half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y + half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    octree_grid_vertex_array[i]   = x + half_voxel_size;
    octree_grid_vertex_array[i+1] = y - half_voxel_size;
    octree_grid_vertex_array[i+2] = z + half_voxel_size;
    i+= 3;
    // ----
  }

  m_octree_grid_vis_initialized = true;
}

void ViewerWidget::setPointcloud(boost::shared_ptr<octomap::Pointcloud> _cloud){

  // prepare points
  if (pointcloud_points_size != 0) {
    delete[] pointcloud_points_array;
  }

  pointcloud_points_size = _cloud->size()* 3;
  pointcloud_points_array = new GLfloat[pointcloud_points_size];

  double min_x, min_y, min_z;
  double max_x, max_y, max_z;
  min_x = min_y = min_z = 1e6;
  max_x = max_y = max_z = -1e6;
  double x, y, z;

  octomap::Pointcloud::iterator it;
  unsigned int i = 0;
  for (it = _cloud->begin(); it != _cloud->end(); ++it){
    x = (*it)->x();
    y = (*it)->y();
    z = (*it)->z();

    if (x < min_x) min_x = x;
    if (y < min_y) min_y = y;
    if (z < min_z) min_z = z;

    if (x > max_x) max_x = x;
    if (y > max_y) max_y = y;
    if (z > max_z) max_z = z;

    pointcloud_points_array[i] = x;
    pointcloud_points_array[i+1] = y;
    pointcloud_points_array[i+2] = z;
    i+= 3;
  }

  // adjust view and BBX (also calls setSceneCenter)
  qglviewer::Vec minPos(min_x, min_y, min_z);
  qglviewer::Vec maxPos(max_x, max_y, max_z);

  this->setSceneBoundingBox (minPos,maxPos);
  this->camera()->setPosition(minPos + 0.5*(maxPos-minPos));
  this->camera()->setViewDirection(qglviewer::Vec(-0.2,.0,0));
  this->camera()->setOrientation(-M_PI/2., M_PI/2.);

  this->showEntireScene();

  updateGL();

}

void ViewerWidget::clearPointcloud(){

  if (pointcloud_points_size != 0) {
    delete[] pointcloud_points_array;
    pointcloud_points_size = 0;
  }
}

void ViewerWidget::clearTrajectory(){

  if (trajectory_size != 0) {
    delete[] trajectory_vertex_array;
    delete[] trajectory_color_array;
    trajectory_size = 0;
  }
}

void ViewerWidget::clearOcTree(){

  if (octree_occupied_cells_vertex_size != 0) {
    for (unsigned i = 0; i<6; ++i){
      delete[] octree_occupied_cells_vertex_array[i];
    }
    delete[] octree_occupied_cells_vertex_array;
    if (octree_occupied_delta_cells_color_array){
      delete[] octree_occupied_cells_color_array;
      octree_occupied_cells_color_array = NULL;
    }
    octree_occupied_cells_vertex_size = 0;
  }

  if (octree_freespace_cells_vertex_size != 0) {
    for (unsigned i = 0; i < 6; ++i){
      delete[] octree_freespace_cells_vertex_array[i];
    }
    delete[] octree_freespace_cells_vertex_array;
    octree_freespace_cells_vertex_size = 0;
  }

  if (octree_occupied_delta_cells_vertex_size != 0) {
    for (unsigned i = 0; i < 6; ++i){
      delete[] octree_occupied_delta_cells_vertex_array[i];
    }
    if (octree_occupied_delta_cells_color_array){
      delete[] octree_occupied_delta_cells_color_array;
      octree_occupied_delta_cells_color_array = NULL;
    }

    delete[] octree_occupied_delta_cells_vertex_array;
    octree_occupied_delta_cells_vertex_size = 0;
  }
  if (octree_freespace_delta_cells_vertex_size != 0) {
    for (unsigned i = 0; i < 6; ++i){
      delete[] octree_freespace_delta_cells_vertex_array[i];
    }
    delete[] octree_freespace_delta_cells_vertex_array;
    octree_freespace_delta_cells_vertex_size = 0;
  }

}

void ViewerWidget::clearOcTreeStructure(){
  if (octree_grid_vertex_size != 0) {
    delete[] octree_grid_vertex_array;
    octree_grid_vertex_size = 0;
  }

  m_octree_grid_vis_initialized = false;
}

void ViewerWidget::clearAll(){
  clearPointcloud();
  clearOcTree();
  clearOcTreeStructure();
  clearTrajectory();
}


void ViewerWidget::setTrajectory(std::vector<octomap::point3d>& traj) {

  if (trajectory_size != 0) {
    delete[] trajectory_vertex_array;
    delete[] trajectory_color_array;
  }

  trajectory_size = traj.size();
  trajectory_vertex_array = new GLfloat[trajectory_size * 3];
  trajectory_color_array = new GLfloat[trajectory_size * 4];

  uint i = 0;
  for (std::vector<octomap::point3d>::iterator it = traj.begin(); it != traj.end(); it++) {
    trajectory_vertex_array[i] = it->x();
    trajectory_vertex_array[i+1] = it->y();
    trajectory_vertex_array[i+2] = it->z();
    i+=3;
  }

  for (unsigned int j=0; j < trajectory_size*4; j+=4) {
    trajectory_color_array[j]   = 0.; // r
    trajectory_color_array[j+1] = 0.; // g
    trajectory_color_array[j+2] = 1.; // b
    trajectory_color_array[j+3] = 1.; // alpha
  }

  m_show_trajectory = false;
}


void ViewerWidget::setScanGraph(boost::shared_ptr<octomap::ScanGraph> _graph) {

  // compute pointcloud for the moment
  boost::shared_ptr<octomap::Pointcloud> map(new octomap::Pointcloud());
  for (octomap::ScanGraph::iterator it = _graph->begin(); it != _graph->end(); it++) {
    octomap::Pointcloud* current_scan = new octomap::Pointcloud((*it)->scan);
    current_scan->transformAbsolute((*it)->pose);
    map->push_back(current_scan);  // insert complete scan in pointcloud
    delete current_scan;
  }
  this->setPointcloud(map);

  // set trajectory
  std::vector<octomap::point3d> traj;
  for (octomap::ScanGraph::iterator it = _graph->begin(); it != _graph->end(); it++) {
    traj.push_back( (*it)->pose.trans() );
  }

  this->setTrajectory(traj);
}


void ViewerWidget::draw(){

  // debugging: draw light in scene
  //drawLight(GL_LIGHT0);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // default bg color (printout mode overrides):
 // glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  glEnable(GL_LIGHTING);
  if (m_printoutMode){
    glCullFace(GL_BACK);
  }

  glEnableClientState(GL_VERTEX_ARRAY);

  if (m_show_trajectory)  drawTrajectory();
  if (m_drawOcTreeCells)  drawOctreeCells();
  if (m_draw_freespace && !m_heightColorMode)  drawFreespace();
  if (m_drawPointcloud)   drawPointcloud();
  if (m_drawOcTreeGrid)   drawOctreeGrid();

  glDisableClientState(GL_VERTEX_ARRAY);
}

void ViewerWidget::drawTrajectory() const {
  if (trajectory_size == 0)
    return;

  glEnableClientState(GL_COLOR_ARRAY);
  glLineWidth(3.0f);
  glVertexPointer(3, GL_FLOAT, 0, trajectory_vertex_array);
  glColorPointer(4, GL_FLOAT, 0, trajectory_color_array);
  glDrawArrays(GL_LINE_STRIP, 0, trajectory_size);
  glDisableClientState(GL_COLOR_ARRAY);
}

void ViewerWidget::drawPointcloud() const{
  if (pointcloud_points_size == 0)
    return;

  glEnable(GL_POINT_SMOOTH);

  if (m_printoutMode){
    if (!m_draw_freespace) {
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    }
    glColor4f(0.0, 0.0, 0.0, 1.);
  } else{
    glColor4f(1.0, 0.0, 0.0, 1.);

  }

  glPointSize(1.0);

  glVertexPointer(3, GL_FLOAT, 0, pointcloud_points_array);
  glDrawArrays(GL_POINTS, 0, pointcloud_points_size/3);
}

void ViewerWidget::drawOctreeCells() const {


  // colors for printout mode:
  if (m_printoutMode) {
    if (!m_draw_freespace) { // gray on white background
      glColor3f(0.5f, 0.5f, 0.5f);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    }
    else {
      glColor3f(0.1f, 0.1f, 0.1f);
    }
  }

  // draw binary occupied cells
  if (octree_occupied_cells_vertex_size != 0) {
    if (!m_printoutMode) glColor4f(0.0, 0.0, 1.0, 1.0);
    if (m_draw_freespaceDeltaOnly) glColor4f(0.2, 0.7, 1.0, 1.0);
    drawCubes(octree_occupied_cells_vertex_array, octree_occupied_cells_vertex_size, octree_occupied_cells_color_array);
  }

  // draw delta occupied cells
  if (octree_occupied_delta_cells_vertex_size != 0) {
    if (!m_printoutMode) glColor4f(0.2, 0.7, 1.0, 1.0);
    drawCubes(octree_occupied_delta_cells_vertex_array, octree_occupied_delta_cells_vertex_size, octree_occupied_delta_cells_color_array);
  }

}


void ViewerWidget::drawFreespace() const {

  if (m_printoutMode) {
    if (!m_drawOcTreeCells) { // gray on white background
      glColor3f(0.5f, 0.5f, 0.5f);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    }
    else {
      glColor3f(0.9f, 0.9f, 0.9f);
    }
  }

  // draw binary freespace cells
  if (!m_draw_freespaceDeltaOnly && octree_freespace_cells_vertex_size != 0) {
    if (!m_printoutMode) glColor4f(0.0, 1.0, 0., 0.2);
    drawCubes(octree_freespace_cells_vertex_array, octree_freespace_cells_vertex_size);
  }


  // draw delta freespace cells
  if (octree_freespace_delta_cells_vertex_size != 0) {
    if (!m_printoutMode) glColor4f(0.5, 1.0, 0.1, 0.2);
    if (m_draw_freespaceDeltaOnly) glColor4f(1.0, 0., 0., 1.0);
    drawCubes(octree_freespace_delta_cells_vertex_array, octree_freespace_delta_cells_vertex_size);
  }
}



void ViewerWidget::drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize, GLfloat* cubeColorArray) const {
  if (cubeArraySize == 0)
    return;

  // save current color
  GLfloat* curcol = new GLfloat[4];
  glGetFloatv(GL_CURRENT_COLOR, curcol);

  // enable color pointer when heightColorMode is enabled:

  if (m_heightColorMode && cubeColorArray != NULL){
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(4, GL_FLOAT, 0, cubeColorArray);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  }

  // top surfaces:
  glNormal3f(0.0f, 1.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // bottom surfaces:
  glNormal3f(0.0f, -1.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // right surfaces:
  glNormal3f(1.0f, 0.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // left surfaces:
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // back surfaces:
  glNormal3f(0.0f, 0.0f, -1.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[4]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // front surfaces:
  glNormal3f(0.0f, 0.0f, 1.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[5]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);

  if (m_heightColorMode && cubeColorArray != NULL){
      glDisableClientState(GL_COLOR_ARRAY);
  }

  // draw bounding linies of cubes in printout:
  if (m_printoutMode && ! m_heightColorMode){
    glDisable(GL_LIGHTING);
    glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable (GL_LINE_SMOOTH);
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);   // Draw Polygons only as Wireframes
    glLineWidth(2.0f);
    glColor3f(0.0f, 0.0f, 0.0f);
    glCullFace(GL_FRONT_AND_BACK);        // Don't draw any Polygons faces
    //glDepthFunc (GL_LEQUAL);

    // top meshes:
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // bottom meshes:
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // right meshes:
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    // left meshes:
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
    glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);

    // restore defaults:
    glCullFace(GL_BACK);
    //glDepthFunc(GL_LESS);
    glDisable(GL_LINE_SMOOTH);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);

  }

  // reset color
  glColor4fv(curcol);
  delete[] curcol;
}

void ViewerWidget::drawOctreeGrid() {

  if (!m_octree_grid_vis_initialized) initOctreeGridVis();

  if (octree_grid_vertex_size == 0)
    return;

  glDisable(GL_LIGHTING);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(1.);
  glVertexPointer(3, GL_FLOAT, 0, octree_grid_vertex_array);
  glColor3f(0.0, 0.0, 0.0);
  glDrawArrays(GL_LINES, 0, octree_grid_vertex_size / 3);

  glDisable(GL_LINE_SMOOTH);
  glEnable(GL_LIGHTING);
}
