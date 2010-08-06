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

#include "OcTreeDrawer.h"

namespace octomap {

OcTreeDrawer::OcTreeDrawer() : SceneObject(),
  m_occupiedThresSize(0), m_freeThresSize(0),
  m_occupiedSize(0), m_freeSize(0),
  octree_grid_vertex_size(0), m_alphaOccupied(0.8)
{
  m_octree_grid_vis_initialized = false;
  m_drawOccupied = true;
  m_drawOcTreeGrid = false;
  m_drawFree = false;

  m_occupiedArray = NULL;
  m_freeArray = NULL;
  m_occupiedThresArray = NULL;
  m_freeThresArray = NULL;
  m_occupiedColorArray = NULL;
  m_occupiedThresColorArray = NULL;
}

OcTreeDrawer::~OcTreeDrawer() {
  clear();
}

void OcTreeDrawer::setAlphaOccupied(double alpha){
  m_alphaOccupied = alpha;
}

void OcTreeDrawer::setOcTree(const octomap::OcTree& octree) {
  std::list<octomap::OcTreeVolume> occupiedThresVoxels;
  std::list<octomap::OcTreeVolume> freeThresVoxels;
  std::list<octomap::OcTreeVolume> occupiedVoxels;
  std::list<octomap::OcTreeVolume> freeVoxels;

  octree.getOccupied(occupiedThresVoxels, occupiedVoxels, m_max_tree_depth);

  if (octree.size() < 5 * 1e6) {
    octree.getFreespace (freeThresVoxels, freeVoxels, m_max_tree_depth);
    octree.getVoxels(m_grid_voxels, m_max_tree_depth-1); // octree structure not drawn at lowest level
  }

  double minX, minY, minZ, maxX, maxY, maxZ;
  octree.getMetricMin(minX, minY, minZ);
  octree.getMetricMax(maxX, maxY, maxZ);

  m_zMin = minZ;
  m_zMax = maxZ;


  m_octree_grid_vis_initialized = false;
  if(m_drawOcTreeGrid) initOctreeGridVis();

  // initialize visualization:
  generateCubes(occupiedThresVoxels, &m_occupiedThresArray, m_occupiedThresSize, &m_occupiedThresColorArray);
  generateCubes(freeThresVoxels, &m_freeThresArray, m_freeThresSize);

  generateCubes(occupiedVoxels, &m_occupiedArray, m_occupiedSize, &m_occupiedColorArray);
  generateCubes(freeVoxels, &m_freeArray, m_freeSize);


}

void OcTreeDrawer::generateCubes(const std::list<octomap::OcTreeVolume>& voxels,
    GLfloat*** glArray, unsigned int& glArraySize, GLfloat** glColorArray)
{


  // clear arrays first if needed:
  clearCubes(glArray, glArraySize, glColorArray);

  // now, allocate arrays:
  glArraySize = voxels.size() * 4 * 3;
  *glArray = new GLfloat* [6];

  for (unsigned i = 0; i<6; ++i){
    (*glArray)[i] = new GLfloat[glArraySize];
  }

  if (glColorArray != NULL)
    *glColorArray = new GLfloat[glArraySize * 4 *4];

  // epsilon to be substracted from cube size so that neighboring planes don't overlap
  // seems to introduce strange artifacts nevertheless...
  double eps = 1e-5;

  // generate the cubes, 6 quads each
  // min and max-values are computed on-the-fly
  unsigned int i = 0;
  unsigned int colorIdx = 0;
  double x,y,z;

  for (std::list<octomap::OcTreeVolume>::const_iterator it=voxels.begin(); it != voxels.end(); it++) {

    double half_cube_size = GLfloat(it->second /2.0 -eps);

    x = it->first.x();
    y = it->first.y();
    z = it->first.z();

    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parrallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

   (*glArray)[0][i]   = x + half_cube_size;
   (*glArray)[0][i+1] = y + half_cube_size;
   (*glArray)[0][i+2] = z - half_cube_size;

   (*glArray)[1][i]   = x + half_cube_size;
   (*glArray)[1][i+1] = y - half_cube_size;
   (*glArray)[1][i+2] = z - half_cube_size;

   (*glArray)[2][i]   = x + half_cube_size;
   (*glArray)[2][i+1] = y + half_cube_size;
   (*glArray)[2][i+2] = z - half_cube_size;

   (*glArray)[3][i]   = x - half_cube_size;
   (*glArray)[3][i+1] = y + half_cube_size;
   (*glArray)[3][i+2] = z - half_cube_size;

   (*glArray)[4][i]   = x + half_cube_size;
   (*glArray)[4][i+1] = y + half_cube_size;
   (*glArray)[4][i+2] = z - half_cube_size;

   (*glArray)[5][i]   = x + half_cube_size;
   (*glArray)[5][i+1] = y + half_cube_size;
   (*glArray)[5][i+2] = z + half_cube_size;
    i+= 3;

   (*glArray)[0][i]   = x - half_cube_size;
   (*glArray)[0][i+1] = y + half_cube_size;
   (*glArray)[0][i+2] = z - half_cube_size;

   (*glArray)[1][i]   = x - half_cube_size;
   (*glArray)[1][i+1] = y - half_cube_size;
   (*glArray)[1][i+2] = z - half_cube_size;

   (*glArray)[2][i]   = x + half_cube_size;
   (*glArray)[2][i+1] = y + half_cube_size;
   (*glArray)[2][i+2] = z + half_cube_size;

   (*glArray)[3][i]   = x - half_cube_size;
   (*glArray)[3][i+1] = y + half_cube_size;
   (*glArray)[3][i+2] = z + half_cube_size;

   (*glArray)[4][i]   = x + half_cube_size;
   (*glArray)[4][i+1] = y - half_cube_size;
   (*glArray)[4][i+2] = z - half_cube_size;

   (*glArray)[5][i]   = x + half_cube_size;
   (*glArray)[5][i+1] = y - half_cube_size;
   (*glArray)[5][i+2] = z + half_cube_size;
    i+= 3;

   (*glArray)[0][i]   = x - half_cube_size;
   (*glArray)[0][i+1] = y + half_cube_size;
   (*glArray)[0][i+2] = z + half_cube_size;

   (*glArray)[1][i]   = x - half_cube_size;
   (*glArray)[1][i+1] = y - half_cube_size;
   (*glArray)[1][i+2] = z + half_cube_size;

   (*glArray)[2][i]   = x + half_cube_size;
   (*glArray)[2][i+1] = y - half_cube_size;
   (*glArray)[2][i+2] = z + half_cube_size;

   (*glArray)[3][i]   = x - half_cube_size;
   (*glArray)[3][i+1] = y - half_cube_size;
   (*glArray)[3][i+2] = z + half_cube_size;

   (*glArray)[4][i]   = x - half_cube_size;
   (*glArray)[4][i+1] = y - half_cube_size;
   (*glArray)[4][i+2] = z - half_cube_size;

   (*glArray)[5][i]   = x - half_cube_size;
   (*glArray)[5][i+1] = y - half_cube_size;
   (*glArray)[5][i+2] = z + half_cube_size;
    i+= 3;

   (*glArray)[0][i]   = x + half_cube_size;
   (*glArray)[0][i+1] = y + half_cube_size;
   (*glArray)[0][i+2] = z + half_cube_size;

   (*glArray)[1][i]   = x + half_cube_size;
   (*glArray)[1][i+1] = y - half_cube_size;
   (*glArray)[1][i+2] = z + half_cube_size;

   (*glArray)[2][i]   = x + half_cube_size;
   (*glArray)[2][i+1] = y - half_cube_size;
   (*glArray)[2][i+2] = z - half_cube_size;

   (*glArray)[3][i]   = x - half_cube_size;
   (*glArray)[3][i+1] = y - half_cube_size;
   (*glArray)[3][i+2] = z - half_cube_size;

   (*glArray)[4][i]   = x - half_cube_size;
   (*glArray)[4][i+1] = y + half_cube_size;
   (*glArray)[4][i+2] = z - half_cube_size;

   (*glArray)[5][i]   = x - half_cube_size;
   (*glArray)[5][i+1] = y + half_cube_size;
   (*glArray)[5][i+2] = z + half_cube_size;
    i += 3;

      if (glColorArray != NULL) {
        // color for 4 vertices (same height)
        for (int k = 0; k < 4; ++k) {
          SceneObject::heightMapColor(z, *glColorArray + colorIdx);
          // set Alpha value:
          (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
          colorIdx += 4;
        }
      }

    }
}

void OcTreeDrawer::clearCubes(GLfloat*** glArray, unsigned int& glArraySize, GLfloat** glColorArray){
  if (glArraySize != 0) {
    for (unsigned i = 0; i < 6; ++i) {
      delete[] (*glArray)[i];
    }
    delete[] *glArray;
    *glArray = NULL;
    glArraySize = 0;
  }

  if (glColorArray != NULL && *glColorArray != NULL) {
    delete[] *glColorArray;
    *glColorArray = NULL;
  }
}


void OcTreeDrawer::initOctreeGridVis() {

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

void OcTreeDrawer::clearOcTreeStructure(){
  if (octree_grid_vertex_size != 0) {
    delete[] octree_grid_vertex_array;
    octree_grid_vertex_size = 0;
  }

  m_octree_grid_vis_initialized = false;
}

void OcTreeDrawer::clear() {
  //clearOcTree();
  clearCubes(&m_occupiedArray, m_occupiedSize, &m_occupiedColorArray);
  clearCubes(&m_occupiedThresArray, m_occupiedThresSize, &m_occupiedThresColorArray);
  clearCubes(&m_freeArray, m_freeSize);
  clearCubes(&m_freeThresArray, m_freeThresSize);


  clearOcTreeStructure();
}

void OcTreeDrawer::draw() const {
  glEnableClientState(GL_VERTEX_ARRAY);

  if (m_drawOccupied)
    drawOccupiedVoxels();
  if (m_drawFree)
    drawFreeVoxels();
  if (m_drawOcTreeGrid)
    drawOctreeGrid();

  glDisableClientState(GL_VERTEX_ARRAY);
}

void OcTreeDrawer::drawOccupiedVoxels() const {


  // colors for printout mode:
  if (m_printoutMode) {
    if (!m_drawFree) { // gray on white background
      glColor3f(0.5f, 0.5f, 0.5f);
    }
    else {
      glColor3f(0.1f, 0.1f, 0.1f);
    }
  }

  // draw binary occupied cells
  if (m_occupiedThresSize != 0) {
    if (!m_printoutMode) glColor4f(0.0, 0.0, 1.0, m_alphaOccupied);
    drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
  }

  // draw delta occupied cells
  if (m_occupiedSize != 0) {
    if (!m_printoutMode) glColor4f(0.2, 0.7, 1.0, m_alphaOccupied);
    drawCubes(m_occupiedArray, m_occupiedSize, m_occupiedColorArray);
  }

}


void OcTreeDrawer::drawFreeVoxels() const {

  if (m_printoutMode) {
    if (!m_drawOccupied) { // gray on white background
      glColor3f(0.5f, 0.5f, 0.5f);
    }
    else {
      glColor3f(0.9f, 0.9f, 0.9f);
    }
  }

  // draw binary freespace cells
  if (m_freeThresSize != 0) {
    if (!m_printoutMode) glColor4f(0.0, 1.0, 0., 0.3);
    drawCubes(m_freeThresArray, m_freeThresSize);
  }


  // draw delta freespace cells
  if (m_freeSize != 0) {
    if (!m_printoutMode) glColor4f(0.5, 1.0, 0.1, 0.3);
    drawCubes(m_freeArray, m_freeSize);
  }
}



void OcTreeDrawer::drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize,
           GLfloat* cubeColorArray) const {
  if (cubeArraySize == 0 || cubeArray == NULL){
    std::cerr << "Warning: GLfloat array to draw cubes appears to be empty, nothing drawn.\n";
    return;
  }

  // save current color
  GLfloat* curcol = new GLfloat[4];
  glGetFloatv(GL_CURRENT_COLOR, curcol);

  // enable color pointer when heightColorMode is enabled:

  if (m_heightColorMode && cubeColorArray != NULL){
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(4, GL_FLOAT, 0, cubeColorArray);
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

void OcTreeDrawer::drawOctreeGrid() const {
  if (!m_octree_grid_vis_initialized) return;

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

void OcTreeDrawer::enableOcTree(bool enabled) {
  m_drawOcTreeGrid = enabled;
  if(m_drawOcTreeGrid && !m_octree_grid_vis_initialized) {
    initOctreeGridVis();
  }
}

}
