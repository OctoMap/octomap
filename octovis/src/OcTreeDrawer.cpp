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

#include <octovis/OcTreeDrawer.h>

#define OTD_RAD2DEG 57.2957795

namespace octomap {

  OcTreeDrawer::OcTreeDrawer() : SceneObject(),
                                 m_occupiedThresSize(0), m_freeThresSize(0),
                                 m_occupiedSize(0), m_freeSize(0), m_selectionSize(0),
                                 octree_grid_vertex_size(0), m_alphaOccupied(0.8), map_id(0)
  {
    m_octree_grid_vis_initialized = false;
    m_drawOccupied = true;
    m_drawOcTreeGrid = false;
    m_drawFree = false;
    m_drawSelection = true;
    m_display_axes = false;

    m_occupiedArray = NULL;
    m_freeArray = NULL;
    m_occupiedThresArray = NULL;
    m_freeThresArray = NULL;
    m_occupiedColorArray = NULL;
    m_occupiedThresColorArray = NULL;
    m_selectionArray = NULL;

    // origin and movement
    initial_origin = octomap::pose6d(0,0,0,0,0,0);
    origin = initial_origin;
  }

  OcTreeDrawer::~OcTreeDrawer() {
    clear();
  }

  void OcTreeDrawer::draw() const {

    // push current status
    glPushMatrix();
    octomap::pose6d relative_transform = origin * initial_origin.inv();

    // apply relative transform
    const octomath::Quaternion& q = relative_transform.rot();
    glTranslatef(relative_transform.x(), relative_transform.y(), relative_transform.z());
    
    // convert quaternion to angle/axis notation
    float scale = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    float axis_x = q.x() / scale;
    float axis_y = q.y() / scale;
    float axis_z = q.z() / scale;
    float angle = acos(q.u()) * 2.0f * OTD_RAD2DEG;  //  opengl expects DEG

    glRotatef(angle, axis_x, axis_y, axis_z);

    glEnableClientState(GL_VERTEX_ARRAY);

    if (m_drawOccupied)
      drawOccupiedVoxels();
    if (m_drawFree)
      drawFreeVoxels();
    if (m_drawOcTreeGrid)
      drawOctreeGrid();
    if (m_drawSelection)
      drawSelection();

    if (m_display_axes) {
      drawAxes();
    }

    glDisableClientState(GL_VERTEX_ARRAY);

    // reset previous status
    glPopMatrix();

  }

  void OcTreeDrawer::setAlphaOccupied(double alpha){
    m_alphaOccupied = alpha;
  }

  void OcTreeDrawer::setOcTree(const octomap::OcTree &octree, octomap::pose6d origin_, int map_id_) {

    this->map_id = map_id_;

    // origin is in global coords
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin_.rot()); // save origin used during cube generation
    this->origin = origin_;

    std::list<octomap::OcTreeVolume> occupiedVoxels;
    std::list<octomap::OcTreeVolume> occupiedThresVoxels;
    std::list<octomap::OcTreeVolume> freeVoxels;
    std::list<octomap::OcTreeVolume> freeThresVoxels;
    m_grid_voxels.clear();

    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool showAll = (octree.size() < 5 * 1e6);
    bool uses_origin = ( (origin_.rot().x() != 0.) && (origin_.rot().y() != 0.)
        && (origin_.rot().z() != 0.) && (origin_.rot().u() != 1.) );
    
    // TODO: still using the lists, these should be gone as well
    OcTreeVolume voxel;
    for(OcTree::tree_iterator it = octree.begin_tree(this->m_max_tree_depth),
            end=octree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        if (uses_origin) 
          voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
        else 
          voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
        
        if (octree.isNodeOccupied(*it)){ // occupied voxels
          if (octree.isNodeAtThreshold(*it)) occupiedThresVoxels.push_back(voxel);
          else                               occupiedVoxels.push_back(voxel);          
        }
        else if (showAll) { // freespace voxels
          if (octree.isNodeAtThreshold(*it)) freeThresVoxels.push_back(voxel);
          else                               freeVoxels.push_back(voxel);
        }

        // grid structure voxel
        if (showAll) m_grid_voxels.push_back(voxel);        
      }
      
      else { // inner node voxels (for grid structure only)
        if (showAll) {
          if (uses_origin) 
            voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
          else 
            voxel = OcTreeVolume(it.getCoordinate(), it.getSize());          
          m_grid_voxels.push_back(voxel);        
        }
      }      
    } // end for all voxels

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);

    // set min/max Z for height map
    m_zMin = minZ;
    m_zMax = maxZ;

    m_octree_grid_vis_initialized = false;
    if(m_drawOcTreeGrid)
      initOctreeGridVis();

    // generate openGL representation of octree
    // TODO: get rid of lists, directly use values from the iterators to build the GL arrays
    generateCubes(occupiedThresVoxels, &m_occupiedThresArray, m_occupiedThresSize, origin, &m_occupiedThresColorArray);
    generateCubes(freeThresVoxels, &m_freeThresArray, m_freeThresSize, origin);

    generateCubes(occupiedVoxels, &m_occupiedArray, m_occupiedSize, origin, &m_occupiedColorArray);
    generateCubes(freeVoxels, &m_freeArray, m_freeSize, origin);
  }

  void OcTreeDrawer::setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedVoxels){
    generateCubes(selectedVoxels, &m_selectionArray, m_selectionSize);

  }

  void OcTreeDrawer::clearOcTreeSelection(){
    clearCubes(&m_selectionArray, m_selectionSize);
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

    unsigned int i = 0;
    unsigned int colorIdx = 0;
    double x,y,z;

    // generate the cubes, 6 quads each

    for (std::list<octomap::OcTreeVolume>::const_iterator it=voxels.begin(); it != voxels.end(); it++) {

      double half_cube_size = GLfloat(it->second /2.0 -eps);

      x = it->first.x();
      y = it->first.y();
      z = it->first.z();

      // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
      // Arrays are filled in parallel (increasing i for all at once)
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
          if (m_colorMode == CM_GRAY_HEIGHT)
            SceneObject::heightMapGray(z, *glColorArray + colorIdx);
          else
            SceneObject::heightMapColor(z, *glColorArray + colorIdx);
          // set Alpha value:
          (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
          colorIdx += 4;
        }
      }

    }
  }


  void OcTreeDrawer::generateCubes(const std::list<octomap::OcTreeVolume>& voxels,
                                   GLfloat*** glArray, unsigned int& glArraySize,
                                   octomath::Pose6D& origin,
                                   GLfloat** glColorArray) {
        

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

    unsigned int i = 0;
    unsigned int colorIdx = 0;

    // list all cube centers
    octomath::Quaternion rot (origin.rot());
    std::vector<octomath::Vector3> cube_vectors;
    cube_vectors.push_back(octomath::Vector3( 1, 1,-1));
    cube_vectors.push_back(octomath::Vector3( 1,-1,-1));
    cube_vectors.push_back(octomath::Vector3( 1, 1,-1));
    cube_vectors.push_back(octomath::Vector3(-1, 1,-1));
    cube_vectors.push_back(octomath::Vector3( 1, 1,-1));
    cube_vectors.push_back(octomath::Vector3( 1, 1, 1));

    cube_vectors.push_back(octomath::Vector3(-1, 1,-1));
    cube_vectors.push_back(octomath::Vector3(-1,-1,-1));
    cube_vectors.push_back(octomath::Vector3( 1, 1, 1));
    cube_vectors.push_back(octomath::Vector3(-1, 1, 1));
    cube_vectors.push_back(octomath::Vector3( 1,-1,-1));
    cube_vectors.push_back(octomath::Vector3( 1,-1, 1));

    cube_vectors.push_back(octomath::Vector3(-1, 1, 1));
    cube_vectors.push_back(octomath::Vector3(-1,-1, 1));
    cube_vectors.push_back(octomath::Vector3( 1,-1, 1));
    cube_vectors.push_back(octomath::Vector3(-1,-1, 1));
    cube_vectors.push_back(octomath::Vector3(-1,-1,-1));
    cube_vectors.push_back(octomath::Vector3(-1,-1, 1));

    cube_vectors.push_back(octomath::Vector3( 1, 1, 1));
    cube_vectors.push_back(octomath::Vector3( 1,-1, 1));
    cube_vectors.push_back(octomath::Vector3( 1,-1,-1));
    cube_vectors.push_back(octomath::Vector3(-1,-1,-1));
    cube_vectors.push_back(octomath::Vector3(-1, 1,-1));
    cube_vectors.push_back(octomath::Vector3(-1, 1, 1));

    // rotate them according to object orientation
    for (std::vector<octomath::Vector3>::iterator it = cube_vectors.begin(); it != cube_vectors.end(); ++it) {
      *it = rot.rotate(*it);
    }


    octomath::Vector3 p;
    for (std::list<octomap::OcTreeVolume>::const_iterator it=voxels.begin(); it != voxels.end(); it++) {

      double half_cube_size = GLfloat(it->second /2.0 -eps);

      // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
      // Arrays are filled in parallel (increasing i for all at once)
      // One color array for all surfaces is filled when requested

      p = it->first + cube_vectors[0] * half_cube_size;
      (*glArray)[0][i]   = p.x();
      (*glArray)[0][i+1] = p.y();
      (*glArray)[0][i+2] = p.z();

      p = it->first + cube_vectors[1] * half_cube_size;
      (*glArray)[1][i]   = p.x();
      (*glArray)[1][i+1] = p.y();
      (*glArray)[1][i+2] = p.z();

      p = it->first + cube_vectors[2] * half_cube_size;
      (*glArray)[2][i]   = p.x();
      (*glArray)[2][i+1] = p.y();
      (*glArray)[2][i+2] = p.z();

      p = it->first + cube_vectors[3] * half_cube_size;
      (*glArray)[3][i]   = p.x(); 
      (*glArray)[3][i+1] = p.y(); 
      (*glArray)[3][i+2] = p.z(); 

      p = it->first + cube_vectors[4] * half_cube_size;
      (*glArray)[4][i]   = p.x(); 
      (*glArray)[4][i+1] = p.y(); 
      (*glArray)[4][i+2] = p.z(); 

      p = it->first + cube_vectors[5] * half_cube_size;
      (*glArray)[5][i]   = p.x(); 
      (*glArray)[5][i+1] = p.y(); 
      (*glArray)[5][i+2] = p.z(); 
      i+= 3;  //-------------------

      p = it->first + cube_vectors[6] * half_cube_size;
      (*glArray)[0][i]   = p.x();
      (*glArray)[0][i+1] = p.y();
      (*glArray)[0][i+2] = p.z();

      p = it->first + cube_vectors[7] * half_cube_size;
      (*glArray)[1][i]   = p.x();
      (*glArray)[1][i+1] = p.y();
      (*glArray)[1][i+2] = p.z();

      p = it->first + cube_vectors[8] * half_cube_size;
      (*glArray)[2][i]   = p.x();
      (*glArray)[2][i+1] = p.y();
      (*glArray)[2][i+2] = p.z();

      p = it->first + cube_vectors[9] * half_cube_size;
      (*glArray)[3][i]   = p.x();
      (*glArray)[3][i+1] = p.y();
      (*glArray)[3][i+2] = p.z();

      p = it->first + cube_vectors[10] * half_cube_size;
      (*glArray)[4][i]   = p.x();
      (*glArray)[4][i+1] = p.y();
      (*glArray)[4][i+2] = p.z();

      p = it->first + cube_vectors[11] * half_cube_size;
      (*glArray)[5][i]   = p.x();
      (*glArray)[5][i+1] = p.y();
      (*glArray)[5][i+2] = p.z();
      i+= 3;  //-------------------

      p = it->first + cube_vectors[12] * half_cube_size;
      (*glArray)[0][i]   = p.x();
      (*glArray)[0][i+1] = p.y();
      (*glArray)[0][i+2] = p.z();

      p = it->first + cube_vectors[13] * half_cube_size;
      (*glArray)[1][i]   = p.x();
      (*glArray)[1][i+1] = p.y();
      (*glArray)[1][i+2] = p.z();

      p = it->first + cube_vectors[14] * half_cube_size;
      (*glArray)[2][i]   = p.x();
      (*glArray)[2][i+1] = p.y();
      (*glArray)[2][i+2] = p.z();

      p = it->first + cube_vectors[15] * half_cube_size;
      (*glArray)[3][i]   = p.x();
      (*glArray)[3][i+1] = p.y();
      (*glArray)[3][i+2] = p.z();

      p = it->first + cube_vectors[16] * half_cube_size;
      (*glArray)[4][i]   = p.x();
      (*glArray)[4][i+1] = p.y();
      (*glArray)[4][i+2] = p.z();

      p = it->first + cube_vectors[17] * half_cube_size;
      (*glArray)[5][i]   = p.x();
      (*glArray)[5][i+1] = p.y();
      (*glArray)[5][i+2] = p.z();
      i+= 3;  //-------------------

      p = it->first + cube_vectors[18] * half_cube_size;
      (*glArray)[0][i]   = p.x();
      (*glArray)[0][i+1] = p.y();
      (*glArray)[0][i+2] = p.z();

      p = it->first + cube_vectors[19] * half_cube_size;
      (*glArray)[1][i]   = p.x();
      (*glArray)[1][i+1] = p.y();
      (*glArray)[1][i+2] = p.z();

      p = it->first + cube_vectors[20] * half_cube_size;
      (*glArray)[2][i]   = p.x();
      (*glArray)[2][i+1] = p.y();
      (*glArray)[2][i+2] = p.z();

      p = it->first + cube_vectors[21] * half_cube_size;
      (*glArray)[3][i]   = p.x();
      (*glArray)[3][i+1] = p.y();
      (*glArray)[3][i+2] = p.z();

      p = it->first + cube_vectors[22] * half_cube_size;
      (*glArray)[4][i]   = p.x();
      (*glArray)[4][i+1] = p.y();
      (*glArray)[4][i+2] = p.z();

      p = it->first + cube_vectors[23] * half_cube_size;
      (*glArray)[5][i]   = p.x();
      (*glArray)[5][i+1] = p.y();
      (*glArray)[5][i+2] = p.z();
      i += 3;  //-------------------

      if (glColorArray != NULL) {
        // color for 4 vertices (same height)
        for (int k = 0; k < 4; ++k) {
          if (m_colorMode == CM_GRAY_HEIGHT)
            SceneObject::heightMapGray(it->first.z(), *glColorArray + colorIdx);
          else
            SceneObject::heightMapColor(it->first.z(), *glColorArray + colorIdx);
          // set Alpha value:
          (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
          colorIdx += 4;
        }
      }
    } // end for all voxel centers

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
    clearCubes(&m_selectionArray, m_selectionSize);


    clearOcTreeStructure();
  }


  void OcTreeDrawer::drawOccupiedVoxels() const {


    if (m_colorMode == CM_SEMANTIC) {
      // hardcoded mapping id -> colors
      if (this->map_id == 0) {  // background
        glColor3f(0.784f, 0.66f, 0); // gold
      }
      else if (this->map_id == 1) {  // table
        glColor3f(0.68f, 0., 0.62f); // purple
      }
      else { // object
        glColor3f(0., 0.784f, 0.725f); // cyan
      }
      drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
    }
    
    else {
      
      // colors for printout mode:
      if (m_colorMode == CM_PRINTOUT) {
        if (!m_drawFree) { // gray on white background
          glColor3f(0.6f, 0.6f, 0.6f);
        }
        else {
          glColor3f(0.1f, 0.1f, 0.1f);
        }  
      }
      
      // draw binary occupied cells
      if (m_occupiedThresSize != 0) {
        if (m_colorMode != CM_PRINTOUT) glColor4f(0.0, 0.0, 1.0, m_alphaOccupied);
        drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
      }

      // draw delta occupied cells
      if (m_occupiedSize != 0) {
        if (m_colorMode != CM_PRINTOUT) glColor4f(0.2, 0.7, 1.0, m_alphaOccupied);
        drawCubes(m_occupiedArray, m_occupiedSize, m_occupiedColorArray);
      }
    }

  }


  void OcTreeDrawer::drawFreeVoxels() const {

    if (m_colorMode == CM_PRINTOUT) {
      if (!m_drawOccupied) { // gray on white background
        glColor3f(0.5f, 0.5f, 0.5f);
      }
      else {
        glColor3f(0.9f, 0.9f, 0.9f);
      }
    }

    // draw binary freespace cells
    if (m_freeThresSize != 0) {
      if (m_colorMode != CM_PRINTOUT) glColor4f(0.0, 1.0, 0., 0.3);
      drawCubes(m_freeThresArray, m_freeThresSize);
    }


    // draw delta freespace cells
    if (m_freeSize != 0) {
      if (m_colorMode != CM_PRINTOUT) glColor4f(0.5, 1.0, 0.1, 0.3);
      drawCubes(m_freeArray, m_freeSize);
    }
  }

  void OcTreeDrawer::drawSelection() const {
    if (m_selectionSize != 0) {
      glColor4f(1.0, 0.0, 0.0, 0.5);
      drawCubes(m_selectionArray, m_selectionSize);
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

    if ((m_colorMode == CM_COLOR_HEIGHT || m_colorMode == CM_GRAY_HEIGHT) && (cubeColorArray != NULL)){
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

    if ((m_colorMode == CM_COLOR_HEIGHT || m_colorMode == CM_GRAY_HEIGHT) && (cubeColorArray != NULL)){
      glDisableClientState(GL_COLOR_ARRAY);
    }

    // draw bounding linies of cubes in printout:
    if (m_colorMode == CM_PRINTOUT){
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


  void OcTreeDrawer::setOrigin(octomap::pose6d t){
    origin = t;  
    std::cout << "OcTreeDrawer: setting new global origin: " << t << std::endl;


    octomap::pose6d relative_transform = origin * initial_origin.inv();

    std::cout << "origin        : " << origin << std::endl;
    std::cout << "inv init orig : " << initial_origin.inv() << std::endl;
    std::cout << "relative trans: " << relative_transform << std::endl;
  }

  void OcTreeDrawer::drawAxes() const {

    octomap::pose6d relative_transform = origin * initial_origin.inv();

    glPushMatrix();

    float length = .15; 

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_COLOR_MATERIAL);

    double angle= 2 * acos(initial_origin.rot().u());
    double scale = sqrt (initial_origin.rot().x()*initial_origin.rot().x() 
                         + initial_origin.rot().y()*initial_origin.rot().y() 
                         + initial_origin.rot().z()*initial_origin.rot().z());
    double ax= initial_origin.rot().x() / scale;
    double ay= initial_origin.rot().y() / scale;
    double az= initial_origin.rot().z() / scale;

    if (angle > 0) glRotatef(OTD_RAD2DEG*angle, ax, ay, az);

    float color[4];
    color[0] = 0.7f;  color[1] = 0.7f;  color[2] = 1.0f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    QGLViewer::drawArrow(length, 0.01*length);

    color[0] = 1.0f;  color[1] = 0.7f;  color[2] = 0.7f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0, 0.0, 1.0, 0.0);
    QGLViewer::drawArrow(length, 0.01*length);
    glPopMatrix();

    color[0] = 0.7f;  color[1] = 1.0f;  color[2] = 0.7f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    QGLViewer::drawArrow(length, 0.01*length);
    glPopMatrix();

    glTranslatef(relative_transform.trans().x(), relative_transform.trans().y(), relative_transform.trans().z());

    if (colorMaterial)
      glEnable(GL_COLOR_MATERIAL);
    if (!lighting)
      glDisable(GL_LIGHTING);

    glPopMatrix();

  }

}
