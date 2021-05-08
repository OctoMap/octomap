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

#include <octovis/OcTreeDrawer.h>
#include <qglviewer.h>

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
    m_displayAxes = false;
    m_update = true;
    m_alternativeDrawing = false;

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
    static int gl_list_index = -1;
    if(m_alternativeDrawing && gl_list_index < 0){
      gl_list_index = glGenLists(1);
      m_update = true;
    }
    if(!m_alternativeDrawing && gl_list_index != -1){//Free video card memory
      //std::cerr << "Freeing VRAM\n";
      glDeleteLists(gl_list_index,1);
      gl_list_index = -1;
    }
    if(m_update || !m_alternativeDrawing)
    {
      if(m_alternativeDrawing) {
        std::cout << "Preparing batch rendering, please wait ...\n";
        glNewList(gl_list_index, GL_COMPILE_AND_EXECUTE);
      }

      // push current status
      glPushMatrix();
      // octomap::pose6d relative_transform = origin * initial_origin.inv();

      octomap::pose6d relative_transform = origin;// * initial_origin;

      // apply relative transform
      const octomath::Quaternion& q = relative_transform.rot();
      glTranslatef(relative_transform.x(), relative_transform.y(), relative_transform.z());

      // convert quaternion to angle/axis notation
      float scale = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
      if (scale) {
        float axis_x = q.x() / scale;
        float axis_y = q.y() / scale;
        float axis_z = q.z() / scale;
        float angle = acos(q.u()) * 2.0f * OTD_RAD2DEG;  //  opengl expects DEG
        glRotatef(angle, axis_x, axis_y, axis_z);
      }

      glEnableClientState(GL_VERTEX_ARRAY);

        if (m_drawSelection) // Drawing voxels in descending alpha-channel magnitude avoids (most) artifacts
          drawSelection();
        if (m_drawOccupied)
          drawOccupiedVoxels();
        if (m_drawFree)
          drawFreeVoxels();
        if (m_drawOcTreeGrid)
          drawOctreeGrid();

        if (m_displayAxes) {
          drawAxes();
        }

      glDisableClientState(GL_VERTEX_ARRAY);

      // reset previous status
      glPopMatrix();
      if(m_alternativeDrawing) {
        glEndList();
        std::cout << "Finished preparation of batch rendering.\n";
      }
      m_update = false;
    }
    else
    {
      glCallList(gl_list_index);
    }

  }

  void OcTreeDrawer::setAlphaOccupied(double alpha){
    m_update = true;
    m_alphaOccupied = alpha;
  }


  void OcTreeDrawer::setOcTree(const AbstractOcTree& in_tree, const pose6d& in_origin, int map_id_) {

    const OcTree& octree = (const OcTree&) in_tree;
    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), in_origin.rot());

    // origin is in global coords
    this->origin = in_origin;

    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool showAll = (octree.size() < 5 * 1e6);
    bool uses_origin = ( (origin.rot().x() != 0.) && (origin.rot().y() != 0.)
        && (origin.rot().z() != 0.) && (origin.rot().u() != 1.) );

    // walk the tree one to find the number of nodes in each category
    // (this is used to set up the OpenGL arrays)
    // TODO: this step may be left out, if we maintained the GLArrays in std::vectors instead...
    unsigned int cnt_occupied(0), cnt_occupied_thres(0), cnt_free(0), cnt_free_thres(0);
    for(OcTree::tree_iterator it = octree.begin_tree(this->m_max_tree_depth),
            end=octree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) {
        if (octree.isNodeOccupied(*it)){ // occupied voxels
          if (octree.isNodeAtThreshold(*it)) ++cnt_occupied_thres;
          else                               ++cnt_occupied;
        }
        else if (showAll) { // freespace voxels
          if (octree.isNodeAtThreshold(*it)) ++cnt_free_thres;
          else                               ++cnt_free;
        }
      }
    }
    // setup GL arrays for cube quads and cube colors
    initGLArrays(cnt_occupied      , m_occupiedSize     , &m_occupiedArray     , &m_occupiedColorArray);
    initGLArrays(cnt_occupied_thres, m_occupiedThresSize, &m_occupiedThresArray, &m_occupiedThresColorArray);
    initGLArrays(cnt_free          , m_freeSize         , &m_freeArray, NULL);
    initGLArrays(cnt_free_thres    , m_freeThresSize    , &m_freeThresArray, NULL);

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);

    // set min/max Z for color height map
    m_zMin = minZ;
    m_zMax = maxZ;

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin, cube_template);

    unsigned int idx_occupied(0), idx_occupied_thres(0), idx_free(0), idx_free_thres(0);
    unsigned int color_idx_occupied(0), color_idx_occupied_thres(0);

    m_grid_voxels.clear();
    OcTreeVolume voxel; // current voxel, possibly transformed
    for(OcTree::tree_iterator it = octree.begin_tree(this->m_max_tree_depth),
            end=octree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        if (uses_origin)
          voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
        else
          voxel = OcTreeVolume(it.getCoordinate(), it.getSize());

        if (octree.isNodeOccupied(*it)){ // occupied voxels
          if (octree.isNodeAtThreshold(*it)) {
            idx_occupied_thres = generateCube(voxel, cube_template, idx_occupied_thres, &m_occupiedThresArray);
            color_idx_occupied_thres = setCubeColorHeightmap(voxel, color_idx_occupied_thres, &m_occupiedThresColorArray);
          }
          else {
            idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
            color_idx_occupied = setCubeColorHeightmap(voxel, color_idx_occupied, &m_occupiedColorArray);
          }
        }
        else if (showAll) { // freespace voxels
          if (octree.isNodeAtThreshold(*it)) {
            idx_free_thres = generateCube(voxel, cube_template, idx_free_thres, &m_freeThresArray);
          }
          else {
            idx_free = generateCube(voxel, cube_template, idx_free, &m_freeArray);
          }
        }
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

    m_octree_grid_vis_initialized = false;

    if(m_drawOcTreeGrid)
      initOctreeGridVis();
  }

  void OcTreeDrawer::setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedVoxels){
    m_update = true;

    // init selectedVoxels GLarray
    initGLArrays(selectedVoxels.size(), m_selectionSize, &m_selectionArray, NULL);

    generateCubes(selectedVoxels, &m_selectionArray, m_selectionSize, this->origin);
  }

  void OcTreeDrawer::clearOcTreeSelection(){
    m_update = true;
    clearCubes(&m_selectionArray, m_selectionSize);
  }

  void OcTreeDrawer::initGLArrays(const unsigned int& num_cubes,
                                  unsigned int& glArraySize,
                                  GLfloat*** glArray, GLfloat** glColorArray) {

    clearCubes(glArray, glArraySize, glColorArray);

    // store size of GL arrays for drawing
    glArraySize = num_cubes * 4 * 3;

    // allocate cube arrays, 6 quads per cube
    *glArray = new GLfloat* [6];
    for (unsigned i = 0; i<6; ++i){
      (*glArray)[i] = new GLfloat[glArraySize];
    }
    // setup quad color array, if given
    if (glColorArray != NULL)
      *glColorArray = new GLfloat[glArraySize * 4 *4];
  }

  void OcTreeDrawer::initCubeTemplate(const octomath::Pose6D& origin,
                                      std::vector<octomath::Vector3>& cube_template) {
    cube_template.clear();
    cube_template.reserve(24);

    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));

    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
  }

  unsigned int OcTreeDrawer::generateCube(const octomap::OcTreeVolume& v,
                                          const std::vector<octomath::Vector3>& cube_template,
                                          const unsigned int& current_array_idx,
                                          GLfloat*** glArray) {

    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;

    octomath::Vector3 p;

    double half_cube_size = GLfloat(v.second /2.0 -eps);
    unsigned int i = current_array_idx;
    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    p = v.first + cube_template[0] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[1] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[2] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[3] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[4] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[5] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[6] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[7] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[8] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[9] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[10] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[11] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[12] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[13] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[14] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[15] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[16] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[17] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[18] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[19] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[20] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[21] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[22] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[23] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i += 3;  //-------------------

    return i; // updated array idx
  }


  unsigned int OcTreeDrawer::setCubeColorHeightmap(const octomap::OcTreeVolume& v,
                                          const unsigned int& current_array_idx,
                                          GLfloat** glColorArray) {

    if (glColorArray == NULL) return current_array_idx;

    unsigned int colorIdx = current_array_idx;
    // color for all 4 vertices (same height)
    for (int k = 0; k < 4; ++k) {
      if (m_colorMode == CM_GRAY_HEIGHT)
        SceneObject::heightMapGray(v.first.z(), *glColorArray + colorIdx);  // sets r,g,b
      else
        SceneObject::heightMapColor(v.first.z(), *glColorArray + colorIdx);   // sets r,g,b
      // set Alpha value:
      (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
      colorIdx += 4;
    }
    return colorIdx;
  }

  unsigned int OcTreeDrawer::setCubeColorRGBA(const unsigned char& r,
                                              const unsigned char& g,
                                              const unsigned char& b,
                                              const unsigned char& a,
                                              const unsigned int& current_array_idx,
                                              GLfloat** glColorArray) {

    if (glColorArray == NULL) return current_array_idx;
    unsigned int colorIdx = current_array_idx;
    // set color for next 4 vertices (=one quad)
    for (int k = 0; k < 4; ++k) {
      (*glColorArray)[colorIdx    ] = (double) r/255.;
      (*glColorArray)[colorIdx + 1] = (double) g/255.;
      (*glColorArray)[colorIdx + 2] = (double) b/255.;
      (*glColorArray)[colorIdx + 3] = (double) a/255.;
      colorIdx += 4;
    }
    return colorIdx;
  }


  void OcTreeDrawer::clearCubes(GLfloat*** glArray,
                                unsigned int& glArraySize,
                                GLfloat** glColorArray) {
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


  // still used for "selection" nodes
  void OcTreeDrawer::generateCubes(const std::list<octomap::OcTreeVolume>& voxels,
                                   GLfloat*** glArray, unsigned int& glArraySize,
                                   octomath::Pose6D& origin,
                                   GLfloat** glColorArray) {
    unsigned int i = 0;
    unsigned int colorIdx = 0;

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin, cube_template);

    for (std::list<octomap::OcTreeVolume>::const_iterator it=voxels.begin();
         it != voxels.end(); it++) {
      i = generateCube(*it, cube_template, i, glArray);
    }

    if (glColorArray != NULL) {
      for (std::list<octomap::OcTreeVolume>::const_iterator it=voxels.begin();
           it != voxels.end(); it++) {
        colorIdx = setCubeColorHeightmap(*it, colorIdx, glColorArray);
      }
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
        if (m_colorMode != CM_PRINTOUT) glColor4f(0.0f, 0.0f, 1.0f, m_alphaOccupied);
        drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
      }

      // draw delta occupied cells
      if (m_occupiedSize != 0) {
        if (m_colorMode != CM_PRINTOUT) glColor4f(0.2f, 0.7f, 1.0f, m_alphaOccupied);
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
      if (m_colorMode != CM_PRINTOUT) glColor4f(0.0f, 1.0f, 0.0f, 0.3f);
      drawCubes(m_freeThresArray, m_freeThresSize);
    }

    // draw delta freespace cells
    if (m_freeSize != 0) {
      if (m_colorMode != CM_PRINTOUT) glColor4f(0.5f, 1.0f, 0.1f, 0.3f);
      drawCubes(m_freeArray, m_freeSize);
    }
  }

  void OcTreeDrawer::drawSelection() const {
    if (m_selectionSize != 0) {
      glColor4f(1.0, 0.0, 0.0, 1.0);
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

    if ((m_colorMode == CM_COLOR_HEIGHT || m_colorMode == CM_GRAY_HEIGHT)
        && (cubeColorArray != NULL)){
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
    if (octree_grid_vertex_size == 0)   return;

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
    m_update = true;
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

    float length = 0.15f;

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

} // namespace
