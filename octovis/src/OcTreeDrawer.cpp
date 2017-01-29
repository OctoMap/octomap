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
#include <QElapsedTimer>

#define OTD_RAD2DEG 57.2957795

namespace octomap {
  OcTreeDrawer::OcTreeIterator::OcTreeIterator(const OcTree* tree) : m_tree(tree)
  {
  }

  void OcTreeDrawer::OcTreeIterator::setOccupancyThres(double threshold){
    // TODO: need a better way than using a const cast for this.
    const_cast<OcTree*>(m_tree)->setOccupancyThres(threshold);
  }

  double OcTreeDrawer::OcTreeIterator::getOccupancyThres() const {
    return m_tree->getOccupancyThres();
  }

  unsigned int OcTreeDrawer::OcTreeIterator::getTreeSize() const {
    return m_tree->size();
  }

  void OcTreeDrawer::OcTreeIterator::getMetricMin(double& x, double& y, double& z) const {
    return m_tree->getMetricMin(x, y, z);
  }

  void OcTreeDrawer::OcTreeIterator::getMetricMax(double& x, double& y, double& z) const {
    return m_tree->getMetricMax(x, y, z);
  }

  bool OcTreeDrawer::OcTreeIterator::isValid() const {
    return m_it != m_tree->end_tree();
  }

  bool OcTreeDrawer::OcTreeIterator::begin(unsigned int max_tree_depth){
    m_it = m_tree->begin_tree(max_tree_depth);
    return m_it != m_tree->end_tree();
  }

  bool OcTreeDrawer::OcTreeIterator::moveNext(){
    ++m_it;
    return m_it != m_tree->end_tree();
  }

  float OcTreeDrawer::OcTreeIterator::getOccupancy() const{
    return m_it->getOccupancy();
  }

  point3d OcTreeDrawer::OcTreeIterator::getCoordinate() const{
    return m_it.getCoordinate();
  }

  double OcTreeDrawer::OcTreeIterator::getSize() const{
    return m_it.getSize();
  }

  bool OcTreeDrawer::OcTreeIterator::isLeaf() const{
    return m_it.isLeaf();
  }

  bool OcTreeDrawer::OcTreeIterator::isOccupied() const{
    return m_tree->isNodeOccupied(*m_it);
  }

  bool OcTreeDrawer::OcTreeIterator::isAtThreshold() const{
    return m_tree->isNodeAtThreshold(*m_it);
  }


  OcTreeDrawer::Regeneration::Regeneration() : phase(OcTreeDrawer::NONE),
                                               showAll(false), uses_origin(false),
                                               progress(0), maxProgress(0),
                                               cnt_occupied(0), cnt_occupied_thres(0),
                                               cnt_free(0), cnt_free_thres(0),
                                               it(NULL)
  {
  }

  OcTreeDrawer::Regeneration::~Regeneration(){
    delete it;
  }

  OcTreeDrawer::OcTreeDrawer() : SceneObject(),
                                 m_occupiedThresSize(0), m_freeThresSize(0),
                                 m_occupiedSize(0), m_freeSize(0), m_selectionSize(0),
                                 octree_grid_vertex_size(0),
                                 m_max_tree_depth(0),
                                 m_alphaOccupied(0.8),
                                 m_occupancyThreshold(0.5),
                                 map_id(0)
  {
    m_occupiedCap = m_occupiedThresCap = m_freeCap = m_freeThresCap = m_occupiedColorSize = m_occupiedThresColorSize = 0;
    m_octree_grid_vis_initialized = false;
    m_gridVoxelsBuilt = false;
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
    octree_grid_vertex_array = NULL;
    octree_grid_vertex_size = 0;

    // origin and movement
    initial_origin = octomap::pose6d(0,0,0,0,0,0);
    origin = initial_origin;
  }

  OcTreeDrawer::~OcTreeDrawer() {
    clear();
  }

  OcTreeDrawer::BuildPhase OcTreeDrawer::update(unsigned int timeout, unsigned int* progress, unsigned int* maxProgress) {
    // Continue amortised construction.
    buildVoxels(timeout);
    if (progress) *progress = m_regeneration.progress;
    if (maxProgress) *maxProgress = m_regeneration.maxProgress;
    return m_regeneration.phase;
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

    if (m_drawOccupied)
      drawOccupiedVoxels();
    if (m_drawFree)
      drawFreeVoxels();
    if (m_drawOcTreeGrid)
      drawOctreeGrid();
    if (m_drawSelection)
      drawSelection();

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


  void OcTreeDrawer::setOcTree(const AbstractOcTree& tree, const pose6d& origin, int map_id_) {
      
    const OcTree& octree = (const OcTree&) tree;
    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin.rot());

    // origin is in global coords
    this->origin = origin;
    
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    m_regeneration.showAll = (octree.size() < 5 * 1e6);
    m_regeneration.uses_origin = ( (origin.rot().x() != 0.) && (origin.rot().y() != 0.)
        && (origin.rot().z() != 0.) && (origin.rot().u() != 1.) );

    // Start amortised construction of the voxels.
    delete m_regeneration.it;
    m_regeneration.it = new OcTreeIterator(&octree);
    m_regeneration.phase = OcTreeDrawer::INIT;
    buildVoxels();
  }

  void OcTreeDrawer::setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedVoxels){
    m_update = true;
    generateCubes(selectedVoxels, &m_selectionArray, m_selectionSize, this->origin);
  }

  void OcTreeDrawer::clearOcTreeSelection(){
    m_update = true;
    clearCubes(&m_selectionArray, m_selectionSize);
  }

  void OcTreeDrawer::setOccupancyThreshold(double threshold){
    if (threshold != m_occupancyThreshold){
      m_occupancyThreshold = threshold;
      clear();
      // Need to rebuild the voxels.
      m_regeneration.phase = OcTreeDrawer::INIT;
      buildVoxels();
    }
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

  void OcTreeDrawer::initOctreeGridVis(bool expand) {

    if (m_octree_grid_vis_initialized) return;

    unsigned int previousSize = octree_grid_vertex_size;
    GLfloat* previousArray = octree_grid_vertex_array;

    if (!expand){
      clearOcTreeStructure();
    }
    // allocate arrays for octree grid visualization
    octree_grid_vertex_size = m_grid_voxels.size() * 12 * 2 * 3;
    octree_grid_vertex_array = new GLfloat[octree_grid_vertex_size];

    unsigned voxelOffset = 0;
    if (expand && previousSize && previousSize <= octree_grid_vertex_size){
      memcpy(octree_grid_vertex_array, previousArray, sizeof(GLfloat) * previousSize);
      delete [] previousArray;
      // Determine which voxel we continue building at.
      voxelOffset = previousSize / (12 * 2 * 3);
      previousArray = NULL;
      previousSize = 0u;
    }

    // generate the cubes, 12 lines each
    std::list<octomap::OcTreeVolume>::iterator it_rec = m_grid_voxels.begin();
    unsigned int i = voxelOffset * 12 * 2 * 3;
    while (voxelOffset-- > 0) {
      ++it_rec;
    }
    double x,y,z;
    for (; it_rec != m_grid_voxels.end(); it_rec++) {

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
    clearCubes(&m_occupiedArray, m_occupiedCap, &m_occupiedColorArray);
    clearCubes(&m_occupiedThresArray, m_occupiedThresCap, &m_occupiedThresColorArray);
    clearCubes(&m_freeArray, m_freeCap);
    clearCubes(&m_freeThresArray, m_freeThresCap);
    clearCubes(&m_selectionArray, m_selectionSize);
    m_occupiedSize = m_occupiedThresSize = m_freeSize = m_freeThresSize = m_selectionSize = 0u;
    m_occupiedThresColorSize = m_occupiedColorSize = 0u;
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

  void OcTreeDrawer::buildVoxels(unsigned int timeout){
    QElapsedTimer timer;
    timer.start();

    switch (m_regeneration.phase)
    {
    case OcTreeDrawer::NONE:
    default:
      break;

    case OcTreeDrawer::INIT:
      m_regeneration.cnt_occupied = m_regeneration.cnt_occupied_thres = m_regeneration.cnt_free = m_regeneration.cnt_free_thres = 0;
      m_regeneration.it->begin(m_max_tree_depth);
      m_regeneration.phase = OcTreeDrawer::COUNT;
      m_regeneration.progress = 0;
      m_regeneration.maxProgress = m_regeneration.it->getTreeSize();
      if (!m_gridVoxelsBuilt){
        clearOcTreeStructure();
        m_grid_voxels.clear();
      }
      // Don't break;
    case OcTreeDrawer::COUNT:
      {
        // TODO: resolve a better way to temporarily change the occupancy threshold.
        double occupancyThresCache = m_regeneration.it->getOccupancyThres();
        m_regeneration.it->setOccupancyThres(m_occupancyThreshold);

        // walk the tree one to find the number of nodes in each category
        // (this is used to set up the OpenGL arrays)
        // TODO: this step may be left out, if we maintained the GLArrays in std::vectors instead...
        while (m_regeneration.it->isValid() && (timeout == 0 || timer.elapsed() < timeout)){
          ++m_regeneration.progress;
          if (m_regeneration.it->isLeaf()) { 
            if (m_regeneration.it->isOccupied()){ // occupied voxels
              if (m_regeneration.it->isAtThreshold()) ++m_regeneration.cnt_occupied_thres;
              else                               ++m_regeneration.cnt_occupied;
            }
            else if (m_regeneration.showAll) { // freespace voxels
              if (m_regeneration.it->isAtThreshold()) ++m_regeneration.cnt_free_thres;
              else                               ++m_regeneration.cnt_free;
            }
          }
          m_regeneration.it->moveNext();
        }    
        // Restore occupancy threshold.
        m_regeneration.it->setOccupancyThres(occupancyThresCache);
        
        if (!m_regeneration.it->isValid()){
          // Done counting. Setup GL arrays and move to the next phase.
          m_regeneration.phase = OcTreeDrawer::BUILD;
          m_regeneration.progress = 0;
          // setup GL arrays for cube quads and cube colors
          initGLArrays(m_regeneration.cnt_occupied      , m_occupiedCap     , &m_occupiedArray     , &m_occupiedColorArray);
          initGLArrays(m_regeneration.cnt_occupied_thres, m_occupiedThresCap, &m_occupiedThresArray, &m_occupiedThresColorArray);
          initGLArrays(m_regeneration.cnt_free          , m_freeCap         , &m_freeArray, NULL);
          initGLArrays(m_regeneration.cnt_free_thres    , m_freeThresCap    , &m_freeThresArray, NULL);

          m_occupiedThresSize = m_freeThresSize = m_occupiedSize = m_freeSize = m_selectionSize = 0u;
          m_occupiedThresColorSize = m_occupiedColorSize = 0u;

          m_regeneration.it->begin(this->m_max_tree_depth);
        }
      }
      break;

    case OcTreeDrawer::BUILD:
      {
        // TODO: resolve a better way to temporarily change the occupancy threshold.
        double occupancyThresCache = m_regeneration.it->getOccupancyThres();
        m_regeneration.it->setOccupancyThres(m_occupancyThreshold);

        double minX, minY, minZ, maxX, maxY, maxZ;
        m_regeneration.it->getMetricMin(minX, minY, minZ);
        m_regeneration.it->getMetricMax(maxX, maxY, maxZ);

        // set min/max Z for color height map
        m_zMin = minZ;
        m_zMax = maxZ;

        std::vector<octomath::Vector3> cube_template;
        initCubeTemplate(origin, cube_template);

        OcTreeVolume voxel; // current voxel, possibly transformed 
        while (m_regeneration.it->isValid() && (timeout == 0 || timer.elapsed() < timeout)) {
            ++m_regeneration.progress;
            if (m_regeneration.it->isLeaf()) { // voxels for leaf nodes
              if (m_regeneration.uses_origin) 
                voxel = OcTreeVolume(origin.rot().rotate(m_regeneration.it->getCoordinate()), m_regeneration.it->getSize());
              else 
                voxel = OcTreeVolume(m_regeneration.it->getCoordinate(), m_regeneration.it->getSize());

              if (m_regeneration.it->isOccupied()){ // occupied voxels
                if (m_regeneration.it->isAtThreshold()) {
                  m_occupiedThresSize = generateCube(voxel, cube_template, m_occupiedThresSize, &m_occupiedThresArray);
                  unsigned char r, g, b;
                  if (!m_regeneration.it->getColor(r, g, b)){
                    m_occupiedThresColorSize = setCubeColorHeightmap(voxel, m_occupiedThresColorSize, &m_occupiedThresColorArray);
                  } else {
                    m_occupiedThresColorSize = setCubeColorRGBA(r, g, b, 
                                                                (unsigned char) (m_regeneration.it->getOccupancy() * 255.),
                                                                m_occupiedThresColorSize, &m_occupiedThresColorArray);
                  }
                }
                else {
                  m_occupiedSize = generateCube(voxel, cube_template, m_occupiedSize, &m_occupiedArray);
                  unsigned char r, g, b;
                  if (!m_regeneration.it->getColor(r, g, b)){
                    m_occupiedColorSize = setCubeColorHeightmap(voxel, m_occupiedColorSize, &m_occupiedColorArray);
                  } else {
                    m_occupiedColorSize = setCubeColorRGBA(r, g, b, 
                                                           (unsigned char)(m_regeneration.it->getOccupancy() * 255.),
                                                           m_occupiedColorSize, &m_occupiedColorArray);
                  }
                }
              }
              else if (m_regeneration.showAll) { // freespace voxels
                if (m_regeneration.it->isAtThreshold()) {
                  m_freeThresSize = generateCube(voxel, cube_template, m_freeThresSize, &m_freeThresArray);
                }
                else {
                  m_freeSize = generateCube(voxel, cube_template, m_freeSize, &m_freeArray);
                }
              }
            }

            else { // inner node voxels (for grid structure only)
              if (!m_gridVoxelsBuilt){
                if (m_regeneration.showAll) {
                  if (m_regeneration.uses_origin)
                    voxel = OcTreeVolume(origin.rot().rotate(m_regeneration.it->getCoordinate()), m_regeneration.it->getSize());
                  else
                    voxel = OcTreeVolume(m_regeneration.it->getCoordinate(), m_regeneration.it->getSize());
                  m_grid_voxels.push_back(voxel);
                }
              }
            }      
            m_regeneration.it->moveNext();
        } // end for all voxels
        // Restore occupancy threshold.
        m_regeneration.it->setOccupancyThres(occupancyThresCache);

        // Expand octree grid.
        m_octree_grid_vis_initialized = false;

        if(m_drawOcTreeGrid)
          initOctreeGridVis(true);

        if (!m_regeneration.it->isValid()){
          m_gridVoxelsBuilt = true;
          m_regeneration.phase = OcTreeDrawer::COMPLETE;
          m_regeneration.progress = m_regeneration.maxProgress = 0;
        }
      }
      break;

    case OcTreeDrawer::COMPLETE:
      m_regeneration.phase = OcTreeDrawer::NONE;
      break;
    }
  }

} // namespace


