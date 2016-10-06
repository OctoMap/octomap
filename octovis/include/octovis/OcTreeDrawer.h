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

#ifndef OCTREEDRAWER_H_
#define OCTREEDRAWER_H_

#include "SceneObject.h"

namespace octomap {

  class OcTreeDrawer: public octomap::SceneObject {
  public:
    OcTreeDrawer();
    virtual ~OcTreeDrawer();
    void clear();

    void draw() const;

    // initialization of drawer  -------------------------

    /// sets a new OcTree that should be drawn by this drawer
    void setOcTree(const AbstractOcTree& octree){
      octomap::pose6d o; // initialized to (0,0,0) , (0,0,0,1) by default
      setOcTree(octree, o, 0);
    }

    /// sets a new OcTree that should be drawn by this drawer
    /// origin specifies a global transformation that should be applied
    virtual void setOcTree(const AbstractOcTree& octree, const octomap::pose6d& origin, int map_id_);

    // modification of existing drawer  ------------------

    /// sets a new selection of the current OcTree to be drawn
    void setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedPoints);

    /// clear the visualization of the OcTree selection
    void clearOcTreeSelection();

    /// sets alpha level for occupied cells
    void setAlphaOccupied(double alpha);
    void setAlternativeDrawing(bool flag){m_alternativeDrawing = flag;}

    void enableOcTree(bool enabled = true);
    void enableOcTreeCells(bool enabled = true) { m_update = true; m_drawOccupied = enabled; };
    void enableFreespace(bool enabled = true) { m_update = true; m_drawFree = enabled; };
    void enableSelection(bool enabled = true) { m_update = true; m_drawSelection = enabled; };
    void setMax_tree_depth(unsigned int max_tree_depth) { m_update = true; m_max_tree_depth = max_tree_depth;};

    // set new origin (move object)
    void setOrigin(octomap::pose6d t);
    void enableAxes(bool enabled = true) { m_update = true; m_displayAxes = enabled; };

  protected:
    //void clearOcTree();
    void clearOcTreeStructure();

    void drawOctreeGrid() const;
    void drawOccupiedVoxels() const;
    void drawFreeVoxels() const;
    void drawSelection() const;
    void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize,
        GLfloat* cubeColorArray = NULL) const;

    void drawAxes() const;

    //! Initializes the OpenGL visualization for a list of OcTreeVolumes
    //! The array is cleared first, if needed
    /// rotates cubes to correct reference frame
    void generateCubes(const std::list<octomap::OcTreeVolume>& voxels,
                       GLfloat*** glArray, unsigned int& glArraySize, 
                       octomath::Pose6D& origin,
                       GLfloat** glColorArray = NULL);
    
    //! clear OpenGL visualization
    void clearCubes(GLfloat*** glArray, unsigned int& glArraySize,
                    GLfloat** glColorArray = NULL);
    //! setup OpenGL arrays
    void initGLArrays(const unsigned int& num_cubes, unsigned int& glArraySize,
                       GLfloat*** glArray, GLfloat** glColorArray);
    //! setup cube template
    void initCubeTemplate(const octomath::Pose6D& origin,
                          std::vector<octomath::Vector3>& cube_template);
    //! add one cube to arrays
    unsigned int generateCube(const octomap::OcTreeVolume& v,
                              const std::vector<octomath::Vector3>& cube_template,
                              const unsigned int& current_array_idx,
                              GLfloat*** glArray);
    unsigned int setCubeColorHeightmap(const octomap::OcTreeVolume& v,
                                       const unsigned int& current_array_idx,
                                       GLfloat** glColorArray);
    unsigned int setCubeColorRGBA(const unsigned char& r, const unsigned char& g, 
                                  const unsigned char& b, const unsigned char& a,
                                  const unsigned int& current_array_idx,
                                  GLfloat** glColorArray);
      

    void initOctreeGridVis();

    //! OpenGL representation of Octree cells (cubes)

    GLfloat** m_occupiedThresArray;
    unsigned int m_occupiedThresSize;
    GLfloat** m_freeThresArray;
    unsigned int m_freeThresSize;
    GLfloat** m_occupiedArray;
    unsigned int m_occupiedSize;
    GLfloat** m_freeArray;
    unsigned int m_freeSize;
    GLfloat** m_selectionArray;
    unsigned int m_selectionSize;

    //! Color array for occupied cells (height)
    GLfloat* m_occupiedThresColorArray;
    GLfloat* m_occupiedColorArray;

    //! OpenGL representation of Octree (grid structure)
    // TODO: put in its own drawer object!
    GLfloat* octree_grid_vertex_array;
    unsigned int octree_grid_vertex_size;

    std::list<octomap::OcTreeVolume> m_grid_voxels;

    bool m_drawOccupied;
    bool m_drawOcTreeGrid;
    bool m_drawFree;
    bool m_drawSelection;
    bool m_octree_grid_vis_initialized;
    bool m_displayAxes;
    bool m_alternativeDrawing;
    mutable bool m_update;

    unsigned int m_max_tree_depth;
    double m_alphaOccupied;

    octomap::pose6d origin;
    octomap::pose6d initial_origin;

    int map_id;
  };
}

#endif /* OCTREEDRAWER_H_ */
