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
    /// Current construction phase for amortised initialisation.
    enum BuildPhase {
      NONE,     ///< No phase. Nothing to do.
      INIT,     ///< Initialisation phase.
      COUNT,    ///< Note counting phase.
      BUILD,    ///< Building voxels.
      COMPLETE  ///< Construction complete in this update.
    };

    OcTreeDrawer();
    virtual ~OcTreeDrawer();
    void clear();

    /// Updates the amortised construction of the render items.
    ///
    /// Processing continues until @p timeout is reached, or construction is completed.
    /// The progress may be monitored by checking the return value and the @p progress
    /// parameters.
    ///
    /// @param timeout Processing timeout in milliseconds. Use zero to force completion (no timeout).
    /// @param[out] progress Optional pointer to write current progress value. This is generally the
    ///   number of tree nodes touched in the current phase.
    /// @param[out] maxProgress Optional pointer to write maximum progress to. This is generally the
    ///   total number of nodes in the tree.
    /// @return The current construction phase for the tree. See @c BuildPhase.
    BuildPhase update(unsigned int timeout, unsigned int* progress = 0, unsigned int* maxProgress = 0);

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

    void setOccupancyThreshold(double threshold);// { m_occupancyThreshold = threshold; }
    inline double getOccupancyThreshold() const { return m_occupancyThreshold; }

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
      

    void initOctreeGridVis(bool expand = false);

    void buildVoxels(unsigned int timeout = 1000/30); 

    /// Abstract tree iteration adaptor used to process the current tree type.
    ///
    /// General usage is to call @c begin(), call various node methods while @c isValid() is true
    /// and use @c moveNext() to skip to the next node.
    class AbstractOcTreeIterator {
    public:
      /// Virtual destructor (empty).
      virtual inline ~AbstractOcTreeIterator() {}

      /// Sets the occupancy threshold for the tree. Nodes are considered occupied above this threshold.
      virtual void setOccupancyThres(double threshold) = 0;
      /// Returns the current occupancy threshold.
      virtual double getOccupancyThres() const = 0;
      /// Returns the number of nodes in the tree.
      virtual unsigned int getTreeSize() const = 0;
      /// minimum value of the bounding box of all known space in x, y, z
      virtual void getMetricMin(double& x, double& y, double& z) const = 0;
      /// maximum value of the bounding box of all known space in x, y, z
      virtual void getMetricMax(double& x, double& y, double& z) const = 0;
      /// Returns true if the currently referencing a valid node.
      virtual bool isValid() const = 0;
      /// Begins iteration of the tree.
      /// @param max_tree_depth Maximum traversal depth.
      /// @return True if the tree is not empty.
      virtual bool begin(unsigned int max_tree_depth) = 0;
      /// Moves to the next node.
      /// @return True if successfully referencing a new node, false if iteration is complete.
      virtual bool moveNext() = 0;
      /// Requests the color for the current node. Only for nodes supporting color.
      /// RGB values are only valid when returning true.
      ///
      /// Do not call when @c isValid() is false.
      /// @param[out] r Red color channel.
      /// @param[out] g Green color channel.
      /// @param[out] b Blue color channel.
      /// @return true if color is supported and valid for the current node.
      virtual bool getColor(unsigned char& r, unsigned char& g, unsigned char& b) const { return false; }
      /// Returns the occupancy value for the current node.
      /// Do not call when @c isValid() is false.
      virtual float getOccupancy() const = 0;
      /// Returns the coordinate for the current node.
      /// Do not call when @c isValid() is false.
      virtual point3d getCoordinate() const = 0;
      /// Returns the size of the current node.
      /// Do not call when @c isValid() is false.
      virtual double getSize() const = 0;
      /// Returns true if the current node is a leaf.
      /// Do not call when @c isValid() is false.
      virtual bool isLeaf() const = 0;
      /// Returns true if the current node is occupied.
      /// Do not call when @c isValid() is false.
      virtual bool isOccupied() const = 0;
      /// Returns true if the current node is at the occupancy threshold.
      /// Do not call when @c isValid() is false.
      virtual bool isAtThreshold() const = 0;
    };

    /// Specialisation of @c AbstractOcTreeIterator for handling @c OcTree instances.
    class OcTreeIterator : public AbstractOcTreeIterator {
    public:
      OcTreeIterator(const OcTree *tree);

      virtual void setOccupancyThres(double threshold);
      virtual double getOccupancyThres() const;
      virtual unsigned int getTreeSize() const;
      virtual void getMetricMin(double& x, double& y, double& z) const;
      virtual void getMetricMax(double& x, double& y, double& z) const;
      virtual bool isValid() const;
      virtual bool begin(unsigned int max_tree_depth);
      virtual bool moveNext();
      virtual float getOccupancy() const;
      virtual point3d getCoordinate() const;
      virtual double getSize() const;
      virtual bool isLeaf() const;
      virtual bool isOccupied() const;
      virtual bool isAtThreshold() const;

    private:
      const OcTree* m_tree;
      OcTree::tree_iterator m_it;
    };

    //! Data structure tracking progressive regeneration of the octree for display.
    //! This is used to amortise generation of render primitives.
    struct Regeneration {
      BuildPhase phase; ///< Current construction phase.
      bool showAll;
      bool uses_origin;
      unsigned int progress, maxProgress;
      unsigned int cnt_occupied, cnt_occupied_thres, cnt_free, cnt_free_thres;
      AbstractOcTreeIterator* it;

      Regeneration();
      ~Regeneration();
    };

    //! OpenGL representation of Octree cells (cubes)

    GLfloat** m_occupiedThresArray;
    unsigned int m_occupiedThresSize, m_occupiedThresCap;
    GLfloat** m_freeThresArray;
    unsigned int m_freeThresSize, m_freeThresCap;
    GLfloat** m_occupiedArray;
    unsigned int m_occupiedSize, m_occupiedCap;
    GLfloat** m_freeArray;
    unsigned int m_freeSize, m_freeCap;
    GLfloat** m_selectionArray;
    unsigned int m_selectionSize;

    //! Color array for occupied cells (height)
    GLfloat* m_occupiedThresColorArray;
    unsigned int m_occupiedThresColorSize;
    GLfloat* m_occupiedColorArray;
    unsigned int m_occupiedColorSize;

    //! OpenGL representation of Octree (grid structure)
    // TODO: put in its own drawer object!
    GLfloat* octree_grid_vertex_array;
    unsigned int octree_grid_vertex_size;

    std::list<octomap::OcTreeVolume> m_grid_voxels;
    bool m_gridVoxelsBuilt;

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
    double m_occupancyThreshold;

    octomap::pose6d origin;
    octomap::pose6d initial_origin;

    int map_id;

    Regeneration m_regeneration;
  };
}

#endif /* OCTREEDRAWER_H_ */
