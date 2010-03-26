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
    void setOcTree(const octomap::OcTree &octree);
    /// sets alpha level for occupied cells
    void setAlphaOccupied(double alpha);

  protected:
    void clearOcTree();
    void clearOcTreeStructure();

    void drawOctreeGrid() const;
    void drawOctreeCells() const;
    void drawFreespace() const;
    void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize,
        GLfloat* cubeColorArray = NULL) const;

    //! setup octree visualizations
    void initOctreeCubeVis(
        const std::list<octomap::OcTreeVolume>& occupied_voxels,
        const std::list<octomap::OcTreeVolume>& freespace_voxels,
        const std::list<octomap::OcTreeVolume>& occupied_delta_voxels,
        const std::list<octomap::OcTreeVolume>& freespace_delta_voxels,
        const std::list<octomap::OcTreeVolume>& changed_free_voxels);

    void generateCubes(const std::list<octomap::OcTreeVolume>& points,
        GLfloat** gl_array, GLfloat* gl_color_array = NULL);
    void initOctreeGridVis();

    /**
     * Add voxels of OcTree to viewer, adjusts view and bounding box to contain all points
     */
    void setOcTreeVoxels(std::list<octomap::OcTreeVolume>& occupied_voxels,
        std::list<octomap::OcTreeVolume>& freespace_voxels, std::list<
            octomap::OcTreeVolume>& occupied_delta_voxels, std::list<
            octomap::OcTreeVolume>& freespace_delta_voxels, std::list<
            octomap::OcTreeVolume>& grid_voxels,
        std::list<octomap::OcTreeVolume>& changed_free_voxels);

    //! OpenGL representation of Octree cells (cubes)

    GLfloat** octree_occupied_cells_vertex_array;
    unsigned int octree_occupied_cells_vertex_size;
    GLfloat** octree_freespace_cells_vertex_array;
    unsigned int octree_freespace_cells_vertex_size;
    GLfloat** octree_occupied_delta_cells_vertex_array;
    unsigned int octree_occupied_delta_cells_vertex_size;
    GLfloat** octree_freespace_delta_cells_vertex_array;
    unsigned int octree_freespace_delta_cells_vertex_size;
    GLfloat** octree_freespace_changed_cells_vertex_array;
    unsigned int octree_freespace_changed_cells_vertex_size;

    //! Color array for occupied cells (height)
    GLfloat* octree_occupied_cells_color_array;
    GLfloat* octree_occupied_delta_cells_color_array;

    //! OpenGL representation of Octree (grid structure)
    GLfloat* octree_grid_vertex_array;
    unsigned int octree_grid_vertex_size;

    std::list<octomap::OcTreeVolume> m_grid_voxels;

    bool m_drawOcTreeCells;
    bool m_drawOcTreeGrid;
    bool m_draw_freespace;
    bool m_draw_freespaceDeltaOnly;
    bool m_octree_grid_vis_initialized;
    bool m_octree_set;
    unsigned int m_max_tree_depth;
    double m_alphaOccupied;

  public: // public getters and setters
    void enableOcTree(bool enabled = true);
    void enableOcTreeCells(bool enabled = true) {m_drawOcTreeCells = enabled; };
    void enableFreespace(bool enabled = true) {m_draw_freespace = enabled; };
    void enableFreespaceDeltaOnly(bool enabled = true) {m_draw_freespaceDeltaOnly = enabled; };
    void setMax_tree_depth(unsigned int max_tree_depth) {m_max_tree_depth = max_tree_depth;};

  };
}

#endif /* OCTREEDRAWER_H_ */
