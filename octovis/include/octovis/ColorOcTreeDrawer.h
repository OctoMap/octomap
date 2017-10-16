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

#ifndef OCTOVIS_COLOR_OCTREEDRAWER_H_
#define OCTOVIS_COLOR_OCTREEDRAWER_H_

#include <octovis/OcTreeDrawer.h>
#include <octomap/ColorOcTree.h>

namespace octomap {

  class ColorOcTreeDrawer : public OcTreeDrawer {
  public:
    ColorOcTreeDrawer();
    virtual ~ColorOcTreeDrawer();

    virtual void setOcTree(const AbstractOcTree& tree_pnt, const pose6d& origin, int map_id_);

  protected:
    /// Specialisation of @c AbstractOcTreeIterator for handling @c ColorOcTree instances.
    class ColorOcTreeIterator : public AbstractOcTreeIterator {
    public:
      ColorOcTreeIterator(const ColorOcTree *tree);

      virtual void setOccupancyThres(double threshold);
      virtual double getOccupancyThres() const;
      virtual unsigned int getTreeSize() const;
      virtual void getMetricMin(double& x, double& y, double& z) const;
      virtual void getMetricMax(double& x, double& y, double& z) const;
      virtual bool isValid() const;
      virtual bool begin(unsigned int max_tree_depth);
      virtual bool moveNext();
      virtual bool getColor(unsigned char& r, unsigned char& g, unsigned char& b) const;
      virtual float getOccupancy() const;
      virtual point3d getCoordinate() const;
      virtual double getSize() const;
      virtual bool isLeaf() const;
      virtual bool isOccupied() const;
      virtual bool isAtThreshold() const;

    private:
      const ColorOcTree* m_tree;
      ColorOcTree::tree_iterator m_it;
    };
  };


} // end namespace

#endif
