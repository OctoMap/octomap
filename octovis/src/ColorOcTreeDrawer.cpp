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

#include <octovis/ColorOcTreeDrawer.h>

namespace octomap {

  ColorOcTreeDrawer::ColorOcTreeIterator::ColorOcTreeIterator(const ColorOcTree* tree) : m_tree(tree)
  {
  }

  void ColorOcTreeDrawer::ColorOcTreeIterator::setOccupancyThres(double threshold){
    // TODO: need a better way than using a const cast for this.
    const_cast<ColorOcTree*>(m_tree)->setOccupancyThres(threshold);
  }

  double ColorOcTreeDrawer::ColorOcTreeIterator::getOccupancyThres() const {
    return m_tree->getOccupancyThres();
  }

  unsigned int ColorOcTreeDrawer::ColorOcTreeIterator::getTreeSize() const {
    return m_tree->size();
  }

  void ColorOcTreeDrawer::ColorOcTreeIterator::getMetricMin(double& x, double& y, double& z) const {
    return m_tree->getMetricMin(x, y, z);
  }

  void ColorOcTreeDrawer::ColorOcTreeIterator::getMetricMax(double& x, double& y, double& z) const {
    return m_tree->getMetricMax(x, y, z);
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::isValid() const {
    return m_it != m_tree->end_tree();
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::begin(unsigned int max_tree_depth){
    m_it = m_tree->begin_tree(max_tree_depth);
    return m_it != m_tree->end_tree();
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::moveNext(){
    ++m_it;
    return m_it != m_tree->end_tree();
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::getColor(unsigned char& r, unsigned char& g, unsigned char& b) const {
    const ColorOcTreeNode::Color c = m_it->getColor();
    r = c.r;
    g = c.g;
    b = c.b;
    return true;
  }

  float ColorOcTreeDrawer::ColorOcTreeIterator::getOccupancy() const{
    return m_it->getOccupancy();
  }

  point3d ColorOcTreeDrawer::ColorOcTreeIterator::getCoordinate() const{
    return m_it.getCoordinate();
  }

  double ColorOcTreeDrawer::ColorOcTreeIterator::getSize() const{
    return m_it.getSize();
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::isLeaf() const{
    return m_it.isLeaf();
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::isOccupied() const{
    return m_tree->isNodeOccupied(*m_it);
  }

  bool ColorOcTreeDrawer::ColorOcTreeIterator::isAtThreshold() const{
    return m_tree->isNodeAtThreshold(*m_it);
  }

  ColorOcTreeDrawer::ColorOcTreeDrawer() 
    : OcTreeDrawer() {
  }

  ColorOcTreeDrawer::~ColorOcTreeDrawer() {
  }

  void ColorOcTreeDrawer::setOcTree(const AbstractOcTree& tree_pnt,
                                    const octomap::pose6d& origin_,
                                    int map_id_) {

    const ColorOcTree& tree = ((const ColorOcTree&) tree_pnt);

    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin_.rot());
    // origin is in global coords
    this->origin = origin_;
    
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    m_regeneration.showAll = (tree.size() < 5 * 1e6);
    m_regeneration.uses_origin = ( (origin_.rot().x() != 0.) && (origin_.rot().y() != 0.)
                                  && (origin_.rot().z() != 0.) && (origin_.rot().u() != 1.) );

    // Start amortised construction of the voxels.
    delete m_regeneration.it;
    m_regeneration.it = new ColorOcTreeIterator(&tree);
    m_regeneration.phase = OcTreeDrawer::INIT;
    buildVoxels();
  }

} // end namespace
