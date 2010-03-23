#ifndef OCTOMAP_OCTREE_NODE_H
#define OCTOMAP_OCTREE_NODE_H

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

#include "octomap_types.h"
#include "OccupancyOcTreeNode.h"

namespace octomap {


#define PROB_HIT  0.7
#define PROB_MISS 0.4
#define OCC_PROB_THRES 0.5
#define CLAMPING_THRES_MIN -2
#define CLAMPING_THRES_MAX 3.5
#define UNKOWN_AS_OBSTACLE false

  // TODO : params of tree (const params pointer?)


  /**
   * Nodes to be used in OcTree. They represent 3d occupancy grid cells.
   *
   * Hint: If a class is derived from OcTreeNode, you have to implement (at least) 
   * createChild, getChild, and getChild const. See OcTreeNodeLabeled for an example.
   *
   */
  class OcTreeNode : public OccupancyOcTreeNode<float> {

  public:

    OcTreeNode();
    virtual ~OcTreeNode();

    virtual OcTreeDataNode<float>* newNode() const;

    // overloaded, so that the return type is correct:
    virtual OcTreeNode* getChild(unsigned int i);
    virtual const OcTreeNode* getChild(unsigned int i) const;

    // -- node occupancy  ----------------------------

    /// integrate a measurement (beam ENDED in cell)
    virtual void integrateHit();
    /// integrate a measurement (beam PASSED in cell)
    virtual void integrateMiss();

    /// \return occupancy probability of node
    double getOccupancy() const;

    /// \return log odds representation of occupancy probability of node
    float getLogOdds() const{ return value; }
    /// sets log odds occupancy of node
    void setLogOdds(float l) { value = l; }

    /// \return true if occupancy probability of node is >= OCC_PROB_THRES
    virtual bool isOccupied() const;

    /// node has reached the given occupancy threshold (CLAMPING_THRES_MIN, CLAMPING_THRES_MAX)
    virtual bool atThreshold() const;

    /// rounds a node's occupancy value to the nearest clamping threshold (free or occupied),
    /// effectively setting occupancy to the maximum likelihood value
    virtual void toMaxLikelihood();
 
    /**
     * @return mean of all children's occupancy probabilities, in log odds
     */
    double getMeanChildLogOdds() const;

    /**
     * @return maximum of children's occupancy probabilities, in log odds
     */
    double getMaxChildLogOdds() const;

    /// update this node's occupancy according to its children's maximum occupancy
    virtual void updateOccupancyChildren(); 



    
    // -- I/O  ---------------------------------------

    /**
     * Read node from binary stream (max-likelihood value), recursively
     * continue with all children.
     *
     * This will set the log_odds_occupancy value of
     * all leaves to either free or occupied.
     *
     * @param s
     * @return
     */
    virtual std::istream& readBinary(std::istream &s);

    /**
     * Write node to binary stream (max-likelihood value),
     * recursively continue with all children.
     *
     * This will discard the log_odds_occupancy value, writing
     * all leaves as either free or occupied.
     *
     * @param s
     * @return
     */
    std::ostream& writeBinary(std::ostream &s) const;

  protected:

    double logodds(double p) const;
    void updateLogOdds(double p);

 // "value" stores log odds occupancy probability

  };


} // end namespace



#endif
