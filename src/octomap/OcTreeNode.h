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
#include "AbstractOcTreeNode.h"

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
   */
  class OcTreeNode : public AbstractOcTreeNode {

  public:

    OcTreeNode();
    virtual ~OcTreeNode();

    // -- children  ----------------------------------

    /// initialize i-th child
    virtual bool createChild(unsigned int i);

    /// \return true if the i-th child exists
    virtual bool childExists(unsigned int i) const;

    /// \return a pointer to the i-th child of the node. The child needs to exist.
    virtual OcTreeNode* getChild(unsigned int i);

    /// \return a const pointer to the i-th child of the node. The child needs to exist.
    virtual const OcTreeNode* getChild(unsigned int i) const;

    /// \return true if the node has at least one child
    virtual bool hasChildren() const;


    // -- node occupancy  ----------------------------

    /// integrate a measurement (beam ENDED in cell)
    virtual void integrateHit();
    /// integrate a measurement (beam PASSED in cell)
    virtual void integrateMiss();

    /// \return occupancy probability of node
    double getOccupancy() const;

    /// \return log odds representation of occupancy probability of node
    float getLogOdds() const{ return log_odds_occupancy; }
    /// sets log odds occupancy of node
    void setLogOdds(float l) { log_odds_occupancy = l; }

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


    // -- pruning of children  -----------------------

    /// A node is collapsible if all children exist, don't have children of their own
    /// and have the same occupancy value
    bool collapsible() const;

    /**
     * Prunes a node when it is collapsible
     * @return true if pruning was successful
     */
    bool pruneNode();

    /**
     * Expands a node (reverse of pruning): All children are created and
     * their occupancy probability is set to the node's value. 
     *
     * You need to verify that this is indeed a pruned node (i.e. not a 
     * leaf at the lowest level)
     *
     */
    void expandNode();

    
    // -- I/O  ---------------------------------------

    // read from / write to binary stream
    virtual std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s);

  protected:

    void allocChildren();

    double logodds(double p) const;
    void updateLogOdds(double p);

  protected:

    float log_odds_occupancy; // stores log odds occupancy probability
    OcTreeNode** itsChildren; // pointer to children, may be NULL
  };


} // end namespace



#endif
