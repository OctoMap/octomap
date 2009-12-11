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
#define ML_OCC_PROB_THRES 0.5
#define CLAMPING_THRES_MIN -2
#define CLAMPING_THRES_MAX 3.5
#define UNKOWN_AS_OBSTACLE false

  // TODO : params of tree (const params pointer?)


  /**
   * Nodes to be used in OcTree, our main data structure.
   */
  class OcTreeNode : public AbstractOcTreeNode {

  public:

    OcTreeNode();
    virtual ~OcTreeNode();

    /// \return a pointer to the i th child of the node. The child needs to exist.
    virtual OcTreeNode* getChild(unsigned int i);
    /// \return a const pointer to the i th child of the node. The child needs to exist.
    virtual const OcTreeNode* getChild(unsigned int i) const;
    virtual bool createChild(unsigned int i);

    /// \return true if the i th child exists
    bool childExists(unsigned int i) const;
    /// \return true if the node has at least one child
    bool hasChildren() const;
    /// A node is collapsible if all children exist, don't have children of their own
    /// and are completely binary.
    bool collapsible() const;

    bool isDelta() const;
    virtual void convertToBinary();
 
    /**
     * @return mean of all child probabilities (if they exist), in log odds
     */
    double getMeanChildLogOdds() const;

    /**
     * @return max of all child probabilities (when they exist), in log odds
     */
    double getMaxChildLogOdds() const;

    void integrateHit();
    void integrateMiss();
    double getOccupancy() const;
    bool isOccupied() const;

    float getLogOdds() const{ return log_odds_occupancy; }
    void setLogOdds(float l) { log_odds_occupancy = l; }


    /**
     * Prunes a node when it is collapsible
     *
     * @return success of pruning
     */
    bool pruneNode();

    // file I/O
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s);

  protected:

    double logodds(double p) const;
    void updateLogOdds(double p);
    double prior() const;

    void allocChildren();
    bool pruneBinary();

    float log_odds_occupancy; // store log odds occupancy probability
    OcTreeNode** itsChildren; // pointer to children, may be NULL
  };


  // =========================================
  // for memory computation only
  class OcTreeNodeLight {
  public:
    float log_odds_occupancy;
    OcTreeNodeLight* itsChildren;
  };


  // for memory computation only
  class OcTreeNodeEightPointers {
  public:
    float log_odds_occupancy;
    char data;
    OcTreeNodeEightPointers* itsChildren[8];
  };

} // end namespace



#endif
