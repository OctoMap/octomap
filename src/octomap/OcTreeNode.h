#ifndef OCTOMAP_OCTREE_NODE_H
#define OCTOMAP_OCTREE_NODE_H

// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "octomap_types.h"
#include "OcTreeDataNode.h"

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
  class OcTreeNode : public OcTreeDataNode<float> {

  public:

    OcTreeNode();
    ~OcTreeNode();

    // overloaded, so that the return type is correct:
    OcTreeNode* getChild(unsigned int i);
    const OcTreeNode* getChild(unsigned int i) const;
    bool createChild(unsigned int i);

    // -- node occupancy  ----------------------------

    /// integrate a measurement (beam ENDED in cell)
    void integrateHit();
    /// integrate a measurement (beam PASSED in cell)
    void integrateMiss();

    /// \return occupancy probability of node
    double getOccupancy() const;

    /// \return log odds representation of occupancy probability of node
    float getLogOdds() const{ return value; }
    /// sets log odds occupancy of node
    void setLogOdds(float l) { value = l; }

    /// \return true if occupancy probability of node is >= OCC_PROB_THRES
    bool isOccupied() const;

    /// node has reached the given occupancy threshold (CLAMPING_THRES_MIN, CLAMPING_THRES_MAX)
    bool atThreshold() const;

    /// rounds a node's occupancy value to the nearest clamping threshold (free or occupied),
    /// effectively setting occupancy to the maximum likelihood value
    void toMaxLikelihood();
 
    /**
     * @return mean of all children's occupancy probabilities, in log odds
     */
    double getMeanChildLogOdds() const;

    /**
     * @return maximum of children's occupancy probabilities, in log odds
     */
    double getMaxChildLogOdds() const;

    /// update this node's occupancy according to its children's maximum occupancy
    void updateOccupancyChildren();



    
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
    std::istream& readBinary(std::istream &s);

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
