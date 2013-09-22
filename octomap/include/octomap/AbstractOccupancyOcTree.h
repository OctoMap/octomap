/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
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

#ifndef OCTOMAP_ABSTRACT_OCCUPANCY_OCTREE_H
#define OCTOMAP_ABSTRACT_OCCUPANCY_OCTREE_H

#include "AbstractOcTree.h"
#include "octomap_utils.h"
#include "OcTreeNode.h"
#include "OcTreeKey.h"
#include <cassert>
#include <fstream>


namespace octomap {

  /**
   * Interface class for all octree types that store occupancy. This serves
   * as a common base class e.g. for polymorphism and contains common code
   * for reading and writing binary trees.
   */
  class AbstractOccupancyOcTree : public AbstractOcTree {
  public:
    AbstractOccupancyOcTree();
    virtual ~AbstractOccupancyOcTree() {};

    //-- IO

    /**
     * Writes OcTree to a binary file using writeBinary().
     * The OcTree is first converted to the maximum likelihood estimate and pruned.
     * @return success of operation
     */
    bool writeBinary(const std::string& filename);

    /**
     * Writes compressed maximum likelihood OcTree to a binary stream.
     * The OcTree is first converted to the maximum likelihood estimate and pruned
     * for maximum compression.
     * @return success of operation
     */
    bool writeBinary(std::ostream &s);

    /**
     * Writes OcTree to a binary file using writeBinaryConst().
     * The OcTree is not changed, in particular not pruned first.
     * Files will be smaller when the tree is pruned first or by using
     * writeBinary() instead.
     * @return success of operation
     */
    bool writeBinaryConst(const std::string& filename) const;

    /**
     * Writes the maximum likelihood OcTree to a binary stream (const variant).
     * Files will be smaller when the tree is pruned first or by using
     * writeBinary() instead.
     * @return success of operation
     */
    bool writeBinaryConst(std::ostream &s) const;

    /// Writes the actual data, implemented in OccupancyOcTreeBase::writeBinaryData()
    virtual std::ostream& writeBinaryData(std::ostream &s) const = 0;
    
    /**
     * Reads an OcTree from an input stream.
     * Existing nodes of the tree are deleted before the tree is read.
     * @return success of operation
     */
    bool readBinary(std::istream &s);
    
    /**
     * Reads OcTree from a binary file.
     * Existing nodes of the tree are deleted before the tree is read.
     * @return success of operation
     */
    bool readBinary(const std::string& filename);

    /// Reads the actual data, implemented in OccupancyOcTreeBase::readBinaryData()
    virtual std::istream& readBinaryData(std::istream &s) = 0;

    // -- occupancy queries

    /// queries whether a node is occupied according to the tree's parameter for "occupancy"
    inline bool isNodeOccupied(const OcTreeNode* occupancyNode) const{
      return (occupancyNode->getLogOdds() >= this->occ_prob_thres_log);
    }

    /// queries whether a node is occupied according to the tree's parameter for "occupancy"
    inline bool isNodeOccupied(const OcTreeNode& occupancyNode) const{
      return (occupancyNode.getLogOdds() >= this->occ_prob_thres_log);
    }

    /// queries whether a node is at the clamping threshold according to the tree's parameter
    inline bool isNodeAtThreshold(const OcTreeNode* occupancyNode) const{
      return (occupancyNode->getLogOdds() >= this->clamping_thres_max
          || occupancyNode->getLogOdds() <= this->clamping_thres_min);
    }

    /// queries whether a node is at the clamping threshold according to the tree's parameter
    inline bool isNodeAtThreshold(const OcTreeNode& occupancyNode) const{
      return (occupancyNode.getLogOdds() >= this->clamping_thres_max
          || occupancyNode.getLogOdds() <= this->clamping_thres_min);
    }

    // - update functions

    /**
     * Manipulate log_odds value of voxel directly
     *
     * @param key of the NODE that is to be updated
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual OcTreeNode* updateNode(const OcTreeKey& key, float log_odds_update, bool lazy_eval = false) = 0;

    /**
     * Manipulate log_odds value of voxel directly.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual OcTreeNode* updateNode(const point3d& value, float log_odds_update, bool lazy_eval = false) = 0;

    /**
     * Integrate occupancy measurement.
     *
     * @param key of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual OcTreeNode* updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval = false) = 0;

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual OcTreeNode* updateNode(const point3d& value, bool occupied, bool lazy_eval = false) = 0;

    virtual void toMaxLikelihood() = 0;

    //-- parameters for occupancy and sensor model:

    /// sets the threshold for occupancy (sensor model)
    void setOccupancyThres(double prob){occ_prob_thres_log = logodds(prob); }
    /// sets the probability for a "hit" (will be converted to logodds) - sensor model
    void setProbHit(double prob){prob_hit_log = logodds(prob); assert(prob_hit_log >= 0.0);}
    /// sets the probability for a "miss" (will be converted to logodds) - sensor model
    void setProbMiss(double prob){prob_miss_log = logodds(prob); assert(prob_miss_log <= 0.0);}
    /// sets the minimum threshold for occupancy clamping (sensor model)
    void setClampingThresMin(double thresProb){clamping_thres_min = logodds(thresProb); }
    /// sets the maximum threshold for occupancy clamping (sensor model)
    void setClampingThresMax(double thresProb){clamping_thres_max = logodds(thresProb); }

    /// @return threshold (probability) for occupancy - sensor model
    double getOccupancyThres() const {return probability(occ_prob_thres_log); }
    /// @return threshold (logodds) for occupancy - sensor model
    float getOccupancyThresLog() const {return occ_prob_thres_log; }

    /// @return probability for a "hit" in the sensor model (probability)
    double getProbHit() const {return probability(prob_hit_log); }
    /// @return probability for a "hit" in the sensor model (logodds)
    float getProbHitLog() const {return prob_hit_log; }
    /// @return probability for a "miss"  in the sensor model (probability)
    double getProbMiss() const {return probability(prob_miss_log); }
    /// @return probability for a "miss"  in the sensor model (logodds)
    float getProbMissLog() const {return prob_miss_log; }

    /// @return minimum threshold for occupancy clamping in the sensor model (probability)
    double getClampingThresMin() const {return probability(clamping_thres_min); }
    /// @return minimum threshold for occupancy clamping in the sensor model (logodds)
    float getClampingThresMinLog() const {return clamping_thres_min; }
    /// @return maximum threshold for occupancy clamping in the sensor model (probability)
    double getClampingThresMax() const {return probability(clamping_thres_max); }
    /// @return maximum threshold for occupancy clamping in the sensor model (logodds)
    float getClampingThresMaxLog() const {return clamping_thres_max; }




  protected:
    /// Try to read the old binary format for conversion, will be removed in the future
    bool readBinaryLegacyHeader(std::istream &s, unsigned int& size, double& res);
    
    // occupancy parameters of tree, stored in logodds:
    float clamping_thres_min;
    float clamping_thres_max;
    float prob_hit_log;
    float prob_miss_log;
    float occ_prob_thres_log;

    static const std::string binaryFileHeader;
  };

}; // end namespace


#endif
