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

#ifndef OCTOMAP_OCTREE_STAMPED_H
#define OCTOMAP_OCTREE_STAMPED_H


#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ctime>

namespace octomap {
  
  // node definition
  class OcTreeNodeStamped : public OcTreeNode {    

  public:
    OcTreeNodeStamped() : OcTreeNode(), timestamp(0) {}

    OcTreeNodeStamped(const OcTreeNodeStamped& rhs) : OcTreeNode(rhs), timestamp(rhs.timestamp) {}

    bool operator==(const OcTreeNodeStamped& rhs) const{
      return (rhs.value == value && rhs.timestamp == timestamp);
    }
    
    // children
    inline OcTreeNodeStamped* getChild(unsigned int i) {
      return static_cast<OcTreeNodeStamped*> (OcTreeNode::getChild(i));
    }
    inline const OcTreeNodeStamped* getChild(unsigned int i) const {
      return static_cast<const OcTreeNodeStamped*> (OcTreeNode::getChild(i));
    }

    bool createChild(unsigned int i) {
      if (children == NULL) allocChildren();
      children[i] = new OcTreeNodeStamped();
      return true;
    }
    
    // timestamp
    inline unsigned int getTimestamp() const { return timestamp; }
    inline void updateTimestamp() { timestamp = (unsigned int) time(NULL);}
    inline void setTimestamp(unsigned int timestamp) {this->timestamp = timestamp; }

    // update occupancy and timesteps of inner nodes 
    inline void updateOccupancyChildren() {      
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
      updateTimestamp();
    }

  protected:
    unsigned int timestamp;
  };


  // tree definition
  class OcTreeStamped : public OccupancyOcTreeBase <OcTreeNodeStamped> {    

  public:
    /// Default constructor, sets resolution of leafs
	  OcTreeStamped(double resolution);
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTreeStamped* create() const {return new OcTreeStamped(resolution); }

    std::string getTreeType() const {return "OcTreeStamped";}

    //! \return timestamp of last update
    unsigned int getLastUpdateTime();

    void degradeOutdatedNodes(unsigned int time_thres);
    
    virtual void updateNodeLogOdds(OcTreeNodeStamped* node, const float& update) const;
    void integrateMissNoTime(OcTreeNodeStamped* node) const;

  protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a 
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTreeStamped* tree = new OcTreeStamped(0.1);
        AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {};
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeStampedMemberInit;
    
  };

} // end namespace

#endif
