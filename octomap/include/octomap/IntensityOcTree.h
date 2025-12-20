/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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

#ifndef OCTOMAP_INTENSITY_OCTREE_H
#define OCTOMAP_INTENSITY_OCTREE_H

#include <iostream>

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap
{

// forward declaration for "friend"
class IntensityOcTree;

// node definition
class IntensityOcTreeNode : public OcTreeNode
{
public:
    friend class IntensityOcTree;// needs access to node children (inherited)

    // Constructors
    IntensityOcTreeNode() : OcTreeNode(), intensity(0.0)
    {
    }

    IntensityOcTreeNode(const IntensityOcTreeNode &rhs)
        : OcTreeNode(rhs), intensity(rhs.intensity)
    {
    }

    // Comparator
    bool operator==(const IntensityOcTreeNode &rhs) const
    {
        return (rhs.value == value && rhs.intensity == intensity);
    }

    // Payload helpers
    inline double getIntensity() const
    {
        return intensity;
    }
    inline void setIntensity(double i)
    {
        intensity = i;
    }

    // Copy when replacing a node
    void copyData(const IntensityOcTreeNode &from)
    {
        OcTreeNode::copyData(from);
        this->intensity = from.getIntensity();
    }

    // Update occupancy and intensity of inner nodes
    inline void updateIntensityChildren();
    double getAverageChildIntensity() const;
    inline bool isIntensitySet() const
    {
        return (ZERO_EPSILON < intensity);
    }

    // Serialization and deserialization
    std::istream &readData(std::istream &s);
    std::ostream &writeData(std::ostream &s) const;

protected:
    double intensity;
    static constexpr double ZERO_EPSILON = 1e-6;// threshold for checking if intensity has been set
};

// tree definition
class IntensityOcTree : public OccupancyOcTreeBase<IntensityOcTreeNode>
{
public:
    // Default constructor, sets resolution of leafs
    IntensityOcTree(double resolution);

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    IntensityOcTree *create() const override
    {
        return new IntensityOcTree(resolution);
    }

    std::string getTreeType() const
    {
        return "IntensityOcTree";
    }

    /**
   * Prunes a node when it is collapsible. This overloaded
   * version only considers the node occupancy for pruning,
   * different intensities of child nodes are ignored.
   * @return true if pruning was successful
   */
    virtual bool pruneNode(IntensityOcTreeNode *node);

    void updateInnerOccupancy();

    void computeUpdateKeys(const octomap::Pointcloud &scan,
                           const octomath::Vector3 &origin,
                           octomap::KeySet &free_cells,
                           octomap::KeySet &occupied_cells,
                           double maxrange = -1.0);

    IntensityOcTreeNode *integrateNodeIntensity(const OcTreeKey &key,
                                                double intensity);
    IntensityOcTreeNode *integrateNodeIntensity(float x, float y, float z,
                                                double intensity);

protected:
    void updateInnerOccupancyRecurs(IntensityOcTreeNode *node, unsigned depth);

    /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
    class StaticMemberInitializer
    {
    public:
        StaticMemberInitializer()
        {
            IntensityOcTree *tree = new IntensityOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }
        /**
     * Dummy function to ensure that MSVC does not drop the
     * StaticMemberInitializer, causing this tree failing to register.
     * Needs to be called from the constructor of this octree.
     */
        void ensureLinking()
        {
        }
    };

    /// to ensure static initialization (only once)
    static StaticMemberInitializer intensityOcTreeMemberInit;
};

}// namespace octomap

#endif// OCTOMAP_INTENSITY_OCTREE_H