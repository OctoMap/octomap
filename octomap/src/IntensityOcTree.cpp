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

#include "octomap/IntensityOcTree.h"

namespace octomap
{
// Node implementation
std::istream &IntensityOcTreeNode::readData(std::istream &s)
{
    s.read((char *) &value, sizeof(value));// occupancy
    s.read(reinterpret_cast<char *>(&intensity), sizeof(intensity));

    return s;
}

std::ostream &IntensityOcTreeNode::writeData(std::ostream &s) const
{
    s.write((const char *) &value, sizeof(value));// occupancy
    s.write(reinterpret_cast<const char *>(&intensity), sizeof(intensity));

    return s;
}

double IntensityOcTreeNode::getAverageChildIntensity() const
{
    double sum = 0.0;
    unsigned cnt = 0;

    if (children != NULL)
    {
        for (unsigned i = 0; i < 8; ++i)
        {
            IntensityOcTreeNode *child =
                    static_cast<IntensityOcTreeNode *>(children[i]);

            if (child != NULL && child->isIntensitySet())
            {
                sum += child->getIntensity();
                ++cnt;
            }
        }
    }

    // If no children have intensity set, return a neutral default (0.0)
    // instead of reusing the parent node's current intensity.
    if (cnt > 0)
    {
        return sum / static_cast<double>(cnt);
    }
    else
    {
        return 0.0;
    }
}

void IntensityOcTreeNode::updateIntensityChildren()
{
    intensity = getAverageChildIntensity();
}

// Tree implementation
IntensityOcTree::IntensityOcTree(double resolution)
    : OccupancyOcTreeBase<IntensityOcTreeNode>(resolution)
{
    intensityOcTreeMemberInit.ensureLinking();
}


void IntensityOcTree::updateInnerOccupancy()
{
    this->updateInnerOccupancyRecurs(this->root, 0);
}

void IntensityOcTree::updateInnerOccupancyRecurs(IntensityOcTreeNode *node,
                                                 unsigned depth)
{
    if (nodeHasChildren(node))
    {
        if (depth < this->tree_depth)
        {
            for (unsigned i = 0; i < 8; ++i)
            {
                if (nodeChildExists(node, i))
                    updateInnerOccupancyRecurs(getNodeChild(node, i),
                                               depth + 1);
            }
        }
        node->updateOccupancyChildren();
        node->updateIntensityChildren();
    }
}


void IntensityOcTree::computeUpdateKeys(const octomap::Pointcloud &scan,
                                        const octomath::Vector3 &origin,
                                        octomap::KeySet &free_cells,
                                        octomap::KeySet &occupied_cells,
                                        double maxrange)
{
    // Delegates to base class
    computeUpdate(scan, octomap::point3d(origin.x(), origin.y(), origin.z()),
                  free_cells, occupied_cells, maxrange);
}

// Node Pruning
bool IntensityOcTree::pruneNode(IntensityOcTreeNode *node)
{
    if (!isNodeCollapsible(node))
        return false;

    // set value to children's values (all assumed equal)
    node->copyData(*(getNodeChild(node, 0)));

    // update intensity to represent the average of all children
    node->setIntensity(node->getAverageChildIntensity());

    // delete children
    for (unsigned int i = 0; i < 8; i++)
    {
        deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;

    return true;
}


// Integration
IntensityOcTreeNode *
IntensityOcTree::integrateNodeIntensity(const OcTreeKey &key, double intensity)
{
    IntensityOcTreeNode *n = search(key);
    if (n != 0)
    {
        if (n->isIntensitySet())
        {
            double prev_intensity = n->getIntensity();
            double node_prob = n->getOccupancy();

            double new_intensity = (prev_intensity * node_prob +
                                    intensity * (1.0 - node_prob));
            n->setIntensity(new_intensity);
        }
        else
        {
            n->setIntensity(intensity);
        }
    }
    return n;
}

IntensityOcTreeNode *IntensityOcTree::integrateNodeIntensity(float x, float y,
                                                             float z,
                                                             double intensity)
{
    OcTreeKey key;
    if (!this->coordToKeyChecked(point3d(x, y, z), key))
        return NULL;
    return integrateNodeIntensity(key, intensity);
}

IntensityOcTree::StaticMemberInitializer IntensityOcTree::intensityOcTreeMemberInit;
}// namespace octomap