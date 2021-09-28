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

#ifndef OCTOMAP_SCANGRAPH_H
#define OCTOMAP_SCANGRAPH_H


#include <string>
#include <math.h>

#include "Pointcloud.h"
#include "octomap_types.h"

namespace octomap {

  class ScanGraph;


  /**
   * A 3D scan as Pointcloud, performed from a Pose6D.
   */
  class ScanNode {

   public:

    ScanNode (Pointcloud* _scan, pose6d _pose, unsigned int _id)
      : scan(_scan), pose(_pose), id(_id) {}
    ScanNode ()
      : scan(NULL) {}

    ~ScanNode();

    bool operator == (const ScanNode& other) {
      return (id == other.id);
    }

    std::ostream& writeBinary(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);

    std::ostream& writePoseASCII(std::ostream &s) const;
    std::istream& readPoseASCII(std::istream &s);

    Pointcloud* scan;
    pose6d pose; ///< 6D pose from which the scan was performed
    unsigned int id;

  };

  /**
   * A connection between two \ref ScanNode "ScanNodes"
   */
  class ScanEdge {

   public:

    ScanEdge(ScanNode* _first, ScanNode* _second, pose6d _constraint)
      : first(_first), second(_second), constraint(_constraint), weight(1.0) { }
    ScanEdge() {}

    bool operator == (const ScanEdge& other) {
      return ( (*first == *(other.first) ) && ( *second == *(other.second) ) );
    }

    std::ostream& writeBinary(std::ostream &s) const;
    // a graph has to be given to recover ScanNode pointers
    std::istream& readBinary(std::istream &s, ScanGraph& graph);

    std::ostream& writeASCII(std::ostream &s) const;
    std::istream& readASCII(std::istream &s, ScanGraph& graph);

    ScanNode* first;
    ScanNode* second;

    pose6d constraint;
    double weight;
  };


  /**
   * A ScanGraph is a collection of ScanNodes, connected by ScanEdges.
   * Each ScanNode contains a 3D scan performed from a pose.
   *
   */
  class ScanGraph {

   public:

    ScanGraph() {};
    ~ScanGraph();

    /// Clears all nodes and edges, and will delete the corresponding objects
    void clear();

    /**
     * Creates a new ScanNode in the graph from a Pointcloud.
     *
     * @param scan Pointer to a pointcloud to be added to the ScanGraph.
     *        ScanGraph will delete the object when it's no longer needed, don't delete it yourself.
     * @param pose 6D pose of the origin of the Pointcloud
     * @return Pointer to the new node
     */
    ScanNode* addNode(Pointcloud* scan, pose6d pose);

    /**
     * Creates an edge between two ScanNodes.
     * ScanGraph will delete the object when it's no longer needed, don't delete it yourself.
     *
     * @param first ScanNode
     * @param second ScanNode
     * @param constraint 6D transform between the two nodes
     * @return
     */
    ScanEdge* addEdge(ScanNode* first, ScanNode* second, pose6d constraint);

    ScanEdge* addEdge(unsigned int first_id, unsigned int second_id);

    /// will return NULL if node was not found
    ScanNode* getNodeByID(unsigned int id);

    /// \return true when an edge between first_id and second_id exists
    bool edgeExists(unsigned int first_id, unsigned int second_id);

    /// Connect previously added ScanNode to the one before that
    void connectPrevious();

    std::vector<unsigned int> getNeighborIDs(unsigned int id);
    std::vector<ScanEdge*> getOutEdges(ScanNode* node);
    // warning: constraints are reversed
    std::vector<ScanEdge*> getInEdges(ScanNode* node);

    void exportDot(std::string filename);

    /// Transform every scan according to its pose
    void transformScans();

    /// Cut graph (all containing Pointclouds) to given BBX in global coords
    void crop(point3d lowerBound, point3d upperBound);

    /// Cut Pointclouds to given BBX in local coords
    void cropEachScan(point3d lowerBound, point3d upperBound);


    typedef std::vector<ScanNode*>::iterator iterator;
    typedef std::vector<ScanNode*>::const_iterator const_iterator;
    iterator begin() { return nodes.begin(); }
    iterator end()   { return nodes.end(); }
    const_iterator begin() const { return nodes.begin(); }
    const_iterator end() const { return nodes.end(); }

    size_t size() const { return nodes.size(); }
    size_t getNumPoints(unsigned int max_id = -1) const;

    typedef std::vector<ScanEdge*>::iterator edge_iterator;
    typedef std::vector<ScanEdge*>::const_iterator const_edge_iterator;
    edge_iterator edges_begin() { return edges.begin(); }
    edge_iterator edges_end()  { return edges.end(); }
    const_edge_iterator edges_begin() const { return edges.begin(); }
    const_edge_iterator edges_end() const  { return edges.end(); }


    std::ostream& writeBinary(std::ostream &s) const;
    std::istream& readBinary(std::ifstream &s);
    bool writeBinary(const std::string& filename) const;
    bool readBinary(const std::string& filename);


    std::ostream& writeEdgesASCII(std::ostream &s) const;
    std::istream& readEdgesASCII(std::istream &s);

    std::ostream& writeNodePosesASCII(std::ostream &s) const;
    std::istream& readNodePosesASCII(std::istream &s);

    /**
     * Reads in a ScanGraph from a "plain" ASCII file of the form
     * NODE x y z R P Y
     * x y z
     * x y z
     * x y z
     * NODE x y z R P Y
     * x y z
     *
     * Lines starting with the NODE keyword contain the 6D pose of a scan node,
     * all 3D point following until the next NODE keyword (or end of file) are
     * inserted into that scan node as pointcloud in its local coordinate frame
     *
     * @param s input stream to read from
     * @return read stream
     */
    std::istream& readPlainASCII(std::istream& s);
    void readPlainASCII(const std::string& filename);

   protected:

    std::vector<ScanNode*> nodes;
    std::vector<ScanEdge*> edges;
  };

}


#endif
