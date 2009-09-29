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

#ifndef OCTOMAP_SCANGRAPH_H
#define OCTOMAP_SCANGRAPH_H


#include <string>
#include <math.h>

#include "Pointcloud.h"
#include "octomap_types.h"

#define SCANGRAPH_HEIGHT_OFFSET 0.55

namespace octomap {

  class ScanGraph;


  /**
   * A 3D scan as Pointcloud, performed from a Pose6D.
   */
  class ScanNode {

   public:

    ScanNode (Pointcloud* _scan, octomath::Pose6D _pose, unsigned int _id)
      : scan(_scan), pose(_pose), id(_id) {}
    ScanNode () {}

    bool operator == (const ScanNode& other) {
      return (id == other.id);
    }

    std::ostream& writeBinary(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);

    std::ostream& writePoseASCII(std::ostream &s) const;
    std::istream& readPoseASCII(std::istream &s);

    Pointcloud* scan;
    octomath::Pose6D pose; ///< 6D pose from which the scan was performed
    unsigned int id;

  };

  class ScanEdge {

   public:

    ScanEdge(ScanNode* _first, ScanNode* _second, octomath::Pose6D _constraint)
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

    octomath::Pose6D constraint;
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

    /// Clears all nodes and edges
    void clear();

    ScanNode* addNode(Pointcloud* scan, octomath::Pose6D pose);
    ScanEdge* addEdge(ScanNode* first, ScanNode* second, octomath::Pose6D constraint);
    ScanEdge* addEdge(unsigned int first_id, unsigned int second_id);

    /// will return NULL if node was not found
    ScanNode* getNodeByID(unsigned int id);

    /// \return true when an edge between first_id and second_id exists
    bool edgeExists(unsigned int first_id, unsigned int second_id);

    /// Connect previously added point to the one before that
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

    unsigned int size() const { return nodes.size(); }

    typedef std::vector<ScanEdge*>::iterator edge_iterator;
    typedef std::vector<ScanEdge*>::const_iterator const_edge_iterator;
    edge_iterator edges_begin() { return edges.begin(); }
    edge_iterator edges_end()  { return edges.end(); }
    const_edge_iterator edges_begin() const { return edges.begin(); }
    const_edge_iterator edges_end() const  { return edges.end(); }


    std::ostream& writeBinary(std::ostream &s) const;
    std::istream& readBinary(std::ifstream &s);
    void writeBinary(std::string filename) const;
    void readBinary(std::string filename);


    std::ostream& writeEdgesASCII(std::ostream &s) const;
    std::istream& readEdgesASCII(std::istream &s);

    std::ostream& writeNodePosesASCII(std::ostream &s) const;
    std::istream& readNodePosesASCII(std::istream &s);

   protected:

    std::vector<ScanNode*> nodes;
    std::vector<ScanEdge*> edges;
  };

}


#endif
