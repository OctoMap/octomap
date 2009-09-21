#ifndef MAPPING3D_SCANGRAPH_H
#define MAPPING3D_SCANGRAPH_H

// ==================================================
// octomap
// Kai M. Wurm <wurm@uni-freiburg.de>
// ==================================================

#include <string>
#include <math.h>

#include "Pointcloud.h"
#include "octomap_types.h"

#define SCANGRAPH_HEIGHT_OFFSET 0.55

namespace octomap {

  class ScanGraph;


  class ScanNode {

   public:

    ScanNode (Pointcloud* _scan, octomath::Pose6D _pose, unsigned int _id) : scan(_scan), pose(_pose), id(_id) {}
    ScanNode () {}

    bool operator == (const ScanNode& other) {
      return (id == other.id);
    }

    std::ostream& writeBinary(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);

    std::ostream& writePoseASCII(std::ostream &s) const;
    std::istream& readPoseASCII(std::istream &s);

    Pointcloud* scan;
    octomath::Pose6D pose;
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


  class ScanGraph {

   public:

    ScanGraph() {};
    ~ScanGraph();

    void clear();

    ScanNode* addNode(Pointcloud* scan, octomath::Pose6D pose);
    ScanEdge* addEdge(ScanNode* first, ScanNode* second, octomath::Pose6D constraint);
    ScanEdge* addEdge(uint first_id, uint second_id);

    // will return NULL if node was not found
    ScanNode* getNodeByID(unsigned int id);

    bool edgeExists(uint first_id, uint second_id);

    // connect previously added point to the one before that
    void connectPrevious();

    // find and update loops according to a given distance threshold
    void updateLoopsDist(double distthres, double angle_thres=M_PI*.75);
    static double estimateOverlap(octomath::Pose6D p, octomath::Pose6D q, double distthres, double angle_thres);


    std::vector<unsigned int> getNeighborIDs(unsigned int id);
    std::vector<ScanEdge*> getOutEdges(ScanNode* node);
    // warning: constraints are reversed
    std::vector<ScanEdge*> getInEdges(ScanNode* node);

    void exportDot(std::string filename);

    // Transform every scan according to its pose
    void transformScans();

    // cut graph (all containing Pointclouds) to given BBX in global coords
    void crop(point3d lowerBound, point3d upperBound);

    // cut Pointclouds to given BBX in local coords
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


    unsigned int loops_size() { return loops.size(); }
    edge_iterator loops_begin() { return loops.begin(); }
    edge_iterator loops_end()   { return loops.end(); }
    bool isLoop(ScanEdge* e);

    std::ostream& writeBinary(std::ostream &s) const;
    std::istream& readBinary(std::ifstream &s);
    void writeBinary(std::string filename) const;
    void readBinary(std::string filename);


    std::ostream& writeEdgesASCII(std::ostream &s) const;
    std::istream& readEdgesASCII(std::istream &s);

    std::ostream& writeNodePosesASCII(std::ostream &s) const;
    std::istream& readNodePosesASCII(std::istream &s);

   protected:

    std::vector<std::pair<uint, uint> > findLoopsDist(ScanNode* node, double distthres, double angle_thres);

    std::vector<ScanNode*> nodes;
    std::vector<ScanEdge*> edges;

    // loops is a subset of edges
    std::vector<ScanEdge*> loops;
  };

}


#endif
