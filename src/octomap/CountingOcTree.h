#ifndef MAPPING3D_COUNTING_OCTREE_HH
#define MAPPING3D_COUNTING_OCTREE_HH

// =====================================================
// octomap
// -----------------------------------------------------
// Kai M. Wurm <wurm@uni-freiburg.de>
// Armin Hornung <hornunga@informatik.uni-freiburg.de>
// =====================================================


#include "AbstractOcTree.h"

namespace octomap {

  /*!
   * An OcTreeNode which stores an internal counter per node / cell
   * count is recursive, parent nodes have the summed count of their
   * children.
   */
  class CountingOcTreeNode {

  public:

    CountingOcTreeNode();
    ~CountingOcTreeNode();

    CountingOcTreeNode* getChild(unsigned int i);
    void setChild(unsigned int i, CountingOcTreeNode* child);
    bool childExists(unsigned int i) const;
    bool hasChildren() const;

    unsigned int getCount() const { return count; }
    void increaseCount() { count++; }

  protected:
    CountingOcTreeNode* itsChildren[8];
    unsigned int count;
  };


  /*!
   * OcTree implementation storing the hit counts for single cells
   */
  class CountingOcTree : public AbstractOcTree <CountingOcTreeNode> {

  public:
    
    CountingOcTree(double resolution = 0.1);
    ~CountingOcTree();
    
    CountingOcTreeNode* updateNode(const point3d& value);

  protected:

    void traverseNode(CountingOcTreeNode* traversedNode);

  };


}


#endif
