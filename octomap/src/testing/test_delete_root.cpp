// Deleting the last occupied node must free the root node, see #307.
// Counts live nodes through a node type derived from OcTreeNode.
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

#include <string>

#include "testing.h"

using namespace octomap;

class CountedNode : public OcTreeNode {
public:
  CountedNode() { ++alive; }
  CountedNode(const CountedNode& rhs) : OcTreeNode(rhs) { ++alive; }
  ~CountedNode() { --alive; }
  static long alive;
};
long CountedNode::alive = 0;

class CountedTree : public OccupancyOcTreeBase<CountedNode> {
public:
  explicit CountedTree(double res) : OccupancyOcTreeBase<CountedNode>(res) {}
  CountedTree* create() const { return new CountedTree(resolution); }
  std::string getTreeType() const { return "CountedTree"; }
};

int main() {
  {
    CountedTree tree(0.1);
    const point3d point(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) {
      tree.updateNode(point, true);
      EXPECT_TRUE(tree.search(point) != NULL);
      tree.deleteNode(point);
      EXPECT_TRUE(tree.getRoot() == NULL);
      EXPECT_EQ(CountedNode::alive, 0);
    }
  }
  EXPECT_EQ(CountedNode::alive, 0);
  return 0;
}
