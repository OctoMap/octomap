#include <iostream>
#include <string>

#include <octomap/IntensityOcTree.h>
#include "testing.h"

using namespace std;
using namespace octomap;

int main(int /*argc*/, char** /*argv*/) {
  double res = 0.1;
  IntensityOcTree tree(res);
  EXPECT_EQ(tree.getTreeType(), "IntensityOcTree");

  tree.setProbHit(0.8);
  point3d p(0.0f, 0.0f, 0.0f);
  IntensityOcTreeNode* node = tree.updateNode(p, true);
  EXPECT_TRUE(node);
  double node_prob = node->getOccupancy();
  EXPECT_NEAR(node_prob, 0.8, 1e-5);
  EXPECT_FALSE(node->isIntensitySet());

  EXPECT_TRUE(tree.integrateNodeIntensity(p.x(), p.y(), p.z(), 10.0));
  node = tree.search(p);
  EXPECT_TRUE(node);
  EXPECT_FLOAT_EQ(node->getIntensity(), 10.0);

  EXPECT_TRUE(tree.integrateNodeIntensity(p.x(), p.y(), p.z(), 20.0));
  node = tree.search(p);
  EXPECT_TRUE(node);
  double expected_intensity = 10.0 * node_prob + 20.0 * (1.0 - node_prob);
  EXPECT_NEAR(node->getIntensity(), expected_intensity, 1e-5);

  EXPECT_FALSE(tree.integrateNodeIntensity(10.0f, 10.0f, 10.0f, 5.0));

  IntensityOcTreeNode* parent = tree.search(p, tree.getTreeDepth() - 1);
  EXPECT_TRUE(parent);

  double intensity_sum = 0.0;
  for (unsigned int i = 0; i < 8; ++i) {
    if (!tree.nodeChildExists(parent, i)) {
      tree.createNodeChild(parent, i);
    }
    IntensityOcTreeNode* child = tree.getNodeChild(parent, i);
    child->setLogOdds(0.0f);
    double intensity = static_cast<double>(i + 1);
    child->setIntensity(intensity);
    intensity_sum += intensity;
  }

  tree.updateInnerOccupancy();
  EXPECT_NEAR(parent->getIntensity(), intensity_sum / 8.0, 1e-5);

  // Pruning compares full child data, so align intensities first.
  const double uniform_intensity = 2.0;
  for (unsigned int i = 0; i < 8; ++i) {
    IntensityOcTreeNode* child = tree.getNodeChild(parent, i);
    child->setIntensity(uniform_intensity);
  }

  EXPECT_TRUE(tree.pruneNode(parent));
  EXPECT_FALSE(tree.nodeHasChildren(parent));
  EXPECT_NEAR(parent->getIntensity(), uniform_intensity, 1e-5);

  std::string filename("simple_intensity_tree.ot");
  EXPECT_TRUE(tree.write(filename));

  AbstractOcTree* read_tree = AbstractOcTree::read(filename);
  EXPECT_TRUE(read_tree);
  EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
  EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
  EXPECT_EQ(read_tree->size(), tree.size());

  IntensityOcTree* read_intensity_tree = dynamic_cast<IntensityOcTree*>(read_tree);
  EXPECT_TRUE(read_intensity_tree);
  EXPECT_TRUE(tree == *read_intensity_tree);

  delete read_tree;
  read_tree = NULL;

  std::cerr << "Test successful.\n";
  return 0;
}
