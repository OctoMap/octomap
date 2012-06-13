#include <stdio.h>
#include <string>

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Utils.h>
#include "testing.h"
 
using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {

  if (argc != 2){
    std::cerr << "Error: you need to specify a testfile (.bt) as argument to read" << std::endl;
    return 1; // exit 1 means failure
  }

  string filename = string(argv[1]);

  string filenameOt = "test_io_file.ot";
  string filenameBtOut = "test_io_file.bt";

  // read reference tree from input file
  OcTree tree (0.1);
  EXPECT_TRUE (tree.readBinary(filename));

  // write again to bt, read & compare
  EXPECT_TRUE(tree.writeBinary(filenameBtOut));
  OcTree readTreeBt(0.1);
  EXPECT_TRUE(readTreeBt.readBinary(filenameBtOut));
  EXPECT_TRUE(tree == readTreeBt);

  // now write to .ot, read & compare
  EXPECT_TRUE(tree.write(filenameOt));

  AbstractOcTree* readTreeAbstract = AbstractOcTree::read(filenameOt);
  EXPECT_TRUE(readTreeAbstract);

  OcTree* readTreeOt = dynamic_cast<OcTree*>(readTreeAbstract);
  EXPECT_TRUE(readTreeOt);
  EXPECT_TRUE(tree == *readTreeOt);

  // sanity test for "==": flip one node, compare again
  point3d coord(0.1, 0.1, 0.1);
  OcTreeNode* node = readTreeOt->search(coord);
  if (node && readTreeOt->isNodeOccupied(node))
    readTreeOt->updateNode(coord, false);
  else
    readTreeOt->updateNode(coord, true);

  EXPECT_FALSE(tree == *readTreeOt);

  // simple test for tree headers (color)
  double res = 0.02;
  std::string filenameColor = "test_io_color_file.ot";
  ColorOcTree colorTree(res);
  EXPECT_EQ(colorTree.getTreeType(), "ColorOcTree");
  ColorOcTreeNode* colorNode = colorTree.updateNode(point3d(0.0, 0.0, 0.0), true);
  ColorOcTreeNode::Color color_red(255, 0, 0);
  colorNode->setColor(color_red);
  colorTree.setNodeColor(0.0, 0.0, 0.0, 255, 0, 0);
  colorTree.updateNode(point3d(0.1, 0.1, 0.1), true);
  colorTree.setNodeColor(0.1, 0.1, 0.1, 0, 0, 255);

  EXPECT_TRUE(colorTree.write(filenameColor));
  readTreeAbstract = AbstractOcTree::read(filenameColor);
  EXPECT_TRUE(readTreeAbstract);
  EXPECT_EQ(colorTree.getTreeType(),  readTreeAbstract->getTreeType());
  ColorOcTree* readColorTree = dynamic_cast<ColorOcTree*>(readTreeAbstract);
  EXPECT_TRUE(readColorTree);
  EXPECT_TRUE(colorTree == *readColorTree);
  colorNode = colorTree.search(0.0, 0.0, 0.0);
  EXPECT_TRUE(colorNode);
  EXPECT_EQ(colorNode->getColor(), color_red);



  std::cerr << "Test successful.\n";
  return 0;
}
