#include <stdio.h>
#include <string>

#include <octomap/octomap.h>
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


  std::cerr << "Test successful.\n";
  return 0;
}
