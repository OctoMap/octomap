
#include <octomap/octomap.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {
    double res = 0.01;
    OcTree tree(res);

    point3d singlePt(-0.05, -0.02, 1.0);
    OcTreeKey singleKey;
    tree.coordToKeyChecked(singlePt, singleKey);
    OcTreeNode* singleNode = tree.updateNode(singleKey, true);
    EXPECT_TRUE(singleNode);
    EXPECT_EQ(singleNode, tree.search(singlePt));
    // check all neighbors, should not exist:
    OcTreeKey key;

    for (key[2] = singleKey[2] - 1; key[2] <= singleKey[2] + 1; ++key[2]){
      for (key[1] = singleKey[1] - 1; key[1] <= singleKey[1] + 1; ++key[1]){
        for (key[0] = singleKey[0] - 1; key[0] <= singleKey[0] + 1; ++key[0]){
          if (key != singleKey){
            OcTreeNode* node = tree.search(key);
            EXPECT_FALSE(node);
          } else {
            OcTreeNode* node = tree.search(key);
            EXPECT_TRUE(node);
            EXPECT_EQ(singleNode, node);
          }
        }
      }
    }
    // pruning should do nothing:
    tree.prune();
    for (key[2] = singleKey[2] - 1; key[2] <= singleKey[2] + 1; ++key[2]){
      for (key[1] = singleKey[1] - 1; key[1] <= singleKey[1] + 1; ++key[1]){
        for (key[0] = singleKey[0] - 1; key[0] <= singleKey[0] + 1; ++key[0]){
          if (key != singleKey){
            OcTreeNode* node = tree.search(key);
            EXPECT_FALSE(node);
          } else {
            OcTreeNode* node = tree.search(key);
            EXPECT_TRUE(node);
            EXPECT_EQ(singleNode, node);
          }
        }
      }
    }
    // node + 1 branch of depth 16
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(tree.size(), 17);
    // create diagonal neighbor in same parent node
    OcTreeKey singleKey2 = singleKey;
    singleKey2[0] +=1;
    singleKey2[1] +=1;
    singleKey2[2] +=1;
    EXPECT_TRUE(tree.updateNode(singleKey2, true));
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(tree.size(), 18); // one more leaf at lowest level
    // pruning should do nothing:
    tree.prune();
    for (key[2] = singleKey[2] - 1; key[2] <= singleKey[2] + 1; ++key[2]){
      for (key[1] = singleKey[1] - 1; key[1] <= singleKey[1] + 1; ++key[1]){
        for (key[0] = singleKey[0] - 1; key[0] <= singleKey[0] + 1; ++key[0]){
          if (key != singleKey && key != singleKey2){
            OcTreeNode* node = tree.search(key);
            EXPECT_FALSE(node);
          }
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(tree.size(), 18);

    // now test larger volume pruning:
    for (float x=0.005; x <= 0.32; x+=res){
      for (float y=0.005; y <= 0.32; y+=res){
        for (float z=0.005; z <= 0.32; z+=res){
          OcTreeNode* node = tree.updateNode(point3d(x,y,z), true);
          EXPECT_TRUE(node);
          EXPECT_TRUE(tree.isNodeOccupied(node));
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(37477, tree.size());
    tree.prune();
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(29, tree.size());
    tree.expand();
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(37477, tree.size());
    tree.prune();
    // test expansion:
    for (float x=0.005; x <= 0.32; x+=res){
      for (float y=0.005; y <= 0.32; y+=res){
        for (float z=0.005; z <= 0.32; z+=res){
          OcTreeNode* node = tree.search(point3d(x,y,z));
          EXPECT_TRUE(node);
          EXPECT_TRUE(tree.isNodeOccupied(node));
        }
      }
    }

    tree.coordToKeyChecked(point3d(0.1, 0.1, 0.1), singleKey);

    EXPECT_TRUE(tree.updateNode(singleKey, true));

    for (float x=0.005; x <= 0.32; x+=res){
      for (float y=0.005; y <= 0.32; y+=res){
        for (float z=0.005; z <= 0.32; z+=res){
          OcTreeNode* node = tree.search(point3d(x,y,z));
          EXPECT_TRUE(node);
          EXPECT_TRUE(tree.isNodeOccupied(node));
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(69, tree.size());




    //tree.write("pruning_test_out.ot");
    std::cerr << "Test successful.\n";
    return 0;

}
