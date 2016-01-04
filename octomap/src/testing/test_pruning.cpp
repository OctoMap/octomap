
#include <octomap/octomap.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {
    float res = 0.01f;
    OcTree tree(res);

    point3d singlePt(-0.05f, -0.02f, 1.0f);
    OcTreeKey singleKey;
    tree.coordToKeyChecked(singlePt, singleKey);
    OcTreeNode* singleNode = tree.updateNode(singleKey, true);
    EXPECT_TRUE(singleNode);
    EXPECT_EQ(singleNode, tree.search(singlePt));

    OcTreeKey key;
    // check all neighbors, none should exist:
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
    OcTreeNode* singleNode2 = tree.updateNode(singleKey2, true);
    EXPECT_TRUE(singleNode2);

    for (key[2] = singleKey[2] - 1; key[2] <= singleKey[2] + 1; ++key[2]){
      for (key[1] = singleKey[1] - 1; key[1] <= singleKey[1] + 1; ++key[1]){
        for (key[0] = singleKey[0] - 1; key[0] <= singleKey[0] + 1; ++key[0]){
          if (key == singleKey){
            OcTreeNode* node = tree.search(key);
            EXPECT_TRUE(node);
            EXPECT_EQ(singleNode, node);
          } else if (key == singleKey2){
            OcTreeNode* node = tree.search(key);
            EXPECT_TRUE(node);
            EXPECT_EQ(singleNode2, node);
          } else{
            OcTreeNode* node = tree.search(key);
            EXPECT_FALSE(node);
          }
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(tree.size(), 18); // one more leaf at lowest level
    // pruning should do nothing:
    tree.prune();
    for (key[2] = singleKey[2] - 1; key[2] <= singleKey[2] + 1; ++key[2]){
      for (key[1] = singleKey[1] - 1; key[1] <= singleKey[1] + 1; ++key[1]){
        for (key[0] = singleKey[0] - 1; key[0] <= singleKey[0] + 1; ++key[0]){
          if (key == singleKey){
            OcTreeNode* node = tree.search(key);
            EXPECT_TRUE(node);
            EXPECT_EQ(singleNode, node);
          } else if (key == singleKey2){
            OcTreeNode* node = tree.search(key);
            EXPECT_TRUE(node);
            EXPECT_EQ(singleNode2, node);
          } else{
            OcTreeNode* node = tree.search(key);
            EXPECT_FALSE(node);
          }
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(tree.size(), 18);

    //tree.write("pruning_test_out0.ot");

    // fill the complete octant, should auto-prune
    tree.updateNode(OcTreeKey(singleKey[0]+1, singleKey[1]+0, singleKey[2]+0), true);
    tree.updateNode(OcTreeKey(singleKey[0]+1, singleKey[1]+1, singleKey[2]+0), true);
    tree.updateNode(OcTreeKey(singleKey[0]+0, singleKey[1]+1, singleKey[2]+0), true);
    tree.updateNode(OcTreeKey(singleKey[0]+0, singleKey[1]+0, singleKey[2]+1), true);
    tree.updateNode(OcTreeKey(singleKey[0]+1, singleKey[1]+0, singleKey[2]+1), true);
    EXPECT_EQ(tree.size(), 23);
    // last node should trigger auto-pruning:
    OcTreeNode* prunedNode = tree.updateNode(OcTreeKey(singleKey[0]+0, singleKey[1]+1, singleKey[2]+1), true);
    EXPECT_EQ(tree.size(), 16);
    // all queries should now end up at same parent node:
    OcTreeNode* parentNode = tree.search(singleKey);
    OcTreeNode* parentNode2 = tree.search(singleKey2);
    EXPECT_EQ(parentNode, parentNode2);
    // test pointer returned by updateNode (pruned)
    EXPECT_EQ(prunedNode, parentNode);

    //tree.write("pruning_test_out1.ot");

    // now test larger volume pruning:
    for (float x=0.005f; x <= 0.32f; x+=res){
      for (float y=0.005f; y <= 0.32f; y+=res){
        for (float z=0.005f; z <= 0.32f; z+=res){
          OcTreeNode* node = tree.updateNode(point3d(x,y,z), true);
          EXPECT_TRUE(node);
          EXPECT_TRUE(tree.isNodeOccupied(node));
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(27, tree.size());
    // TODO: replace with test for lazy eval?
    tree.prune();
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(27, tree.size());
    tree.expand();
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(37483, tree.size());
    tree.prune();
    EXPECT_EQ(27, tree.size());
    // test expansion:
    for (float x=0.005f; x <= 0.32f; x+=res){
      for (float y=0.005f; y <= 0.32f; y+=res){
        for (float z=0.005f; z <= 0.32f; z+=res){
          OcTreeNode* node = tree.search(point3d(x,y,z));
          EXPECT_TRUE(node);
          EXPECT_TRUE(tree.isNodeOccupied(node));
        }
      }
    }

    tree.coordToKeyChecked(point3d(0.1f, 0.1f, 0.1f), singleKey);

    EXPECT_TRUE(tree.updateNode(singleKey, true));

    for (float x=0.005f; x <= 0.32f; x+=res){
      for (float y=0.005f; y <= 0.32f; y+=res){
        for (float z=0.005f; z <= 0.32f; z+=res){
          OcTreeNode* node = tree.search(point3d(x,y,z));
          EXPECT_TRUE(node);
          EXPECT_TRUE(tree.isNodeOccupied(node));
        }
      }
    }
    EXPECT_EQ(tree.calcNumNodes(), tree.size());
    EXPECT_EQ(67, tree.size());

    //tree.write("pruning_test_out.ot");
    std::cerr << "Test successful.\n";
    return 0;

}
