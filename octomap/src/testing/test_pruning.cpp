
#include <octomap/octomap.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {
    float res = 0.01f;
    OcTree tree(res);
    
    EXPECT_EQ(tree.size(), 0);
    tree.prune();
    EXPECT_EQ(tree.size(), 0);

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
    
    // test deletion / pruning of single nodes
    {
      std::cout << "\nCreating / deleting nodes\n===============================\n";
      size_t initialSize = tree.size();
      EXPECT_EQ(initialSize, tree.calcNumNodes());
      EXPECT_EQ(initialSize, 67);
            
      point3d newCoord(-2.0, -2.0, -2.0);
      OcTreeNode* newNode = tree.updateNode(newCoord, true);
      EXPECT_TRUE(newNode != NULL);
      
      size_t insertedSize = tree.size();
      std::cout << "Size after one insertion: " << insertedSize << std::endl;
      EXPECT_EQ(insertedSize, tree.calcNumNodes());
      EXPECT_EQ(insertedSize, 83);
      
      // find parent of newly inserted node:
      OcTreeNode* parentNode = tree.search(newCoord, tree.getTreeDepth() -1);
      EXPECT_TRUE(parentNode);
      EXPECT_TRUE(tree.nodeHasChildren(parentNode));
      
      // only one child exists:
      for (size_t i = 0; i < 7; ++i){
        EXPECT_FALSE(tree.nodeChildExists(parentNode, i));
      }
      EXPECT_TRUE(tree.nodeChildExists(parentNode, 7));
      
      // create another new node manually:
      OcTreeNode* newNodeCreated = tree.createNodeChild(parentNode, 0);
      EXPECT_TRUE(newNodeCreated != NULL);
      EXPECT_TRUE(tree.nodeChildExists(parentNode, 0));
      const float value = 0.123f;
      newNodeCreated->setValue(value);
      tree.write("pruning_test_edited.ot");
      
      EXPECT_EQ(tree.size(), tree.calcNumNodes());
      EXPECT_EQ(tree.size(), insertedSize+1);
      tree.prune();
      EXPECT_EQ(tree.calcNumNodes(), insertedSize+1);
      
      tree.deleteNodeChild(parentNode, 0);
      tree.deleteNodeChild(parentNode, 7);
      
      EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
      EXPECT_EQ(tree.size(), insertedSize-1);
      
      tree.prune();
      EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
      EXPECT_EQ(tree.size(), insertedSize-1);
                
      tree.expandNode(parentNode);
      EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
      EXPECT_EQ(tree.size(), insertedSize+7);
      
      
      EXPECT_TRUE(tree.pruneNode(parentNode));
      EXPECT_EQ(tree.size(), tree.calcNumNodes()); 
      EXPECT_EQ(tree.size(), insertedSize-1);    
      
      
    }
    
    tree.write("pruning_test_out.ot");
    
    {
      std::cout << "\nClearing tree / recursive delete\n===============================\n";
      
      OcTree emptyTree(0.1234);
      EXPECT_EQ(emptyTree.size(), 0);
      emptyTree.clear();
      EXPECT_EQ(emptyTree.size(), emptyTree.calcNumNodes());
      EXPECT_EQ(emptyTree.size(), 0);
    
      tree.clear();
      EXPECT_EQ(tree.size(), 0);
      EXPECT_EQ(tree.size(), tree.calcNumNodes());
      
      tree.prune();
      EXPECT_EQ(tree.size(), 0);      
    }

    tree.write("pruning_test_out.ot");
    std::cerr << "Test successful.\n";
    return 0;

}
