
#include <octomap/octomap.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;

int main(int /*argc*/, char** /*argv*/) {
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
    OcTreeNode* parentNode1 = tree.search(singleKey);
    OcTreeNode* parentNode2 = tree.search(singleKey2);
    EXPECT_EQ(parentNode1, parentNode2);
    // test pointer returned by updateNode (pruned)
    EXPECT_EQ(prunedNode, parentNode1);

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
    
    //tree.write("pruning_test_out.ot"); for debugging
    
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

    {

    // ------------------------------------------------------------
    std::cout << "\nCreation and pruning of 8 siblings\n===============================\n";
    OcTree pruningTree(0.2f);
    // Define 8 neighboring points in different octants around origin
    // These points occupy all 8 octants of their parent cell at depth 1
    point3d pt_000( 0.1f, 0.1f, 0.1f);  // octant 0
    point3d pt_001( 0.1f, 0.1f, 0.3f);  // octant 1
    point3d pt_010( 0.1f, 0.3f, 0.1f);  // octant 2
    point3d pt_011( 0.1f, 0.3f, 0.3f);  // octant 3
    point3d pt_100( 0.3f, 0.1f, 0.1f);  // octant 4
    point3d pt_101( 0.3f, 0.1f, 0.3f);  // octant 5
    point3d pt_110( 0.3f, 0.3f, 0.1f);  // octant 6
    point3d pt_111( 0.3f, 0.3f, 0.3f);  // octant 7
    
    // Insert all 8 nodes with occupied state (true)
    OcTreeKey key_000, key_001, key_010, key_011, key_100, key_101, key_110, key_111;
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_000, key_000));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_001, key_001));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_010, key_010));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_011, key_011));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_100, key_100));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_101, key_101));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_110, key_110));
    EXPECT_TRUE(pruningTree.coordToKeyChecked(pt_111, key_111));
    
    pruningTree.updateNode(key_000, true);
    pruningTree.updateNode(key_001, true);
    pruningTree.updateNode(key_010, true);
    pruningTree.updateNode(key_011, true);
    pruningTree.updateNode(key_100, true);
    pruningTree.updateNode(key_101, true);
    pruningTree.updateNode(key_110, true);
    //pruningTree.write("octree_pruning_test1.ot"); // DEBUGGING

    auto checkPruneExpandConstant = [&](OcTree& t, size_t expectedNumNodes, size_t expectedNumLeafs, bool toMaxLikelihood=true){
      EXPECT_EQ (t.calcNumNodes(), t.size()); // check for size inconsistencies
      EXPECT_EQ (t.size(), expectedNumNodes); 
      EXPECT_EQ (t.getNumLeafNodes(), expectedNumLeafs);
      
      t.prune();
      EXPECT_EQ (t.size(), expectedNumNodes); 
      EXPECT_EQ (t.getNumLeafNodes(), expectedNumLeafs);
      EXPECT_EQ (t.calcNumNodes(), t.size());

      if (toMaxLikelihood) {
        t.toMaxLikelihood();
      }
      
      EXPECT_EQ (t.size(), expectedNumNodes); 
      EXPECT_EQ (t.getNumLeafNodes(), expectedNumLeafs);
      t.prune();
      EXPECT_EQ (t.calcNumNodes(), t.size());
      EXPECT_EQ (t.size(), expectedNumNodes); 
      EXPECT_EQ (t.getNumLeafNodes(), expectedNumLeafs);
      EXPECT_EQ (t.calcNumNodes(), t.size());

      t.expand();
      EXPECT_EQ (t.calcNumNodes(), t.size());
      EXPECT_EQ (t.size(), expectedNumNodes); 
      EXPECT_EQ (t.getNumLeafNodes(), expectedNumLeafs);
      EXPECT_EQ (t.calcNumNodes(), t.size());
    };


        
    size_t expectedNumNodes = 16+7;
    size_t expectedNumLeafs = 7;
    // seven separate nodes added, all separate leafs
    EXPECT_EQ (pruningTree.size(), expectedNumNodes); 
    EXPECT_EQ (pruningTree.getNumLeafNodes(), expectedNumLeafs);
    checkPruneExpandConstant(pruningTree, expectedNumNodes, expectedNumLeafs, false);

    pruningTree.updateNode(key_111, true); // last insertion will cause pruning by one level
    //pruningTree.write("octree_structure_pruning_test2.ot"); // DEBUGGING
    const size_t expectedNumNodesPruned = 16;
    const size_t expectedNumLeafsPruned = 1;
    EXPECT_EQ (pruningTree.size(), expectedNumNodesPruned); 
    EXPECT_EQ (pruningTree.calcNumNodes(), pruningTree.size());
    EXPECT_EQ (pruningTree.getNumLeafNodes(), expectedNumLeafsPruned);        
    
    //pruning should not have an additional effect
    pruningTree.prune();
    EXPECT_EQ (pruningTree.size(), expectedNumNodesPruned); 
    EXPECT_EQ (pruningTree.calcNumNodes(), pruningTree.size());
    EXPECT_EQ (pruningTree.getNumLeafNodes(), expectedNumLeafsPruned); 
    
    // expanding should recreate all 8 nodes again
    pruningTree.expand();
    const size_t expectedNumNodesExp = 16+8;
    const size_t expectedNumLeafsExp = 8;
    EXPECT_EQ (pruningTree.size(), expectedNumNodesExp);
    EXPECT_EQ (pruningTree.calcNumNodes(), pruningTree.size());
    EXPECT_EQ (pruningTree.getNumLeafNodes(), expectedNumLeafsExp);

    pruningTree.prune(); // back to pruned node
    EXPECT_EQ (pruningTree.size(), expectedNumNodesPruned); 
    EXPECT_EQ (pruningTree.calcNumNodes(), pruningTree.size());
    EXPECT_EQ (pruningTree.getNumLeafNodes(), expectedNumLeafsPruned); 

    // updating one of the child node to completely "free" should expand the tree again, and it is no longer collapsible
    float logOddsChange = -1.0*pruningTree.getProbHitLog()+pruningTree.getProbMissLog(); // enough to go from occupied to free
    pruningTree.updateNode(key_101, logOddsChange); // set one node to free
    //pruningTree.write("octree_structure_pruning_test2.ot"); // DEBUGGING
    
    EXPECT_EQ (pruningTree.size(), expectedNumNodesExp);
    EXPECT_EQ (pruningTree.calcNumNodes(), pruningTree.size());
    EXPECT_EQ (pruningTree.getNumLeafNodes(), expectedNumLeafsExp);
    checkPruneExpandConstant(pruningTree, expectedNumNodesExp, expectedNumLeafsExp); 



    }

    {
      std::cout << "\nPruning fully filled octree\n===============================\n";
      // test pruning of a fully filled octree to single root node
      // manually create nodes at level 2, filling all space as single leafs nodes will exhaust memory
      const float resolution = 0.2f; 
      OcTree fullTree(resolution);
      
      // insert one node to initialize tree
      point3d pt1(0.1f, 0.1f, 0.1f);
      OcTreeKey key1;
      EXPECT_TRUE(fullTree.search(pt1) == NULL);
      EXPECT_TRUE(fullTree.coordToKeyChecked(pt1, key1));
      OcTreeNode* node1 = fullTree.updateNode(key1, true);
      EXPECT_TRUE(node1);
      EXPECT_EQ(node1, fullTree.search(pt1));
      size_t expectedNumNodes = 17; // inserting first node creates 17 nodes in total (root + 16 levels)
      size_t expectedNumLeafs = 1; // one new leaf node
      EXPECT_EQ (fullTree.size(), expectedNumNodes); 
      EXPECT_EQ (fullTree.getNumLeafNodes(), expectedNumLeafs);
      

      auto getOrCreateChild = [&](OcTreeNode* parent, unsigned int idx)->OcTreeNode* {
        OcTreeNode* child = nullptr;
        if (fullTree.nodeChildExists(parent, idx))
          child = fullTree.getNodeChild(parent, idx);
        else
          child = fullTree.createNodeChild(parent, idx);
        return child;
      };


      OcTreeNode* root = fullTree.getRoot();
      // fill first layer of children
      for (unsigned int pos1 = 0; pos1 < 8; ++pos1){
        OcTreeNode* newNodeL1 = getOrCreateChild(root, pos1);
        newNodeL1->setLogOdds(fullTree.getProbHitLog()); 
      }
      root->setLogOdds(fullTree.getProbHitLog()); 

      fullTree.deleteNode(key1); // remove initial placeholder

      // sweep now through all possible positions at level 1 (again) + 2, 3 to fill properly and fully
      for (unsigned int pos1 = 0; pos1 < 8; ++pos1){
        OcTreeNode* newNodeL1 = getOrCreateChild(root, pos1);
          for (unsigned int pos2 = 0; pos2 < 8; ++pos2){
            OcTreeNode* newNodeL2 = getOrCreateChild(newNodeL1, pos2);
            newNodeL2->setLogOdds(fullTree.getProbHitLog()); 
          }
        newNodeL1->setLogOdds(fullTree.getProbHitLog()); 
      }
      EXPECT_EQ(fullTree.size(), 73); // = 1+8+64
      EXPECT_EQ(fullTree.size(), fullTree.calcNumNodes());
      EXPECT_EQ (fullTree.getNumLeafNodes(), 64);

      
      //fullTree.write("octree_full_pruning_test.ot"); // DEBUGGING

      // workaround for bug in pruning: does not prune an already partially pruned fullTree
      fullTree.updateNode(pt1, true); 
      fullTree.toMaxLikelihood();
      fullTree.prune();

      EXPECT_EQ(fullTree.size(), 1); 
      EXPECT_EQ(fullTree.size(), fullTree.calcNumNodes());
      EXPECT_EQ (fullTree.getNumLeafNodes(), 1);

    }
  // ------------------------------------------------------------




    
    std::cerr <<"\nTest successful.\n";
    return 0;

}
