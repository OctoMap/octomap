
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <octomap/octomap_timing.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " input.bt\n\n";


  exit(1);
}

void computeChildCenter (const unsigned int& pos,
    const float& center_offset,
    const point3d& parent_center,
    point3d& child_center) {
  // x-axis
  if (pos & 1) child_center(0) = parent_center(0) + center_offset;
  else     child_center(0) = parent_center(0) - center_offset;

  // y-axis
  if (pos & 2) child_center(1) = parent_center(1) + center_offset;
  else   child_center(1) = parent_center(1) - center_offset;
  // z-axis
  if (pos & 4) child_center(2) = parent_center(2) + center_offset;
  else   child_center(2) = parent_center(2) - center_offset;
}

/// mimics old deprecated behavior to compare against
void getLeafNodesRecurs(std::list<OcTreeVolume>& voxels,
    unsigned int max_depth,
    OcTreeNode* node, unsigned int depth,
    const point3d& parent_center, const point3d& tree_center,
    OcTree* tree, bool occupied)
{
  if ((depth <= max_depth) && (node != NULL) ) {
    if (node->hasChildren() && (depth != max_depth)) {

      double center_offset = tree_center(0) / pow( 2., (double) depth+1);
      point3d search_center;

      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {

          computeChildCenter(i, center_offset, parent_center, search_center);
          getLeafNodesRecurs(voxels, max_depth, node->getChild(i), depth+1, search_center, tree_center, tree, occupied);

        } // GetChild
      }
    }
    else {
      if (tree->isNodeOccupied(node) == occupied){
        double voxelSize = tree->getResolution() * pow(2., double(16 - depth));
        voxels.push_back(std::make_pair(parent_center - tree_center, voxelSize));

      }
    }
  }
}


/// mimics old deprecated behavior to compare against
void getVoxelsRecurs(std::list<OcTreeVolume>& voxels,
                                       unsigned int max_depth,
                                       OcTreeNode* node, unsigned int depth,
                                       const point3d& parent_center, const point3d& tree_center,
                                       double resolution){

  if ((depth <= max_depth) && (node != NULL) ) {
    if (node->hasChildren() && (depth != max_depth)) {

      double center_offset = tree_center(0) / pow(2., (double) depth + 1);
      point3d search_center;

      for (unsigned int i = 0; i < 8; i++) {
        if (node->childExists(i)) {
          computeChildCenter(i, (float) center_offset, parent_center, search_center);
          getVoxelsRecurs(voxels, max_depth, node->getChild(i), depth + 1, search_center, tree_center, resolution);

        }
      } // depth
    }
    double voxelSize = resolution * pow(2., double(16 - depth));
    voxels.push_back(std::make_pair(parent_center - tree_center, voxelSize));
  }
}


/// compare two lists of octree nodes on equality
void compareResults(const std::list<OcTreeVolume>& list_iterator, const std::list<OcTreeVolume>& list_depr){
  EXPECT_EQ(list_iterator.size(), list_depr.size());
  std::cout << "Sizes match: "<< list_iterator.size() << std::endl;


    list<OcTreeVolume>::const_iterator list_it = list_iterator.begin();
    list<OcTreeVolume>::const_iterator list_depr_it = list_depr.begin();

    for (; list_it != list_iterator.end(); ++list_it, ++list_depr_it){
      EXPECT_NEAR(list_it->first.x(), list_depr_it->first.x(), 0.005);
      EXPECT_NEAR(list_it->first.y(), list_depr_it->first.y(), 0.005);
      EXPECT_NEAR(list_it->first.z(), list_depr_it->first.z(), 0.005);
    }
}

// for unique comparing, need to sort the lists:
bool OcTreeVolumeSortPredicate(const OcTreeVolume& lhs, const OcTreeVolume& rhs)
{
  return ( lhs.second < rhs.second
      ||  (lhs.second == rhs.second &&
          lhs.first.x() < rhs.first.x()
          &&  lhs.first.y() < rhs.first.y()
          &&  lhs.first.z() < rhs.first.z()));
}


double timediff(const timeval& start, const timeval& stop){
  return (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
}

int main(int argc, char** argv) {


  //##############################################################     

  string btFilename = "";


  // test timing:
  timeval start;
  timeval stop;
  double time_it, time_depr;

  if (argc != 2 || (argc > 1 && strcmp(argv[1], "-h") == 0)){
    printUsage(argv[0]);
  }

  btFilename = std::string(argv[1]);

  cout << "\nReading OcTree file\n===========================\n";
  OcTree* tree = new OcTree(btFilename);
  if (tree->size()<= 1){
    std::cout << "Error reading file, exiting!\n";
    return 1;
  }

  unsigned count;
  std::list<OcTreeVolume> list_depr;
  std::list<OcTreeVolume> list_iterator;

  /**
   * get number of nodes:
   */
  gettimeofday(&start, NULL);  // start timer
  unsigned num_leafs_recurs = tree->getNumLeafNodes();
  gettimeofday(&stop, NULL);  // stop timer
  time_depr = timediff(start, stop);

  gettimeofday(&start, NULL);  // start timer
  unsigned num_leafs_it = 0;
  for(OcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
    num_leafs_it++;
  }
  gettimeofday(&stop, NULL);  // stop timer
  time_it = timediff(start, stop);
  std::cout << "Number of leafs: " << num_leafs_it << " / " << num_leafs_recurs << ", times: "
        <<time_it << " / " << time_depr << "\n========================\n\n";


  /**
   * get all occupied leafs
   */
  const unsigned char tree_depth(16);
  const unsigned int tree_max_val(32768);
  point3d tree_center;
  tree_center(0) = tree_center(1) = tree_center(2)
              = (float) (((double) tree_max_val) * tree->getResolution());

  gettimeofday(&start, NULL);  // start timer
  getLeafNodesRecurs(list_depr,tree_depth,tree->getRoot(), 0, tree_center, tree_center, tree, true);
  gettimeofday(&stop, NULL);  // stop timer
  time_depr = timediff(start, stop);

  gettimeofday(&start, NULL);  // start timer
  for(OcTree::iterator it = tree->begin(), end=tree->end(); it!= end; ++it){
    if(tree->isNodeOccupied(*it))
    {
      //count ++;
     list_iterator.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
    }

  }
  gettimeofday(&stop, NULL);  // stop timer
  time_it = timediff(start, stop);

  compareResults(list_iterator, list_depr);
  std::cout << "Occupied lists traversed, times: "
      <<time_it << " / " << time_depr << "\n========================\n\n";


  /**
   * get all free leafs
   */
  list_iterator.clear();
  list_depr.clear();
  gettimeofday(&start, NULL);  // start timer
  for(OcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
    if(!tree->isNodeOccupied(*it))
      list_iterator.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
  }
  gettimeofday(&stop, NULL);  // stop timer
  time_it = timediff(start, stop);

  gettimeofday(&start, NULL);  // start timer
  getLeafNodesRecurs(list_depr,tree_depth,tree->getRoot(), 0, tree_center, tree_center, tree, false);
  gettimeofday(&stop, NULL);  // stop timer
  time_depr = timediff(start, stop);

  compareResults(list_iterator, list_depr);
  std::cout << "Free lists traversed, times: "
      <<time_it << " / " << time_depr << "\n========================\n\n";




  /**
   * get all volumes
   */
  list_iterator.clear();
  list_depr.clear();

  gettimeofday(&start, NULL);  // start timer
  getVoxelsRecurs(list_depr,tree_depth,tree->getRoot(), 0, tree_center, tree_center, tree->getResolution());
  gettimeofday(&stop, NULL);  // stop timer
  time_depr = timediff(start, stop);

  gettimeofday(&start, NULL);  // start timers
  for(OcTree::tree_iterator it = tree->begin_tree(), end=tree->end_tree();
      it!= end; ++it){
      //count ++;
     list_iterator.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
  }
  gettimeofday(&stop, NULL);  // stop timer
  time_it = timediff(start, stop);

  list_iterator.sort(OcTreeVolumeSortPredicate);
  list_depr.sort(OcTreeVolumeSortPredicate);

  compareResults(list_iterator, list_depr);
  std::cout << "All inner lists traversed, times: "
      <<time_it << " / " << time_depr << "\n========================\n\n";

  /**
   * get bounding box
   */
  point3d min(-1, -1, -1);
  point3d max(5, 5, 5);
  list_iterator.clear();
  list_depr.clear();
  gettimeofday(&start, NULL);  // start timer
  //tree->setBBXMax()  getVoxels(list_depr);
  gettimeofday(&stop, NULL);  // stop timer
  time_depr = timediff(start, stop);

  OcTree bbxTree(tree->getResolution());
  tree->expand();

  count = 0;
  gettimeofday(&start, NULL);  // start timers
  for(OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min,max), end=tree->end_leafs_bbx();
      it!= end; ++it){
    count ++;
    bbxTree.updateNode(it.getCoordinate(), tree->isNodeOccupied(*it));
  }
  gettimeofday(&stop, NULL);  // stop timer
  time_it = timediff(start, stop);

  bbxTree.writeBinary("test_bbx.bt");
  std::cout << "BBX volume traversed, "
    << count<< " voxels in " << time_it << "s\n========================\n\n";



  return 0;
}

