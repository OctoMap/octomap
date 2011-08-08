
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " input.bt\n\n";


  exit(0);
}

void compareResults(const std::list<OcTreeVolume>& list_iterator, const std::list<OcTreeVolume>& list_depr){
  if (list_iterator.size() == list_depr.size()){
      std::cout << " Sizes match: "<< list_iterator.size() << std::endl;
    } else {
      std::cerr << "Size mismatch: " << list_iterator.size() << " / " << list_depr.size() <<std::endl;
      std::cerr << "Skipping exact test." <<std::endl;
      return;

    }

    list<OcTreeVolume>::const_iterator list_it = list_iterator.begin();
    list<OcTreeVolume>::const_iterator list_depr_it = list_depr.begin();

    for (; list_it != list_iterator.end(); ++list_it, ++list_depr_it){
      if (   abs(list_it->first.x() -list_depr_it->first.x()) > 0.001
          || abs(list_it->first.y() -list_depr_it->first.y()) > 0.001
          || abs(list_it->first.z() -list_depr_it->first.z()) > 0.001)

      {
        std::cerr << "Error: difference at pos " << list_it->first << " / " << list_depr_it->first << std::endl;
      }
      if (!(list_it->second == list_depr_it->second)){
        std::cerr << "Error: difference at res " << list_it->second << " / " << list_depr_it->second << std::endl;
      }
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

  std::list<OcTreeVolume> list_depr;
  unsigned count;
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

  gettimeofday(&start, NULL);  // start timer
  tree->getOccupied(list_depr);
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
  tree->getFreespace(list_depr);
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
  tree->getVoxels(list_depr);
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




}

