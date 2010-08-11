
#include <stdio.h>
#include "octomap.h"
#include <octomath/Utils.h>

using namespace std;
using namespace octomap;
using namespace octomath;



int main(int argc, char** argv) {


  //##############################################################     

  OcTree tree (0.05);  

  //  point3d origin (10.01, 10.01, 10.02);
  point3d origin (0.01, 0.01, 0.02);
  point3d point_on_surface (2.01,0.01,0.01);

  cout << "generating sphere at " << origin << " ..." << endl;

  for (int i=0; i<360; i++) {    
    for (int j=0; j<360; j++) {
      if (!tree.insertRay(origin, origin+point_on_surface)) {
        cout << "ERROR while inserting ray from " << origin << " to " << point_on_surface << endl;
      }
      point_on_surface.rotate_IP (0,0,DEG2RAD(1.));
    }
    point_on_surface.rotate_IP (0,DEG2RAD(1.),0);
  }  

  cout << "done." << endl;
  cout << "writing to sphere.bt..." << endl;
  tree.writeBinary("sphere.bt");

  // -----------------------------------------------

  cout << "casting rays ..." << endl;

  OcTree sampled_surface (0.05);  

  point3d direction = point3d (1.0,0.0,0.0);
  point3d obstacle(0,0,0);

  unsigned int hit (0);
  unsigned int miss (0);
  double mean_dist(0);

  for (int i=0; i<360; i++) {    
    for (int j=0; j<360; j++) {
      if (!tree.castRay(origin, direction, obstacle, true, 3.)) {
        miss++;
      }
      else {
        hit++;
        mean_dist += (obstacle - origin).norm2();
        sampled_surface.updateNode(obstacle, true);
      }
      direction.rotate_IP (0,0,DEG2RAD(1.));
    }
    direction.rotate_IP (0,DEG2RAD(1.),0);
  }
  cout << "done." << endl;

  mean_dist /= (double) hit;
  std::cout << " hits / misses: " << hit  << " / " << miss << std::endl;
  std::cout << " mean obstacle dist: " << mean_dist << std::endl;

  cout << "writing sampled_surface.bt" << endl;
  sampled_surface.writeBinary("sampled_surface.bt");


}
