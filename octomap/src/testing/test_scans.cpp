
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;



int main(int argc, char** argv) {


  //##############################################################     

  OcTree tree (0.05);  

  point3d origin (0.01f, 0.01f, 0.02f);
  point3d point_on_surface (4.01f, 0.01f, 0.01f);

  cout << "generating spherical scan at " << origin << " ..." << endl;

  Pointcloud cloud;

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d rotated = point_on_surface;
      rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
      cloud.push_back(rotated);
    }
  }  

  // insert in global coordinates:
  tree.insertPointCloud(cloud, origin);

  cout << "done." << endl;
  cout << "writing to spherical_scan.bt..." << endl;
  tree.writeBinary("spherical_scan.bt");



}
