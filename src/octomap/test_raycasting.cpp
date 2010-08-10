
#include <stdio.h>
#include "octomap.h"
#include <octomath/Utils.h>

using namespace std;
using namespace octomap;
using namespace octomath;



int main(int argc, char** argv) {


  //##############################################################     

  OcTree tree (0.05);  

  point3d origin (0.01, 0.01, 0.02);
  point3d endpoint (2.01,0.01,0.01);

  cout << "generating sphere at " << origin << " ..." << endl;

  for (int i=0; i<360; i++) {    
    for (int j=0; j<360; j++) {
      if (!tree.insertRay(origin, endpoint)) {
        cout << "ERROR while inserting ray from " << origin << " to " << endpoint << endl;
      }
    endpoint.rotate_IP (0,0,DEG2RAD(1.));
    }
    endpoint.rotate_IP (0,DEG2RAD(1.),0);
  }  

  cout << "done." << endl;
  cout << "writing to sphere.bt..." << endl;
  tree.writeBinary("sphere.bt");
}
