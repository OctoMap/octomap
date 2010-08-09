
#include <stdio.h>
#include "octomap.h"
#include "OcTreeSE.h"
#include <octomath/Utils.h>

using namespace std;
using namespace octomap;
using namespace octomath;



int main(int argc, char** argv) {


  //##############################################################     

  OcTreeSE tree (0.05);  

  point3d origin (0.01, 0.01, 0.02);

  point3d endpoint (2.01,0.01,0.01);

  for (int i=0; i<360; i++) {    
    for (int j=0; j<360; j++) {
      if (!tree.insertRay(origin, endpoint)) {
        cout << "ERROR while inserting ray from " << origin << " to " << endpoint << endl;
      }
    endpoint.rotate_IP (0,0,DEG2RAD(1.));
    }
    endpoint.rotate_IP (0,DEG2RAD(1.),0);
  }  

  tree.writeBinary("sphere.bt");
}
