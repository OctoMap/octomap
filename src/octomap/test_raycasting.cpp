
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

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
        mean_dist += (obstacle - origin).norm();
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

	// -----------------------------------------------
	cout << "generating single rays..." << endl;
	OcTree single_beams(0.03333);
	int num_beams = 17;
	double beamLength = 10.0;
	point3d single_origin (1.0, 0.45, 0.45);
	point3d single_endpoint(beamLength, 0.0, 0.0);
	
	
	for (int i=0; i<num_beams; i++) {    
			if (!single_beams.insertRay(single_origin, single_origin+single_endpoint)) {
				cout << "ERROR while inserting ray from " << single_origin << " to " << single_endpoint << endl;
			}
			single_endpoint.rotate_IP (0,0,DEG2RAD(360.0/num_beams));
		}
	
	  cout << "done." << endl;
		cout << "writing to beams.bt..." << endl;
		single_beams.writeBinary("beams.bt");

}
