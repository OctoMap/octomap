
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;



int main(int argc, char** argv) {



  OcTree tree (0.05);  



  //  point3d origin (10.01, 10.01, 10.02);
  point3d origin (0.01f, 0.01f, 0.02f);
  point3d point_on_surface (2.01f, 0.01f, 0.01f);

  cout << "Generating sphere at " << origin << " ..." << endl;

  unsigned sphere_beams = 500;
  double angle = 2.0*M_PI/double(sphere_beams);
  Pointcloud p;
  for (unsigned i=0; i<sphere_beams; i++) {
    for (unsigned j=0; j<sphere_beams; j++) {
      p.push_back(origin+point_on_surface);
      point_on_surface.rotate_IP (0,0,angle);
    }
    point_on_surface.rotate_IP (0,angle,0);
  }
  tree.insertPointCloud(p, origin);


  cout << "Writing to sphere.bt..." << endl;
  EXPECT_TRUE(tree.writeBinary("sphere.bt"));

  // -----------------------------------------------

  cout << "Casting rays in sphere ..." << endl;

  OcTree sampled_surface (0.05);  

  point3d direction = point3d (1.0,0.0,0.0);
  point3d obstacle(0,0,0);

  unsigned int hit (0);
  unsigned int miss (0);
  unsigned int unknown (0);
  double mean_dist(0);

  for (unsigned i=0; i<sphere_beams; i++) {
    for (unsigned j=0; j<sphere_beams; j++) {
      if (!tree.castRay(origin, direction, obstacle, false, 3.)) {
        // hit unknown
        if (!tree.search(obstacle))
          unknown++;
        else
          miss++;
      }
      else {
        hit++;
        mean_dist += (obstacle - origin).norm();
        sampled_surface.updateNode(obstacle, true);
      }
      direction.rotate_IP (0,0,angle);
    }
    direction.rotate_IP (0,angle,0);
  }

  cout << "Writing sampled_surface.bt" << endl;
  EXPECT_TRUE(sampled_surface.writeBinary("sampled_surface.bt"));

  mean_dist /= (double) hit;
  std::cout << " hits / misses / unknown: " << hit  << " / " << miss << " / " << unknown << std::endl;
  std::cout << " mean obstacle dist: " << mean_dist << std::endl;
  EXPECT_NEAR(mean_dist, 2., 0.05);
  EXPECT_EQ(hit, (sphere_beams*sphere_beams));
  EXPECT_EQ(miss, 0);
  EXPECT_EQ(unknown, 0);


  // -----------------------------------------------

  cout << "generating single rays..." << endl;
  OcTree single_beams(0.03333);
  int num_beams = 17;
  float beamLength = 10.0f;
  point3d single_origin (1.0f, 0.45f, 0.45f);
  point3d single_origin_top (1.0f, 0.45f, 1.0);
  point3d single_endpoint(beamLength, 0.0f, 0.0f);
	
	
  for (int i=0; i<num_beams; i++) {    
    for (int j=0; j<num_beams; j++) {          
      if (!single_beams.insertRay(single_origin, single_origin+single_endpoint)) {
	cout << "ERROR while inserting ray from " << single_origin << " to " << single_endpoint << endl;
      }
      single_endpoint.rotate_IP (0,0,DEG2RAD(360.0/num_beams));
    }
    single_endpoint.rotate_IP (0,DEG2RAD(360.0/num_beams),0);
  }

	
  cout << "done." << endl;
  cout << "writing to beams.bt..." << endl;
  EXPECT_TRUE(single_beams.writeBinary("beams.bt"));


  ////// more tests from unit_tests.cpp:
  double res = 0.1;
  double res_2 = res/2.0;
  OcTree cubeTree(res);
  // fill a cube with "free", end is "occupied":
  for (float x=-0.95; x <= 1.0; x+=res){
    for (float y=-0.95; y <= 1.0; y+=res){
      for (float z=-0.95; z <= 1.0; z+=res){
        if (x < 0.9){
          EXPECT_TRUE(cubeTree.updateNode(point3d(x,y,z), false));
        } else{
          EXPECT_TRUE(cubeTree.updateNode(point3d(x,y,z), true));
        }
      }
    }
  }

  // fill some "floor":
  EXPECT_TRUE(cubeTree.updateNode(res_2,res_2,-res_2, true));
  EXPECT_TRUE(cubeTree.updateNode(3*res_2,res_2,-res_2, true));
  EXPECT_TRUE(cubeTree.updateNode(-res_2,res_2,-res_2, true));
  EXPECT_TRUE(cubeTree.updateNode(-3*res_2,res_2,-res_2, true));

  cubeTree.writeBinary("raycasting_cube.bt");
  origin = point3d(0.0f, 0.0f, 0.0f);
  point3d end;
  // hit the corner:
  direction = point3d(0.95f, 0.95f, 0.95f);
  EXPECT_TRUE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_TRUE(cubeTree.isNodeOccupied(cubeTree.search(end)));
  std::cout << "Hit occupied voxel: " << end << std::endl;
  direction = point3d(1.0, 0.0, 0.0);
  EXPECT_TRUE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_TRUE(cubeTree.isNodeOccupied(cubeTree.search(end)));
  std::cout << "Hit occupied voxel: " << end << std::endl;
  EXPECT_NEAR(1.0, (origin - end).norm(), res_2);

  // hit bottom:
  origin = point3d(res_2, res_2, 0.5f);
  direction = point3d(0.0, 0.0, -1.0f);
  EXPECT_TRUE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_TRUE(cubeTree.isNodeOccupied(cubeTree.search(end)));
  std::cout << "Hit voxel: " << end << std::endl;
  EXPECT_FLOAT_EQ(origin(0), end(0));
  EXPECT_FLOAT_EQ(origin(1), end(1));
  EXPECT_FLOAT_EQ(-res_2, end(2));


  // hit boundary of unknown:
  origin = point3d(0.0f, 0.0f, 0.0f);
  direction = point3d(0.0f, 1.0f, 0.0f);
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, false));
  EXPECT_FALSE(cubeTree.search(end));
  std::cout << "Boundary unknown hit: " << end << std::endl;

  // hit boundary of octree:
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, true));
  EXPECT_FALSE(cubeTree.search(end));
  EXPECT_FLOAT_EQ(end.x(), res_2);
  EXPECT_FLOAT_EQ(end.y(), float(32768*res-res_2));
  EXPECT_FLOAT_EQ(end.z(), res_2);
  direction = point3d(-1.0f, 0.0f, 0.0f);
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, true));
  EXPECT_FALSE(cubeTree.search(end));
  EXPECT_FLOAT_EQ(end.x(), float(-32767*res-res_2));
  EXPECT_FLOAT_EQ(end.y(), res_2);
  EXPECT_FLOAT_EQ(end.z(), res_2);

  // test maxrange:
  EXPECT_FALSE(cubeTree.castRay(origin, direction, end, true, 0.9));
  std::cout << "Max range endpoint: " << end << std::endl;
  OcTreeNode* endPt = cubeTree.search(end);
  EXPECT_TRUE(endPt);
  EXPECT_FALSE(cubeTree.isNodeOccupied(endPt));
  double dist = (origin - end).norm();
  EXPECT_NEAR(0.9, dist, res);


  std::cout << "Test successful\n";
  return 0;
}
