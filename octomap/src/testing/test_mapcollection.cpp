
#include <stdio.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;

OcTree* generateSphereTree(point3d origin, float radius){
	OcTree* tree = new OcTree(0.05);

	point3d point_on_surface = origin;
	point_on_surface.x() += radius;
	for (int i=0; i<360; i++) {
		for (int j=0; j<360; j++) {
			if (!tree->insertRay(origin, origin+point_on_surface)) {
				cout << "ERROR while inserting ray from " << origin << " to " << point_on_surface << endl;
			}
			point_on_surface.rotate_IP (0,0,DEG2RAD(1.));
		}
		point_on_surface.rotate_IP (0,DEG2RAD(1.),0);
	}
	return tree;
}

int main(int /*argc*/, char** argv) {

  // //Generate a MapCollection
  // MapCollection<MapNode<OcTree> > coll;
  // OcTree* tree1 = generateSphereTree(point3d(0.01f,0.01f,0.01f), 2.0f);
  // MapNode<OcTree>* mn1 = new MapNode<OcTree>(tree1, pose6d(0.0,0.0,0.0,0.0,0.0,0.0));
  // mn1->setId("nodeone");
  // coll.addNode(mn1);
  // OcTree* tree2 = generateSphereTree(point3d(0.01f,0.01f,0.01f), 3.5f);
  // MapNode<OcTree>* mn2 = new MapNode<OcTree>(tree2, pose6d(3.0,7.0,10.0,0.0,0.0,0.0));
  // mn2->setId("nodetwo");
  // coll.addNode(mn2);

  // //Write the MapCollection
  // coll.write("tempcollection.txt");

  // //Read it back in
  // MapCollection<MapNode<OcTree> > coll_read("tempcollection.txt");

  //TODO do some ray operations
  //TODO do some isOccupied operations
  //TODO do some comparisons between original and re-read MaCollection

  //Read MapCollection from command line
  std::string filename(argv[1]);
  MapCollection<MapNode<OcTree> > collection(filename);
  EXPECT_TRUE(collection.size() > 0);

  //Write it to file
  collection.write("writeout.txt");
  //Write pointcloud to file
  //	collection.writePointcloud("test.vrml");

  //TODO delete temporary files?
  //tempcollection.txt
  //nodeone.bt
  //nodetwo.bt
  //writeout.txt
  //test.vrml

  std::vector<point3d> query;
  query.push_back(point3d(0,0,0));
  query.push_back(point3d(2,0,0));
  query.push_back(point3d(2,0,2));
  query.push_back(point3d(1.99f,0.0f,0.0f));
  query.push_back(point3d(0,0,3));
  query.push_back(point3d(3,7,13.5));
  query.push_back(point3d(0,-1,-1));

  for (std::vector<point3d>::iterator it = query.begin(); it != query.end(); ++it) {
    point3d& q = *it;
    if (collection.isOccupied(q))
      printf("q (%0.2f, %0.2f, %0.2f) is occupied\n", q.x(), q.y(), q.z());
    else 
      printf("q (%0.2f, %0.2f, %0.2f) is NOT occupied\n", q.x(), q.y(), q.z());
    printf("in fact, it has an occupancy probability of %0.2f\n", collection.getOccupancy(q));
  }

  point3d ray_origin (0,0,10);
  point3d ray_direction (0,0,-10);
  point3d ray_end (100,100,100);
  
  if (collection.castRay(ray_origin, ray_direction, ray_end,true)) {
    printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f hit obstacle at %.2f,%.2f,%.2f\n",
           ray_origin.x(), ray_origin.y(), ray_origin.z(),
           ray_direction.x(), ray_direction.y(), ray_direction.z(),
           ray_end.x(), ray_end.y(), ray_end.z());
  }
  else {
    printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f FAIL\n",
           ray_origin.x(), ray_origin.y(), ray_origin.z(),
           ray_direction.x(), ray_direction.y(), ray_direction.z());
  }


  printf("\n\n");

  ray_origin = point3d(0,0,-10);
  ray_direction = point3d(0,0,10);
  
  if (collection.castRay(ray_origin, ray_direction, ray_end,true)) {
    printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f hit obstacle at %.2f,%.2f,%.2f\n",
           ray_origin.x(), ray_origin.y(), ray_origin.z(),
           ray_direction.x(), ray_direction.y(), ray_direction.z(),
           ray_end.x(), ray_end.y(), ray_end.z());
  }
  else {
    printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f FAIL\n",
           ray_origin.x(), ray_origin.y(), ray_origin.z(),
           ray_direction.x(), ray_direction.y(), ray_direction.z());
  }

  printf("\n\n");

  //....

  ray_origin = point3d(0,-1.5,-3);
  ray_direction = point3d(0,0,1);
  
  if (collection.castRay(ray_origin, ray_direction, ray_end,true, 5)) {
    printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f hit obstacle at %.2f,%.2f,%.2f\n",
           ray_origin.x(), ray_origin.y(), ray_origin.z(),
           ray_direction.x(), ray_direction.y(), ray_direction.z(),
           ray_end.x(), ray_end.y(), ray_end.z());
  }
  else {
    printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f FAIL\n",
           ray_origin.x(), ray_origin.y(), ray_origin.z(),
           ray_direction.x(), ray_direction.y(), ray_direction.z());
  }


  return 0;
}
