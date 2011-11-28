
#include <stdio.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

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

int main(int argc, char** argv) {

	//Generate a MapCollection
	MapCollection<MapNode<OcTree> > coll;
	OcTree* tree1 = generateSphereTree(point3d(0.01f,0.01f,0.01f), 2.0f);
	MapNode<OcTree>* mn1 = new MapNode<OcTree>(tree1, pose6d(0.0,0.0,0.0,0.0,0.0,0.0));
	mn1->setId("nodeone");
	coll.addNode(mn1);
	OcTree* tree2 = generateSphereTree(point3d(0.01f,0.01f,0.01f), 3.5f);
	MapNode<OcTree>* mn2 = new MapNode<OcTree>(tree2, pose6d(3.0,7.0,10.0,0.0,0.0,0.0));
	mn2->setId("nodetwo");
	coll.addNode(mn2);

	//Write the MapCollection
	coll.write("tempcollection.txt");

	//Read it back in
	MapCollection<MapNode<OcTree> > coll_read("tempcollection.txt");

	//TODO do some ray operations
	//TODO do some isOccupied operations
	//TODO do some comparisons between original and re-read MaCollection

	//Read MapCollection from command line
	std::string filename(argv[1]);
	MapCollection<MapNode<OcTree> > collection(filename);

	//Write it to file
	collection.write("writeout.txt");
  //Write pointcloud to file
	collection.writePointcloud("test.vrml");

	//TODO delete temporary files?
	//tempcollection.txt
	//nodeone.bt
	//nodetwo.bt
	//writeout.txt
	//test.vrml

  return 0;
}
