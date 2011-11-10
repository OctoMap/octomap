
#include <stdio.h>
#include <octomap/MapCollection.h>

using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {

	std::string filename(argv[1]);

	MapCollection<MapNode<OcTree> > collection(filename);

	collection.write("writeout.txt");

	collection.writePointcloud("test.vrml");




  return 0;
}
