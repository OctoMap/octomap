#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace octomap;


int main(int argc, char** argv) {

  std::string filename(argv[1]);

  std::ifstream infile(filename.c_str(), std::ios_base::in |std::ios_base::binary);
  if (!infile.is_open()) {
    cout << "file "<< filename << " could not be opened for reading.\n";
    return -1;
  }

  ColorOcTree tree (0.1);
  tree.readData(infile);
  infile.close();
  cout << "color tree read from "<< filename <<"\n"; 

  tree.writeColorHistogram("histogram.eps");

  return 0;
}
