
#include <octomap/octomap.h>
#include "octreex64.h"
#include "testing.h"

using namespace std;
using namespace octomap;

int main(int argc, char** argv) {
  const float res = 0.05f;
  OcTreeX64 tree(res);
  const float step = res;
  float x = 0.0f;
  octomap::point3d origin(-1.0, 1.0, 1.2);
  for (float y = 0.0f; y < 2.0f; y = y + step) {
    for (float z = -1.0f; z < 1.0f; z = z + step) {
      octomap::point3d end(x, y, z);
      tree.insertRay(origin, end);
    }
  }
  x = 1.0f;
  for (float y = 1.0f; y < 3.0f; y = y + step) {
    for (float z = -2.0f; z < 2.0f; z = z + step) {
        octomap::point3d end(x, y, z);
        tree.insertRay(origin, end);
    }
  }
  std::cout << "SUCCESS\n";
}
