#include <octomap/octomap.h>

int main(int argc, char** argv) {
  octomap::OcTree tree(0.1);
  octomap::point3d ptMinbbx(-0.1, -0.1, -0.1);
  octomap::point3d ptMaxbbx(0.1, 0.1, 0.1);
  const float clampingThreshold = tree.getClampingThresMinLog();
  for (octomap::OcTree::leaf_bbx_iterator it =
       tree.begin_leafs_bbx(ptMinbbx, ptMaxbbx),
       end = tree.end_leafs_bbx();
       it != end; ++it) {
    it->setLogOdds(clampingThreshold);
  }
  return 0;
}
