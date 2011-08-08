#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"
 
using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {
  
  {
    OcTree tree (0.05);
    tree.setProbHit(0.7);
    tree.setProbMiss(0.4);

    point3d origin (0.01, 0.01, 0.02);
    point3d point_on_surface (2.01,0.01,0.01);
  
    for (int i=0; i<360; i++) {    
      for (int j=0; j<360; j++) {
        EXPECT_TRUE (tree.insertRay(origin, origin+point_on_surface));
        point_on_surface.rotate_IP (0,0,DEG2RAD(1.));
      }
      point_on_surface.rotate_IP (0,DEG2RAD(1.),0);
    }

    tree.writeBinary("sphere.bt");
  
    OcTree sampled_surface (0.05);  
    point3d direction = point3d (1.0,0.0,0.0);
    point3d obstacle(0,0,0);
    unsigned int hits (0);
    unsigned int misses (0);
    double mean_dist(0);

    for (int i=0; i<360; i++) {    
      for (int j=0; j<360; j++) {
        if (!tree.castRay(origin, direction, obstacle, true, 3.)) {
          misses++;
        }
        else {
          hits++;
          mean_dist += (obstacle - origin).norm ();
          sampled_surface.updateNode(obstacle, true);
        }
        direction.rotate_IP (0,0,DEG2RAD(1.));
      }
      direction.rotate_IP (0,DEG2RAD(1.),0);
    }
  
    mean_dist /= (double) hits;
    //   std::cout << " hits / misses: " << hits  << " / " << misses << std::endl;
    //   std::cout << " mean obstacle dist: " << mean_dist << std::endl;
  
    //   cout << "writing sampled_surface.bt: " << flush;
    sampled_surface.writeBinary("sampled_surface.bt");
  
    EXPECT_EQ ((int) hits, 129416);
    EXPECT_EQ ((int) misses,  184);
    EXPECT_NEAR(mean_dist, 2., 0.1);
    printf("expected: 129416, got: %d\n ", hits);
  }

  // insert scan test
  // insert graph node test
  {
    Pointcloud* measurement = new Pointcloud();
  
    point3d origin (0.01, 0.01, 0.02);
    point3d point_on_surface (2.01,0.01,0.01);
  
    for (int i=0; i<360; i++) {    
      for (int j=0; j<360; j++) {
        point3d p = origin+point_on_surface;
        measurement->push_back(p);
        point_on_surface.rotate_IP (0,0,DEG2RAD(1.));
      }
      point_on_surface.rotate_IP (0,DEG2RAD(1.),0);
    }
  
    OcTree tree (0.05);
    tree.insertScan(*measurement, origin);
    EXPECT_EQ ((int) tree.size(), 54076);

    ScanGraph* graph = new ScanGraph();
    Pose6D node_pose (0.01, 0.01, 0.02, 0,0,0);
    graph->addNode(measurement, node_pose);
    graph->writeBinary("test.graph");
    delete graph;
  }

  // tree read file test
  {
    OcTree tree (0.05);  
    tree.readBinary("sphere.bt");
  }
  
  // graph read file test
  {
    ScanGraph graph;
    graph.readBinary("test.graph");
  }
  
  // re-read written file (ensure that file is readable)
  {
    OcTree tree (0.05);  
    tree.readBinary("sphere.bt");
  }
  
  return 0;
}
