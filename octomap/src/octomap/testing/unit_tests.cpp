#include <stdio.h>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/math/Utils.h>
#include "testing.h"
 
using namespace std;
using namespace octomap;
using namespace octomath;

int main(int argc, char** argv) {

  if (argc < 2) return -1;
  std::string test_name (argv[1]);


  // ------------------------------------------------------------
  if (test_name == "MathVector") {
    // test constructors
    Vector3* twos = new Vector3();        
    Vector3* ones = new Vector3(1,1,1);    
    for (int i=0;i<3;i++) {
      (*twos)(i) = 2;
    }  
    // test basic operations
    Vector3 subtraction = *twos - *ones;
    Vector3 addition = *twos + *ones;
    Vector3 multiplication = *twos * 2.;
  
    for (int i=0;i<3;i++) {
      EXPECT_FLOAT_EQ (subtraction(i), 1.);
      EXPECT_FLOAT_EQ (addition(i), 3.);
      EXPECT_FLOAT_EQ (multiplication(i), 4.);
    }

    // copy constructor
    Vector3 rotation =  *ones;

    // rotation
    rotation.rotate_IP (M_PI, 1., 0.1);
    EXPECT_FLOAT_EQ (rotation.x(), 1.2750367);
    EXPECT_FLOAT_EQ (rotation.y(), -1.1329513);
    EXPECT_FLOAT_EQ (rotation.z(), 0.30116868);
  }
  
  // ------------------------------------------------------------
  if (test_name == "MathPose") {
    // constructors  
    Pose6D a (1. ,0.1 ,0.1 , 0., 0.1, M_PI/4.);
    Pose6D b ();

    Vector3 trans(1., 0.1, 0.1);
    Quaternion rot(0., 0.1, M_PI/4.);
    Pose6D c(trans, rot);

    // comparator
    EXPECT_TRUE ( a == c);
    // toEuler
    EXPECT_FLOAT_EQ (c.yaw() , M_PI/4.);

    // transform
    Vector3 t = c.transform (trans);
    EXPECT_FLOAT_EQ (t.x() , 1.6399229);
    EXPECT_FLOAT_EQ (t.y() , 0.8813442);
    EXPECT_FLOAT_EQ (t.z() , 0.099667005);

    // inverse transform
    Pose6D c_inv = c.inv();
    Vector3 t2 = c_inv.transform (t);
    EXPECT_FLOAT_EQ (t2.x() , trans.x());
    EXPECT_FLOAT_EQ (t2.y() , trans.y());
    EXPECT_FLOAT_EQ (t2.z() , trans.z());
  }
  
  // ------------------------------------------------------------
  if (test_name == "InsertRay") {
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
    EXPECT_TRUE (tree.writeBinary("sphere.bt"));
  }

  // ------------------------------------------------------------
  // tree read file test
  if (test_name == "ReadTree") {
    OcTree tree (0.05);  
    EXPECT_TRUE (tree.readBinary("sphere.bt"));
    EXPECT_EQ ((int) tree.size(), 59420);
  }

  if (test_name == "CastRay") {
    OcTree tree (0.05);  
    EXPECT_TRUE (tree.readBinary("sphere.bt"));
    OcTree sampled_surface (0.05);  
    point3d origin (0.01, 0.01, 0.02);
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
    EXPECT_TRUE (sampled_surface.writeBinary("sampled_surface.bt"));
  
    mean_dist /= (double) hits;
    EXPECT_NEAR(mean_dist, 2., 0.1);
  
    EXPECT_EQ ((int) hits, 129416);
    EXPECT_EQ ((int) misses,  184);
  }

  // ------------------------------------------------------------
  // insert scan test
  // insert graph node test
  // write graph test
  if (test_name == "InsertScan") {
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
    EXPECT_TRUE (graph->writeBinary("test.graph"));
    delete graph;
  }
  
  // ------------------------------------------------------------
  // graph read file test
  if (test_name == "ReadGraph") {
    ScanGraph graph;
    EXPECT_TRUE (graph.readBinary("test.graph"));
  }

  // ------------------------------------------------------------
  if (test_name == "StampedTree") { 
    OcTreeStamped stamped_tree (0.05);
    // fill tree
    for (int x=-20; x<20; x++) 
      for (int y=-20; y<20; y++) 
        for (int z=-20; z<20; z++) {
          point3d p ((double) x*0.05+0.01, (double) y*0.05+0.01, (double) z*0.05+0.01);
          stamped_tree.updateNode(p, true); // integrate 'occupied' measurement 
        }
    // test if update times set
    point3d query (0.1, 0.1, 0.1);
    OcTreeNodeStamped* result = stamped_tree.search (query);
    EXPECT_TRUE (result);
    unsigned int tree_time = stamped_tree.getLastUpdateTime();
    unsigned int node_time = result->getTimestamp();
    EXPECT_TRUE (tree_time > 0);
    EXPECT_EQ (node_time, tree_time);
  }

  fprintf(stderr, "test successful.\n");
  return 0;
}
