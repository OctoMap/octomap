#include <stdio.h>
#include <octomap/octomap.h>
#include "testing.h"
 
using namespace std;
using namespace octomath;

int main(int argc, char** argv) {

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

  return 0;
}
