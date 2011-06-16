
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeLUT.h>

using namespace std;
using namespace octomap;
using namespace octomath;



int main(int argc, char** argv) {

  OcTreeLUT lut(16);

  // OcTreeKey start_key (32768, 32768, 32768);
  OcTreeKey start_key (100, 100, 100);
  OcTreeKey neighbor_key;

  cout << endl << "center key:" << endl;
  cout << "[" << start_key.k[0] << "," << start_key.k[1] << "," << start_key.k[2] << "]" << endl;

  cout << endl << "face neighbor keys:" << endl;
  
  lut.genNeighborKey(start_key, OcTreeLUT::N, neighbor_key);
  cout << "N -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::S, neighbor_key);
  cout << "S -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::W, neighbor_key);
  cout << "W -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::E, neighbor_key);
  cout << "E -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::T, neighbor_key);
  cout << "T -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::B, neighbor_key);
  cout << "B -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  cout << endl << "some edge neighbor keys:" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::SE, neighbor_key);
  cout << "SE -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::SW, neighbor_key);
  cout << "SW -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::BS, neighbor_key);
  cout << "BS -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  cout << endl << "some vertex neighbor keys:" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::TNW, neighbor_key);
  cout << "TNW -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::BNW, neighbor_key);
  cout << "BNW -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;

  lut.genNeighborKey(start_key, OcTreeLUT::BSE, neighbor_key);
  cout << "BSE -> [" << neighbor_key.k[0] << "," << neighbor_key.k[1] << "," << neighbor_key.k[2] << "]" << endl;


  return 0;
}
