#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace octomap;


void print_query_info(point3d query, ColorOcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    cout << "color of node is: [" << (unsigned int) node->getColor().r << " , "
         << (unsigned int) node->getColor().g << " , " 
         << (unsigned int) node->getColor().b << "]" 
         << endl;    
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;  
}


int main(int argc, char** argv) {

  ColorOcTree tree (0.1);  // create empty tree with resolution 0.1
  // insert some measurements of occupied cells
  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f+0.01f, (float) y*0.05f+0.01f, (float) z*0.05f+0.01f);
        ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
        n->setColor(z*5+100,x*5+100,y*5+100); // set color to red
      }
    }
  }

  // insert some measurements of free cells
  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f+2.0f, (float) y*0.02f+2.0f, (float) z*0.02f+2.0f);
        ColorOcTreeNode* n = tree.updateNode(endpoint, false); 
        n->setColor(255,255,0); // set color to yellow
      }
    }
  }

  // set inner node colors
  tree.updateInnerOccupancy();

  cout << endl;
  cout << "performing some queries:" << endl;
  
  {
    point3d query (0., 0., 0.);
    ColorOcTreeNode* result = tree.search (query);
    print_query_info(query, result);
    
    query = point3d(-1.,-1.,-1.);
    result = tree.search (query);
    print_query_info(query, result);
    
    query = point3d(1.,1.,1.);
    result = tree.search (query);
    print_query_info(query, result);
  }

 
  std::string filename ("simple_tree.cot");
  // write color tree
  std::ofstream outfile(filename.c_str(), std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    tree.writeConst(outfile); 
    outfile.close();
    cout << "color tree written "<< filename <<"\n";
  }
  else {
    cout << "could not open file "<< filename << " for writing\n";
    return -1;
  }

  // read tree file
  std::ifstream infile(filename.c_str(), std::ios_base::in |std::ios_base::binary);
  if (!infile.is_open()) {
    cout << "file "<< filename << " could not be opened for reading.\n";
    return -1;
  }

  ColorOcTree read_tree (0.1);
  read_tree.read(infile);
  infile.close();
  cout << "color tree read from "<< filename <<"\n"; 

  // perform queries again
  {
    point3d query (0., 0., 0.);
    ColorOcTreeNode* result = read_tree.search (query);
    print_query_info(query, result);

    query = point3d(-1.,-1.,-1.);
    result = read_tree.search (query);
    print_query_info(query, result);

    query = point3d(1.,1.,1.);
    result = read_tree.search (query);
    print_query_info(query, result);
  }

  return 0;
}
