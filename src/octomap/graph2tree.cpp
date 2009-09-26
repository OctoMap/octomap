#include "octomap.h"
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "USAGE: " << self << " [options]\n"
            "OPTIONS:\n-i <InputFile.graph> (required)\n"
            "-o <OutputFile.tree> (required) \n"
            "-res <resolution> (default: 0.1 m)\n";

  exit(0);
}

int main(int argc, char** argv) {
  // default values:
  double res = 0.1;
  string graphFilename = "";
  string treeFilename = "";


  int arg = 0;
  while (++arg < argc) {
    if (! strcmp(argv[arg], "-i"))
      graphFilename = std::string(argv[++arg]);
    else if (!strcmp(argv[arg], "-o"))
      treeFilename = std::string(argv[++arg]);
    else if (! strcmp(argv[arg], "-res"))
      res = atof(argv[++arg]);
    else {
      printUsage(argv[0]);
    }
  }

  if (graphFilename == "" || treeFilename == "")
    printUsage(argv[0]);
  cout << "\nReading Graph file\n===========================\n";
  ScanGraph* graph = new ScanGraph();
  graph->readBinary(graphFilename);

  cout << "\nCreating tree\n===========================\n";
  OcTree* tree = new OcTree(res);

  unsigned numScans = graph->size();
  unsigned currentScan = 1;
  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
    cout << "("<<currentScan << "/" << numScans << ") " << flush;
    tree->insertScan(**scan_it);
    currentScan++;
  }
  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;

  unsigned numBinary, numDelta;
  tree->calcNumberOfNodesPerType(numBinary, numDelta);

  cout << "\nDone building tree.\n";
  cout << "Tree size: " << tree->size() <<"(" <<numBinary<<" binary, "<< numDelta << " delta)\n";
  cout << "Memory: " << tree->memoryUsage()
    <<" B\nFull grid: "<< tree->memoryFullGrid() <<" B\n8Pointers: " << tree->memoryUsageEightPointers() <<" B\n";
  tree->prune();
  cout << "Pruned tree size: " << tree->size() <<"(" <<numBinary<<" binary, "<< numDelta << " delta\n";
  cout << "Pruned memory: " << tree->memoryUsage() << " B\n";

  cout << "\nWriting tree file\n===========================\n";
  tree->writeBinary(treeFilename);
}
