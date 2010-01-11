// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "octomap.h"
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "USAGE: " << self << " [options]\n"
            "OPTIONS:\n-i <InputFile.graph> (required)\n"
            "-o <OutputFile.bt> (required) \n"
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

  unsigned numThresholded, numOther;
  tree->calcNumThresholdedNodes(numThresholded, numOther);

  cout << "\nDone building tree.\n";
  cout << "Tree size: " << tree->size() <<"(" <<numThresholded <<" thresholded, "<< numOther << " other)\n";
  cout << "Memory: " << tree->memoryUsage()
       <<" B\nFull grid: "<< tree->memoryFullGrid() <<" B\n";

  tree->prune();
  tree->calcNumThresholdedNodes(numThresholded, numOther);

  cout << "Pruned tree size: " << tree->size() <<"(" <<numThresholded<<" thresholded, "<< numOther << " other)\n";
  cout << "Pruned memory: " << tree->memoryUsage() << " B\n";

  cout << "\nWriting tree file\n===========================\n";
  tree->writeBinary(treeFilename);
}
