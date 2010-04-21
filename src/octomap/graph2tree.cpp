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
#include "OcTreeFileIO.h"
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " [options]\n\n"
            "OPTIONS:\n-i <InputFile.graph> (required)\n"
            "-o <OutputFile.bt> (required) \n"
            "-m <maxrange> (optional) \n"
            "-n <max scan no.> (optional) \n"
            "-res <resolution> (default: 0.1 m)\n\n";

  std::cerr << "This tool inserts the data of a binary" << std::endl;
  std::cerr << "graph file into an octree. A binary octree" << std::endl;
  std::cerr << "file (.bt, bonsai tree) is generated." << std::endl;
  std::cerr << std::endl;

  exit(0);
}

int main(int argc, char** argv) {
  // default values:
  double res = 0.1;
  string graphFilename = "";
  string treeFilename = "";
  double maxrange = -1;
  int max_scan_no = -1;

  timeval start; 
  timeval stop; 

  int arg = 0;
  while (++arg < argc) {
    if (! strcmp(argv[arg], "-i"))
      graphFilename = std::string(argv[++arg]);
    else if (!strcmp(argv[arg], "-o"))
      treeFilename = std::string(argv[++arg]);
    else if (! strcmp(argv[arg], "-res"))
      res = atof(argv[++arg]);
    else if (! strcmp(argv[arg], "-m"))
      maxrange = atof(argv[++arg]);
    else if (! strcmp(argv[arg], "-n"))
      max_scan_no = atoi(argv[++arg]);
    else {
      printUsage(argv[0]);
    }
  }

  if (graphFilename == "" || treeFilename == "")
    printUsage(argv[0]);


  std::string treeFilenameOT = treeFilename + ".ot";
  cout << "\nReading Graph file\n===========================\n";
  ScanGraph* graph = new ScanGraph();
  graph->readBinary(graphFilename);
  unsigned int num_points_in_graph = 0;
  if (max_scan_no > 0) {
    num_points_in_graph = graph->getNumPoints(max_scan_no-1);
    cout << "\n Data points in graph upto scan " << max_scan_no << ": " << num_points_in_graph << endl;
  }
  else {
    num_points_in_graph = graph->getNumPoints();
    cout << "\n Data points in graph: " << num_points_in_graph << endl;
  }

  cout << "\nCreating tree\n===========================\n";
  OcTree* tree = new OcTree(res);


  gettimeofday(&start, NULL);  // start timer
  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;
  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
    if (max_scan_no > 0) cout << "("<<currentScan << "/" << max_scan_no << ") " << flush;
    else cout << "("<<currentScan << "/" << numScans << ") " << flush;

    tree->insertScan(**scan_it, maxrange, false);
    currentScan++;
    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no)) break;
  }
  gettimeofday(&stop, NULL);  // stop timer
  
  double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);

  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;



  cout << "\nDone building tree.\n\n";
  cout << "time to insert scans: " << time_to_insert << " sec" << endl;
  cout << "inserting 100.000 points took: " << time_to_insert/ ((double) num_points_in_graph / 100000) << " sec (average)" << endl << endl;  

  unsigned int numThresholded, numOther;
  tree->calcNumThresholdedNodes(numThresholded, numOther);

  cout << "Tree size: " << tree->size() <<" (" <<numThresholded <<" thresholded, "<< numOther << " other)\n";

  unsigned int memUsage = tree->memoryUsage();
  unsigned int memFullGrid = tree->memoryFullGrid();
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;

  tree->prune();
  tree->calcNumThresholdedNodes(numThresholded, numOther);
  memUsage = tree->memoryUsage();

  cout << endl;
  cout << "Pruned tree size: " << tree->size() <<" (" <<numThresholded<<" thresholded, "<< numOther << " other)\n";
  cout << "Pruned memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;

  double x, y, z;
  tree->getMetricSize(x, y, z);
  cout << "size: " << x << " x " << y << " x " << z << endl;
  cout << endl;

  cout << "\nWriting tree files\n===========================\n";
  OcTreeFileIO::write(tree, treeFilenameOT);
  std::cout << "Full Octree written to "<< treeFilenameOT << std::endl;
  tree->writeBinary(treeFilename);
  std::cout << "Bonsai tree written to "<< treeFilename << std::endl;
  cout << endl;

  delete tree;
}
