// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "USAGE: " << self << " [options]\n\n";
  std::cerr << "This tool is part of OctoMap and inserts the data of a scan graph\n"
               "file (point clouds with poses) into an octree.\n"
               "The output is a compact maximum-likelihood binary octree file \n"
               "(.bt, bonsai tree) and general octree files (.ot) with the full\n"
               "information.\n\n";


  std::cerr << "OPTIONS:\n  -i <InputFile.graph> (required)\n"
            "  -o <OutputFile.bt> (required) \n"
            "  -res <resolution> (optional, default: 0.1 m)\n"
            "  -m <maxrange> (optional) \n"
            "  -n <max scan no.> (optional) \n"
            "  -log (enable a detailed log file with statistics) \n"
            "  -compress (enable lossless compression after every scan)\n"
            "  -compressML (enable maximum-likelihood compression (lossy) after every scan)\n"
            "  -clamping <p_min> <p_max> (override default sensor model clamping probabilities between 0..1)\n"
            "  -sensor <p_miss> <p_hit> (override default sensor model hit and miss probabilities between 0..1)"
  "\n";




  exit(0);
}

void outputStatistics(const OcTree* tree){
  unsigned int numThresholded, numOther;
  tree->calcNumThresholdedNodes(numThresholded, numOther);
  size_t memUsage = tree->memoryUsage();
  unsigned long long memFullGrid = tree->memoryFullGrid();
  size_t numLeafNodes = tree->getNumLeafNodes();

  cout << "Tree size: " << tree->size() <<" nodes (" << numLeafNodes<< " leafs). " <<numThresholded <<" nodes thresholded, "<< numOther << " other\n";
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
  double x, y, z;
  tree->getMetricSize(x, y, z);
  cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
  cout << endl;
}

int main(int argc, char** argv) {
  // default values:
  double res = 0.1;
  string graphFilename = "";
  string treeFilename = "";
  double maxrange = -1;
  int max_scan_no = -1;
  bool detailedLog = false;
  unsigned char compression = 0;

  // get default sensor model values:
  OcTree emptyTree(0.1);
  double clampingMin = emptyTree.getClampingThresMin();
  double clampingMax = emptyTree.getClampingThresMax();
  double probMiss = emptyTree.getProbMiss();
  double probHit = emptyTree.getProbHit();


  timeval start; 
  timeval stop; 

  int arg = 0;
  while (++arg < argc) {
    if (! strcmp(argv[arg], "-i"))
      graphFilename = std::string(argv[++arg]);
    else if (!strcmp(argv[arg], "-o"))
      treeFilename = std::string(argv[++arg]);
    else if (! strcmp(argv[arg], "-res") && argc-arg < 2)
      printUsage(argv[0]);
    else if (! strcmp(argv[arg], "-res"))
      res = atof(argv[++arg]);
    else if (! strcmp(argv[arg], "-log"))
      detailedLog = true;
    else if (! strcmp(argv[arg], "-compress"))
      compression = 1;
    else if (! strcmp(argv[arg], "-compressML"))
      compression = 2;
    else if (! strcmp(argv[arg], "-m"))
      maxrange = atof(argv[++arg]);
    else if (! strcmp(argv[arg], "-n"))
      max_scan_no = atoi(argv[++arg]);
    else if (! strcmp(argv[arg], "-clamping") && (argc-arg < 3))
      printUsage(argv[0]);
    else if (! strcmp(argv[arg], "-clamping")){
      clampingMin = atof(argv[++arg]);
      clampingMax = atof(argv[++arg]);
    }
    else if (! strcmp(argv[arg], "-sensor") && (argc-arg < 3))
      printUsage(argv[0]);
    else if (! strcmp(argv[arg], "-sensor")){
      probMiss = atof(argv[++arg]);
      probHit = atof(argv[++arg]);
    }
    else {
      printUsage(argv[0]);
    }
  }

  if (graphFilename == "" || treeFilename == "")
    printUsage(argv[0]);

  // verify input:
  if (res <= 0.0){
    OCTOMAP_ERROR("Resolution must be positive");
    exit(1);
  }

  if (clampingMin >= clampingMax || clampingMin < 0.0 || clampingMax > 1.0){
    OCTOMAP_ERROR("Error in clamping values:  0.0 <= [%f] < [%f] <= 1.0\n", clampingMin, clampingMax);
    exit(1);
  }

  if (probMiss >= probHit || probMiss < 0.0 || probHit > 1.0){
    OCTOMAP_ERROR("Error in sensor model (hit/miss prob.):  0.0 <= [%f] < [%f] <= 1.0\n", probMiss, probHit);
    exit(1);
  }


  std::string treeFilenameOT = treeFilename + ".ot";
  std::string treeFilenameMLOT = treeFilename + "_ml.ot";

  cout << "\nReading Graph file\n===========================\n";
  ScanGraph* graph = new ScanGraph();
  if (!graph->readBinary(graphFilename))
    exit(2);

  unsigned int num_points_in_graph = 0;
  if (max_scan_no > 0) {
    num_points_in_graph = graph->getNumPoints(max_scan_no-1);
    cout << "\n Data points in graph up to scan " << max_scan_no << ": " << num_points_in_graph << endl;
  }
  else {
    num_points_in_graph = graph->getNumPoints();
    cout << "\n Data points in graph: " << num_points_in_graph << endl;
  }
  std::ofstream logfile;
  if (detailedLog){
    logfile.open((treeFilename+".log").c_str());
    logfile << "# Memory of processing " << graphFilename << " over time\n";
    logfile << "# Resolution: "<< res <<"; compression: " << int(compression) << "; scan endpoints: "<< num_points_in_graph << std::endl;
    logfile << "# [scan number] [bytes octree] [bytes full 3D grid]\n";
  }

  cout << "\nCreating tree\n===========================\n";
  OcTree* tree = new OcTree(res);

  tree->setClampingThresMin(clampingMin);
  tree->setClampingThresMax(clampingMax);
  tree->setProbHit(probHit);
  tree->setProbMiss(probMiss);


  gettimeofday(&start, NULL);  // start timer
  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;
  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
    if (max_scan_no > 0) cout << "("<<currentScan << "/" << max_scan_no << ") " << flush;
    else cout << "("<<currentScan << "/" << numScans << ") " << flush;

    tree->insertScan(**scan_it, maxrange, (compression==1));
    if (compression == 2){
      tree->toMaxLikelihood();
      tree->prune();
    }

    if (detailedLog)
      logfile << currentScan << " " << tree->memoryUsage() << " " << tree->memoryFullGrid() << "\n";

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;
  }
  gettimeofday(&stop, NULL);  // stop timer
  
  double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);

  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;
  if (logfile.is_open())
    logfile.close();



  cout << "\nDone building tree.\n\n";
  cout << "time to insert scans: " << time_to_insert << " sec" << endl;
  cout << "time to insert 100.000 points took: " << time_to_insert/ ((double) num_points_in_graph / 100000) << " sec (avg)" << endl << endl;


  std::cout << "Full tree\n" << "===========================\n";
  outputStatistics(tree);

  std::cout << "Pruned tree (lossless compression)\n" << "===========================\n";
  tree->prune();
  outputStatistics(tree);

  tree->write(treeFilenameOT);

  std::cout << "Pruned max-likelihood tree (lossy compression)\n" << "===========================\n";
  tree->toMaxLikelihood();
  tree->prune();
  outputStatistics(tree);


  cout << "\nWriting tree files\n===========================\n";
  tree->write(treeFilenameMLOT);
  std::cout << "Full Octree (pruned) written to "<< treeFilenameOT << std::endl;
  std::cout << "Full Octree (max.likelihood, pruned) written to "<< treeFilenameMLOT << std::endl;
  tree->writeBinary(treeFilename);
  std::cout << "Bonsai tree written to "<< treeFilename << std::endl;
  cout << endl;

  delete tree;

  exit(0);
}
