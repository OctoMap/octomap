/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
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
  std::cerr << "USAGE: " << self << " <InputFile.graph>\n";
  std::cerr << "This tool is part of OctoMap and evaluates the statistical accuracy\n"
               "of an octree map from scan graph data (point clouds with poses).\n";

  std::cerr << "OPTIONS:\n"
            "  -res <resolution> (default: 0.1 m)\n"
            "  -m <maxrange> (optional) \n"
            "  -n <max scan no.> (optional) \n"
  "\n";

  exit(0);
}


int main(int argc, char** argv) {
  // default values:
  double res = 0.1;

  if (argc < 2)
    printUsage(argv[0]);

  string graphFilename = std::string(argv[1]);

  double maxrange = -1;
  int max_scan_no = -1;
  int skip_scan_eval = 5;

  int arg = 1;
  while (++arg < argc) {
    if (! strcmp(argv[arg], "-i"))
      graphFilename = std::string(argv[++arg]);
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

  cout << "\nCreating tree\n===========================\n";
  OcTree* tree = new OcTree(res);

  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;
  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {

    if (currentScan % skip_scan_eval != 0){
      if (max_scan_no > 0) cout << "("<<currentScan << "/" << max_scan_no << ") " << flush;
      else cout << "("<<currentScan << "/" << numScans << ") " << flush;
      tree->insertPointCloud(**scan_it, maxrange);
    } else
      cout << "(SKIP) " << flush;

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;
  }

  tree->expand();

  
  cout << "\nEvaluating scans\n===========================\n";
  currentScan = 1;
  unsigned num_points = 0;
  unsigned num_voxels_correct = 0;
  unsigned num_voxels_wrong = 0;
  unsigned num_voxels_unknown = 0;


  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {

    if (currentScan % skip_scan_eval == 0){
      if (max_scan_no > 0) cout << "("<<currentScan << "/" << max_scan_no << ") " << flush;
      else cout << "("<<currentScan << "/" << numScans << ") " << flush;


      pose6d frame_origin = (*scan_it)->pose;
      point3d sensor_origin = frame_origin.inv().transform((*scan_it)->pose.trans());

      // transform pointcloud:
      Pointcloud scan (*(*scan_it)->scan);
      scan.transform(frame_origin);
      point3d origin = frame_origin.transform(sensor_origin);

      KeySet free_cells, occupied_cells;
      tree->computeUpdate(scan, origin, free_cells, occupied_cells, maxrange);

      num_points += scan.size();

      // count free cells
      for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
        OcTreeNode* n = tree->search(*it);
        if (n){
          if (tree->isNodeOccupied(n))
            num_voxels_wrong++;
          else
            num_voxels_correct++;
        } else
          num_voxels_unknown++;
      } // count occupied cells
      for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
        OcTreeNode* n = tree->search(*it);
        if (n){
          if (tree->isNodeOccupied(n))
            num_voxels_correct++;
          else
            num_voxels_wrong++;
        } else
          num_voxels_unknown++;
      }


    }

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;


  }

  cout << "\nFinished evaluating " << num_points <<"/"<< num_points_in_graph << " points.\n"
      <<"Voxels correct: "<<num_voxels_correct<<" #wrong: " <<num_voxels_wrong << " #unknown: " <<num_voxels_unknown
      <<". % correct: "<< num_voxels_correct/double(num_voxels_correct+num_voxels_wrong)<<"\n\n";


  delete graph;
  delete tree;
  
  return 0;
}
