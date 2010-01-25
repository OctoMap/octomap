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
  std::cerr << "\nUSAGE: " << self << " InputFile.log OutputFile.graph\n\n";

  std::cerr << "This tool converts a plain text log file into a binary scangraph file" << std::endl;
  std::cerr << "which can be used in Octomap.\n\n";
  std::cerr << "The log file needs to be in the format of:\n"
      << "NODE x y z roll pitch yaw\n"
      << "x y z\nx y z\n...\n"
      << "NODE x y z roll pitch yaw\n"
      << "x y z\n...\n\n"
      << "Lines starting with '#' or empty lines are ignored.\n\n";

  exit(0);
}

int main(int argc, char** argv) {
  // default values:
  string logFilename = "";
  string graphFilename = "";


  if (argc != 3){
    printUsage(argv[0]);
  } else{
    logFilename = std::string(argv[1]);
    graphFilename = std::string(argv[2]);
  }

  cout << "\nReading Log file\n===========================\n";
  ScanGraph* graph = new ScanGraph();
  graph->readPlainASCII(logFilename);

  cout << "\nWriting binary graph file\n===========================\n";

  graph->writeBinary(graphFilename);
}
