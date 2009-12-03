// $Id: graph2tree.cpp 22 2009-09-28 09:59:31Z ahornung $

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

using namespace std;
using namespace octomap;

int main(int argc, char** argv) {

  OcTree tree(0.1);
  tree.readBinary("test.bt");
  
  point3d origin(-1.0 ,4,1.0);
  point3d direction1(0, 1, 0);
  point3d direction2(-0.5, 1, 0);
  point3d direction3(-1.5, 0, 0);
  point3d direction4(1.0, 0, 0);

  point3d hit;
  if (tree.castRay(origin, direction1, hit)){
    std::cout << "Raycast1 hit "<<hit << std::endl;
  } else{
    std::cout << "Raycast1 no hit\n";
  }
  if (tree.castRay(origin, direction2, hit)){
    std::cout << "Raycast2 hit "<<hit << std::endl;
  } else{
    std::cout << "Raycast2 no hit\n";
  }
  if (tree.castRay(origin, direction3, hit)){
    std::cout << "Raycast3 hit "<<hit << std::endl;
  } else{
    std::cout << "Raycast3 no hit\n";
  }

  if (tree.castRay(origin, direction4, hit)){
    std::cout << "Raycast4 hit "<<hit << std::endl;
  } else{
    std::cout << "Raycast4 no hit\n";
  }


}
