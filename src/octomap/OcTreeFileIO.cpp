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

#include <OcTreeFileIO.h>

namespace octomap {

  OcTreeFileIO::OcTreeFileIO() {


  }

  OcTreeFileIO::~OcTreeFileIO() {

  }

  bool OcTreeFileIO::write(const OcTreeBase<NODE>& tree, const std::string& filename) const{
    std::ofstream stream(filename.c_str(), std::ios_base::binary);

    if (!stream.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return false;
    } else {
      // TODO: check is_good of finished stream
      write(tree, stream);
      stream.close();
    }
  }

  ostream OcTreeFileIO::write(const OcTreeBase<NODE>& tree, std::ostream &s){
    // write header:
    ofstream << "# Octomap OcTree file\n";

   return ostream;
  }

}
