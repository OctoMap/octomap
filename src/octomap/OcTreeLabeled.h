#ifndef OCTOMAP_OCTREE_LABELED_H
#define OCTOMAP_OCTREE_LABELED_H

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

#include "OccupancyOcTreeBase.h"
#include "OcTreeNodeLabeled.h"

namespace octomap {

  class OcTreeLabeled : public OccupancyOcTreeBase <OcTreeNodeLabeled> {

  public:
    static const int TREETYPE=4;

  public:

    OcTreeLabeled(double _resolution);
    virtual ~OcTreeLabeled();

    void toMaxLikelihood();

    void getChangedFreespace(std::list<OcTreeVolume>& nodes) const;

    void writeBinary(std::string filename); // TODO


  protected:

    void getChangedFreespaceRecurs(OcTreeNodeLabeled* node, unsigned int depth, unsigned int max_depth, 
                                   const point3d& parent_center, std::list<OcTreeVolume>& nodes) const;


    void toMaxLikelihoodRecurs(OcTreeNodeLabeled* node, unsigned int depth, unsigned int max_depth);

  };


} // end namespace

#endif
