/**
* octomap_server: A ROS map server for the 3D Octomaps
* (inspired by the ROS map_server)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009.
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


/*
 * This file includes some common headers for octomaps in ROS, and provides
 * converstion between the octomap structure and ROS message format *
 *
 */

#ifndef OCTOMAP_SERVER_H_
#define OCTOMAP_SERVER_H_

#include <octomap/octomap.h>
#include <octomap_server/OctomapBinary.h>
#include <octomap_server/GetOctomap.h>
#include <iostream>


namespace octomap_server{

	/**
	 * Converts an octomap map structure to a ROS octomap msg as binary data
	 *
	 * @param octomap input OcTree
	 * @param mapMsg output msg
	 */
	static inline void octomapMapToMsg(const octomap::OcTree& octomap, OctomapBinary& mapMsg){
		// conversion via stringstream

		// TODO: read directly into buffer? see
		// http://stackoverflow.com/questions/132358/how-to-read-file-content-into-istringstream
		std::stringstream datastream;
		octomap.writeBinaryConst(datastream);
		std::string datastring = datastream.str();
		mapMsg.header.stamp = ros::Time::now();
		mapMsg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
	}

	/**
	 * Converts a ROS octomap msg (binary data) to an octomap map structure
	 *
	 * @param mapMsg
	 * @param octomap
	 */
	static inline void octomapMsgToMap(const OctomapBinary& mapMsg, octomap::OcTree& octomap){
		std::stringstream datastream;
		assert(mapMsg.data.size() > 0);
		datastream.write((const char*) &mapMsg.data[0], mapMsg.data.size());
		octomap.readBinary(datastream);
	}
}

#endif /* OCTOMAP_SERVER_H_ */
