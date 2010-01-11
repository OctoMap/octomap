/**
* octomap_saver: A Tool to save 3D Octomaps in ROS
* (inspired by the ROS map_saver)
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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_server/octomap_server.h>
#include <octomap/octomap.h>

#define USAGE "\nUSAGE: octomap_server <map.bt>\n" \
              "  map.bt: octomap 3D map file to read\n"

class OctomapServer{
public:
	OctomapServer(const std::string& filename);
	virtual ~OctomapServer();
	void readMap(const std::string& filename);
	bool serviceCallback(octomap_server::GetOctomap::Request  &req,
			octomap_server::GetOctomap::Response &res);

private:
	ros::NodeHandle m_nh;
	ros::Publisher m_markerPub, m_binaryMapPub;
	ros::ServiceServer m_service;

	// (cached) map data
	octomap_server::GetOctomap::Response m_mapResponse;
	// (cached) map visualization data:
	visualization_msgs::MarkerArray m_occupiedCellsVis;

	std::string m_frameId;
};


OctomapServer::OctomapServer(const std::string& filename)
  : m_nh(), m_frameId("/map")
{
	ros::NodeHandle private_nh("~");
	private_nh.param("frame_id", m_frameId, m_frameId);

	readMap(filename);

	m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
	m_binaryMapPub = m_nh.advertise<octomap_server::OctomapBinary>("octomap_binary", 1, true);
	m_service = m_nh.advertiseService("octomap_binary", &OctomapServer::serviceCallback, this);

	// publish once as latched topic:
	m_binaryMapPub.publish(m_mapResponse.map);
	m_markerPub.publish(m_occupiedCellsVis);
}

OctomapServer::~OctomapServer(){

}

/**
 * Reads in a map file and fills internal (cached) messages accordingly
 *
 * @param filename of map file
 */
void OctomapServer::readMap(const std::string& filename){

	octomap::OcTree map(filename);

	m_mapResponse.map.header.frame_id = m_frameId;
	octomap_server::octomapMapToMsg(map, m_mapResponse.map);

	// each array stores all cubes of a different size, one for each depth level:
	m_occupiedCellsVis.markers.resize(16);
	double lowestRes = map.getResolution();

	std::list<octomap::OcTreeVolume> occupiedCells;
	map.getOccupied(occupiedCells);

	// rough heuristics for expected size of cells at lowest level
	m_occupiedCellsVis.markers[0].points.reserve(occupiedCells.size());
	m_occupiedCellsVis.markers[1].points.reserve(occupiedCells.size()/2);
	m_occupiedCellsVis.markers[2].points.reserve(occupiedCells.size()/4);
	m_occupiedCellsVis.markers[3].points.reserve(occupiedCells.size()/4);


	std::list<octomap::OcTreeVolume>::iterator it;

	for (it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
		// which array to store cubes in?
		int idx = int(log2(it->second / lowestRes) +0.5);
		assert (idx >= 0 && unsigned(idx) < m_occupiedCellsVis.markers.size());
		geometry_msgs::Point cubeCenter;
		cubeCenter.x = it->first.x();
		cubeCenter.y = it->first.y();
		cubeCenter.z = it->first.z();

		m_occupiedCellsVis.markers[idx].points.push_back(cubeCenter);
	}

	for (unsigned i= 0; i < m_occupiedCellsVis.markers.size(); ++i){
		double size = lowestRes * pow(2,i);

		m_occupiedCellsVis.markers[i].header.frame_id = m_frameId;
		m_occupiedCellsVis.markers[i].header.stamp = ros::Time::now();
		m_occupiedCellsVis.markers[i].ns = "map";
		m_occupiedCellsVis.markers[i].id = i;
		m_occupiedCellsVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		m_occupiedCellsVis.markers[i].scale.x = size;
		m_occupiedCellsVis.markers[i].scale.y = size;
		m_occupiedCellsVis.markers[i].scale.z = size;
		m_occupiedCellsVis.markers[i].color.r = 1.0f;
		m_occupiedCellsVis.markers[i].color.g = 0.0f;
		m_occupiedCellsVis.markers[i].color.b = 0.0f;
		m_occupiedCellsVis.markers[i].color.a = 0.5f;

		if (m_occupiedCellsVis.markers[i].points.size() > 0)
			m_occupiedCellsVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
			m_occupiedCellsVis.markers[i].action = visualization_msgs::Marker::DELETE;
	}

	ROS_INFO("Octomap file %s loaded (%d nodes).", filename.c_str(),map.size());
}

bool OctomapServer::serviceCallback(octomap_server::GetOctomap::Request  &req,
			octomap_server::GetOctomap::Response &res)
{
	res = m_mapResponse;
	ROS_INFO("Sending map data on service request");

	return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server");
  std::string mapFilename("");
  if (argc == 2)
	  mapFilename = std::string(argv[1]);
  else{
	  ROS_ERROR("%s", USAGE);
	  exit(-1);
  }

  try{
	  OctomapServer ms(mapFilename);
	  ros::spin();
  }catch(std::runtime_error& e){
	  ROS_ERROR("map_server exception: %s", e.what());
	  return -1;
  }

  return 0;
}

