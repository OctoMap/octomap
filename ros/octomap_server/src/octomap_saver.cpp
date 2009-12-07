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

#include <ros/ros.h>
#include <octomap_server/GetOctomap.h>
#include <octomap_server/octomap_server.h>
#include <octomap/octomap.h>
#include <fstream>

#define USAGE "\nUSAGE: octomap_saver <map.bt>\n" \
              "  map.bt: filename of map to be saved\n"

using namespace std;
 
/**
 * @brief Map generation node.
 */
class MapSaver{
  public:
    MapSaver(const std::string& mapname){
      ros::NodeHandle n;
      const static std::string servname = "octomap_binary";
      ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
      octomap_server::GetOctomap::Request req;
      octomap_server::GetOctomap::Response resp;
      while(n.ok() && !ros::service::call(servname, req, resp))
      {
        ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
        usleep(1000000);
      }

      if (n.ok()){ // skip when CTRL-C
		  ROS_INFO("Map received, saving to %s", mapname.c_str());
		  ofstream mapfile(mapname.c_str(), ios_base::binary);

		  if (!mapfile.is_open()){
			  ROS_ERROR("Could not open file %s for writing", mapname.c_str());
		  } else {
			  // test conversion:
//			  octomap::OcTree octomap(0.1);
//			  octomap_server::octomapMsgToMap(resp.map, octomap);
//			  octomap.writeBinary(mapname);

			  // write out stream directly
			  mapfile.write((char*)&resp.map.data[0], resp.map.get_data_size());
			  mapfile.close();
		  }
      }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_saver");
  std::string mapFilename("");
  if (argc == 2)
	  mapFilename = std::string(argv[1]);
  else{
	  ROS_ERROR("%s", USAGE);
	  exit(-1);
  }

  try{
	  MapSaver ms(mapFilename);
  }catch(std::runtime_error& e){
	  ROS_ERROR("map_saver exception: %s", e.what());
	  return -1;
  }

  return 0;
}


