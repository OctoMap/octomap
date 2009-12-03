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

#include "Pointcloud.h"
#include <ext/algorithm>
#include <list>
#include <fstream>

namespace octomap {


  Pointcloud::Pointcloud() {
  }

  Pointcloud::~Pointcloud() {
    this->clear();
  }

  void Pointcloud::clear() {

    // delete the points
    if (points.size()) {
      for (unsigned int i=0; i < points.size(); i++) {
	delete (points[i]);
	//  printf("-");
      }
      points.clear();
    }

  }


  Pointcloud::Pointcloud(const Pointcloud& other) {
    for (Pointcloud::const_iterator it = other.begin(); it != other.end(); it++) {
      points.push_back(new point3d(**it));
    }
  }

  Pointcloud::Pointcloud(Pointcloud* other) {
    for (Pointcloud::const_iterator it = other->begin(); it != other->end(); it++) {
      points.push_back(new point3d(**it));
    }
  }


  void Pointcloud::push_back(double x, double y, double z) {
    point3d* p = new point3d (x,y,z);
    points.push_back(p);
  }

  void Pointcloud::push_back(point3d* p) {
    point3d* np = new point3d (*p);
    points.push_back(np);
  }

  void Pointcloud::push_back(point3d& p) {
    point3d* np = new point3d (p);
    points.push_back(np);
  }

  void Pointcloud::push_back(const Pointcloud& other)   {
    for (Pointcloud::const_iterator it = other.begin(); it != other.end(); it++) {
      points.push_back(new point3d(**it));
    }
  }

  point3d* Pointcloud::getPoint(unsigned int i) {
    if (i<points.size()) return points[i];
    else {
      fprintf(stderr, "\nWARNING: Pointcloud::getPoint index out of range!\n\n");
      return NULL;
    }
  }

  void Pointcloud::transform(octomath::Pose6D transform) {

    for (unsigned int i=0; i<points.size(); i++) {
      *(points[i]) = transform.transform(*(points[i]));
    }

   // FIXME: not correct for multiple transforms
    current_inv_transform = transform.inv();
  }


  void Pointcloud::transformAbsolute(pose6d transform) {

    // undo previous transform, then apply current transform
    pose6d transf = current_inv_transform * transform;

    for (unsigned int i=0; i<points.size(); i++) {
      *(points[i]) = transf.transform(*(points[i]));
    }

    current_inv_transform = transform.inv();
  }


  void Pointcloud::rotate(double roll, double pitch, double yaw) {

    for (unsigned int i=0; i<points.size(); i++) {
      points[i]->rotate_IP(roll, pitch, yaw);
    }
  }


  void Pointcloud::calcBBX(point3d& lowerBound, point3d& upperBound) const {
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    min_x = min_y = min_z = 1e6;
    max_x = max_y = max_z = -1e6;

    double x,y,z;

    for (Pointcloud::const_iterator it=begin(); it!=end(); it++) {

      x = (**it)(0);
      y = (**it)(1);
      z = (**it)(2);

      if (x < min_x) min_x = x;
      if (y < min_y) min_y = y;
      if (z < min_z) min_z = z;

      if (x > max_x) max_x = x;
      if (y > max_y) max_y = y;
      if (z > max_z) max_z = z;
    }

    lowerBound(0) = min_x; lowerBound(1) = min_y; lowerBound(2) = min_z;
    upperBound(0) = max_x; upperBound(1) = max_y; upperBound(2) = max_z;
  }


  void Pointcloud::crop(point3d lowerBound, point3d upperBound) {

    Pointcloud result;

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    double x,y,z;

    min_x = lowerBound(0); min_y = lowerBound(1); min_z = lowerBound(2);
    max_x = upperBound(0); max_y = upperBound(1); max_z = upperBound(2);

    for (Pointcloud::const_iterator it=begin(); it!=end(); it++) {

      x = (**it)(0);
      y = (**it)(1);
      z = (**it)(2);

      if ( (x >= min_x) &&
	   (y >= min_y) &&
	   (z >= min_z) &&
	   (x <= max_x) &&
	   (y <= max_y) &&
	   (z <= max_z) ) {
	result.push_back (x,y,z);
      }
    } // end for points

    this->clear();
    this->push_back(result);

  }


  void Pointcloud::subSampleRandom(unsigned int num_samples, Pointcloud& sample_cloud) {

    point3d_collection samples;
    random_sample_n(begin(), end(), std::back_insert_iterator<point3d_collection>(samples), num_samples);
    for (unsigned int i=0; i<samples.size(); i++) {
      sample_cloud.push_back(*samples[i]);
    }
  }


  void Pointcloud::writeVrml(std::string filename){

    std::ofstream outfile (filename.c_str());

    outfile << "#VRML V2.0 utf8" << std::endl;
    outfile << "Transform {" << std::endl;
    outfile << "translation 0 0 0" << std::endl;
    outfile << "rotation 0 0 0 0" << std::endl;
    outfile << "  children [" << std::endl;
    outfile << "     Shape{" << std::endl;
    outfile << "  geometry PointSet {" << std::endl;
    outfile << "      coord Coordinate {" << std::endl;
    outfile << "          point [" << std::endl;

    std::cout << "PointCloud::writeVrml writing " 
	      << points.size() << " points to " 
	      << filename <<  ".\n";

    for (unsigned int i = 0; i < (points.size()); i++){
      outfile << "\t\t" << (*(points[i]))(0) 
	      << " " << (*(points[i]))(1) 
	      <<  " " << (*(points[i]))(2) 
	      << "\n";
    }

    outfile << "                 ]" << std::endl;
    outfile << "      }" << std::endl;
    outfile << "    color Color{" << std::endl;
    outfile << "              color [" << std::endl;

    for (unsigned int i = 0; i < points.size(); i++){
      outfile << "\t\t 1.0 1.0 1.0 \n";
    }

    outfile << "                 ]" << std::endl;
    outfile << "      }" << std::endl;

    outfile << "   }" << std::endl;
    outfile << "     }" << std::endl;


    outfile << "  ]" << std::endl;
    outfile << "}" << std::endl;


  }

  std::istream& Pointcloud::readBinary(std::istream &s) {

    unsigned int pc_size = 0;
    s.read((char*)&pc_size, sizeof(pc_size));
    fprintf(stderr, "reading %d points from binary file...", pc_size); fflush(stderr);

    if (pc_size > 0) {
      this->points.reserve(pc_size);
      point3d p;
      for (unsigned int i=0; i<pc_size; i++) {
	p.readBinary(s);
	if (!s.fail()) {
	  this->push_back(p);
	}
	else {
	  printf("Pointcloud::readBinary: ERROR.\n" );
	  break;
	}
      }
    }
    fprintf(stderr, "done.\n");

    return s;
  }


  std::ostream& Pointcloud::writeBinary(std::ostream &s) const {

    unsigned int pc_size = this->size();
    fprintf(stderr, "writing %d points to binary file...", pc_size); fflush(stderr);
    s.write((char*)&pc_size, sizeof(pc_size));

    for (Pointcloud::const_iterator it = this->begin(); it != this->end(); it++) {
      (*it)->writeBinary(s);
    }
    fprintf(stderr, "done.\n");

    return s;
  }

} // end namespace
