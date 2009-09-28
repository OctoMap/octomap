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


  Pointcloud::Pointcloud() : has_remissions(true), has_normals(false) {
    //    cout << COLOR_RED << "\nPointcloud created.\n" << COLOR_NORMAL;
  }

  Pointcloud::~Pointcloud() {
    this->clear();
    //    cout << COLOR_GREEN << "\nPointcloud deleted.\n" << COLOR_NORMAL;
  }

  void Pointcloud::clear() {

    // delete the points
    if (points.size()) {
      for (uint i=0; i < points.size(); i++) {
	delete (points[i]);
	//  printf("-");
      }
      points.clear();

      if (has_normals) {
	for (uint i=0; i < normals.size(); i++) {
	  delete (normals[i]);
	  //  printf("-");
	}
	normals.clear();
      }
    }

  }


  Pointcloud::Pointcloud(const Pointcloud& other) {
    for (Pointcloud::const_iterator it = other.begin(); it != other.end(); it++) {
      points.push_back(new point3d(**it));
    }
    if (other.hasRemissions()) {
      for (Pointcloud::rem_const_iterator rit=other.begin_rem(); rit != other.end_rem(); rit++) {
	remissions.push_back(*rit);
      }
      this->has_remissions = true;
    }
    if (other.hasNormals()) {
      for (Pointcloud::norm_const_iterator nit=other.begin_norm(); nit != other.end_norm(); nit++) {
	normals.push_back(new point3d(**nit));
      }
      this->has_normals = true;
    }
    if (other.hasBorder()) {
      for (border_const_iterator it = other.begin_border(); it != other.end_border(); it++) {
	on_border.push_back(*it);
      }
    }

  }

  Pointcloud::Pointcloud(Pointcloud* other) {
    for (Pointcloud::const_iterator it = other->begin(); it != other->end(); it++) {
      points.push_back(new point3d(**it));
    }
    if (other->hasRemissions()) {
      for (Pointcloud::rem_const_iterator rit=other->begin_rem(); rit != other->end_rem(); rit++) {
	remissions.push_back(*rit);
      }
      this->has_remissions = true;
    }
    if (other->hasNormals()) {
      for (Pointcloud::norm_const_iterator nit=other->begin_norm(); nit != other->end_norm(); nit++) {
	normals.push_back(new point3d(**nit));
      }
      this->has_normals = true;
    }
    if (other->hasBorder()) {
      for (border_const_iterator it = other->begin_border(); it != other->end_border(); it++) {
	on_border.push_back(*it);
      }
    }
  }


  // ---
  // WARNING: if you use the methods below, the kdtree
  // (if it should exist) will be inconsistent.
  // call to deleteKDTree(); omitted for performance

  void Pointcloud::push_back(double x, double y, double z) {
    point3d* p = new point3d (x,y,z);
    has_remissions = false;
    points.push_back(p);
  }

  void Pointcloud::push_back(point3d* p) {
    point3d* np = new point3d (*p);
    has_remissions = false;
    points.push_back(np);
  }

  void Pointcloud::push_back(point3d& p) {
    point3d* np = new point3d (p);
    has_remissions = false;
    points.push_back(np);
  }

  void Pointcloud::push_back(point3d& p, double r) {
    point3d* np = new point3d (p);
    points.push_back(np);
    remissions.push_back(r);
  }


  void Pointcloud::push_back(double x, double y, double z, double r) {

    point3d* p = new point3d(x,y,z);

    points.push_back(p);
    if (has_remissions) {
      remissions.push_back(r);
    }
    else {
      fprintf(stderr, "ERROR: range and remission inconsistent\n");
    }
  }
  // ---

  void Pointcloud::push_back(const Pointcloud& other)   {

    for (Pointcloud::const_iterator it = other.begin(); it != other.end(); it++) {
      points.push_back(new point3d(**it));
    }

    if (other.hasRemissions() && this->hasRemissions()) {
      for (Pointcloud::rem_const_iterator rit=other.begin_rem(); rit != other.end_rem(); rit++) {
	remissions.push_back(*rit);
      }
      this->has_remissions = true;
    }
    else {
      this->has_remissions = false;
    }

    if (other.hasNormals() && this->hasNormals()) {
      for (Pointcloud::norm_const_iterator nit=other.begin_norm(); nit != other.end_norm(); nit++) {
	normals.push_back(new point3d(**nit));
      }
      this->has_normals = true;
    }
    else {
      this->has_normals = false;
    }

  }


  point3d* Pointcloud::getPoint(unsigned int i) {
    if (i<points.size()) return points[i];
    else {
      fprintf(stderr, "\nWARNING: Pointcloud::getPoint index out of range!\n\n");
      return NULL;
    }
  }

  point3d* Pointcloud::getNormal(unsigned int i) {
    if (i<normals.size()) return normals[i];
    else {
      fprintf(stderr, "\nWARNING: Pointcloud::getNormal index out of range!\n\n");
      return NULL;
    }
  }


  void Pointcloud::transform(octomath::Pose6D transform) {

    for (uint i=0; i<points.size(); i++) {
      *(points[i]) = transform.transform(*(points[i]));
    }

    // transform normals
    if (has_normals) {
      for (uint i=0; i<normals.size(); i++) {
	*(normals[i]) = transform.transform(*(normals[i]));
      }
    }

   // FIXME: not correct for multiple transforms
    current_inv_transform = transform.inv();
  }


  void Pointcloud::transformAbsolute(octomath::Pose6D transform) {

    // undo previous transform, then apply current transform
    octomath::Pose6D transf = current_inv_transform * transform;

    for (uint i=0; i<points.size(); i++) {
      *(points[i]) = transf.transform(*(points[i]));
    }

    // transform normals
    if (has_normals) {
      for (uint i=0; i<normals.size(); i++) {
	*(normals[i]) = transform.transform(*(normals[i]));
      }
    }

    current_inv_transform = transform.inv();
  }


  void Pointcloud::rotate(double roll, double pitch, double yaw) {

    for (uint i=0; i<points.size(); i++) {
      points[i]->rotate_IP(roll, pitch, yaw);
    }

    // transform normals
    if (has_normals) {
      for (uint i=0; i<normals.size(); i++) {
	normals[i]->rotate_IP(roll, pitch, yaw);
      }
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

    if (hasRemissions()) {
      Pointcloud::rem_const_iterator rit = begin_rem();
      for (Pointcloud::const_iterator it=begin(); (it!=end() && rit != end_rem()); it++, rit++) {

	x = (**it)(0);
	y = (**it)(1);
	z = (**it)(2);

	if ( (x >= min_x) &&
	     (y >= min_y) &&
	     (z >= min_z) &&
	     (x <= max_x) &&
	     (y <= max_y) &&
	     (z <= max_z) ) {
	  result.push_back (x,y,z, *rit);
	}
      } // end for points
    } // end if has rem
    else {
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
    }

    this->clear();
    this->push_back(result);

  }


  void Pointcloud::subSampleRandom(uint num_samples, Pointcloud& sample_cloud) {

    point3d_collection samples;
    random_sample_n(begin(), end(), std::back_insert_iterator<point3d_collection>(samples), num_samples);
    for (uint i=0; i<samples.size(); i++) {
      sample_cloud.push_back(*samples[i]);
    }
  }


  void Pointcloud::writeVrml(std::string filename, bool remvis){

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

    std::cout << "PointCloud::writeVrml writing " << points.size() << " points to " << filename <<  ".\n";

    for (uint i = 0; i < (points.size()); i++){
      outfile << "\t\t" << (*(points[i]))(0) << " " << (*(points[i]))(1) <<  " " << (*(points[i]))(2)  << "\n";
    }

    outfile << "                 ]" << std::endl;
    outfile << "      }" << std::endl;
    outfile << "    color Color{" << std::endl;
    outfile << "              color [" << std::endl;


    if (has_remissions && remvis) {
      double r;
      for (uint i = 0; i < (remissions.size()); i++){
	r =  (remissions[i] - 9500.)/1000.;
	outfile << "\t\t" << r << " " << r << " " << r << "\n";
      }
    }
    else {
      for (uint i = 0; i < points.size(); i++){
	outfile << "\t\t 1.0 1.0 1.0 \n";
      }
    }


    outfile << "                 ]" << std::endl;
    outfile << "      }" << std::endl;

    outfile << "   }" << std::endl;
    outfile << "     }" << std::endl;

    if (has_normals) {

      //      std::cout << "writing " << normals.size() << " normals.\n";
      outfile << "Shape {\n";
      outfile << "  appearance Appearance{\n";
      outfile << "    material Material{\n";
      outfile << "      diffuseColor 0.7 0.7 0.7\n";
      outfile << "    }\n";
      outfile << "  }\n";
      outfile << "  geometry IndexedLineSet {\n";
      outfile << "    coord Coordinate {\n";
      outfile << "      point [\n";

      for (uint i = 0; i < (points.size()); i++){
	outfile << "\t\t" << (*(points[i]))(0) << " " << (*(points[i]))(1) <<  " " << (*(points[i]))(2)  << "\n";
	outfile << "\t\t" << (*(points[i]))(0) + (*(normals[i]))(0)/5.
		<< " " << (*(points[i]))(1)  + (*(normals[i]))(1)/5.
		<<  " " << (*(points[i]))(2) + (*(normals[i]))(2)/5. << "\n";
      }
      outfile << "      ]\n";
      outfile << "    }\n";
      outfile << "    coordIndex [\n";

      for (uint i = 0; i < (points.size()); i++){
	outfile << 2*i << " " << (2*i)+1 << " -1" << std::endl;
      }
      outfile << "    ]\n";

      outfile << "  }\n";
      outfile << "}\n";

    } // end if normals


    outfile << "  ]" << std::endl;
    outfile << "}" << std::endl;


  }


  void Pointcloud::writeVrml(std::string filename, std::vector<double>& probs) {


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

    std::cout << "PointCloud::writeVrml writing " << points.size() << " points to " << filename <<  ".\n";

    for (uint i = 0; i < (points.size()); i++){
      outfile << "\t\t" << (*(points[i]))(0) << " " << (*(points[i]))(1) <<  " " << (*(points[i]))(2)  << "\n";
    }

    outfile << "                 ]" << std::endl;
    outfile << "      }" << std::endl;
    outfile << "    color Color{" << std::endl;
    outfile << "              color [" << std::endl;

    for (uint i = 0; i < (probs.size()); i++){
      if (probs[i] > 0.5) {
	outfile << "\t\t" << 0. << " " << 1. << " " << 0. << "\n";
      }
      else {
	outfile << "\t\t" << 0. << " " << 0. << " " << 1.0 << "\n";
      }
    }



    outfile << "                 ]" << std::endl;
    outfile << "      }" << std::endl;

    outfile << "   }" << std::endl;
    outfile << "     }" << std::endl;


    outfile << "  ]" << std::endl;
    outfile << "}" << std::endl;

  }

  std::istream& Pointcloud::readBinary(std::istream &s) {

    uint pc_size = 0;
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

    uint pc_size = this->size();
    fprintf(stderr, "writing %d points to binary file...", pc_size); fflush(stderr);
    s.write((char*)&pc_size, sizeof(pc_size));

    for (Pointcloud::const_iterator it = this->begin(); it != this->end(); it++) {
      (*it)->writeBinary(s);
    }
    fprintf(stderr, "done.\n");

    return s;
  }

} // end namespace
