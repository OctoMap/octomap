#ifndef MAPPING3D_POINTCLOUD_H
#define MAPPING3D_POINTCLOUD_H

// ==================================================
// octomap
// Kai M. Wurm <wurm@uni-freiburg.de>
// ==================================================

#include<vector>
#include<list>
#include <octomap_types.h>

namespace octomap {

  class Pointcloud {

  public:

    Pointcloud();
    ~Pointcloud();

    Pointcloud(const Pointcloud& other);
    Pointcloud(Pointcloud* other);

    uint size() const {  return points.size(); }
    void clear();

    // add beam endpoint
    void push_back(double x, double y, double z);
    void push_back(point3d& p);
    void push_back(point3d* p);

    // add beam endpoint and corresponding remission value
    void push_back(double x, double y, double z, double r);
    void push_back(point3d& p, double r);

    // add points from other Pointcloud
    void push_back(const Pointcloud& other);

    bool hasRemissions() const { return (remissions.size() > 0); }
    bool hasNormals() const { return has_normals; }

    bool hasBorder() const { return (on_border.size() > 0); }
    void setOnBorder(uint id);
    bool isOnBorder(uint id);


    void writeVrml(std::string filename, bool remvis = true);
    void writeVrml(std::string filename, std::vector<double>& probs);

    // apply transform to each point
    void transform(octomath::Pose6D transform);

    // rotate each point in pointcloud
    void rotate(double roll, double pitch, double yaw);

    // apply transform to each point, undo prev transform
    // FIXME: probably buggy...
    void transformAbsolute(octomath::Pose6D transform);

    void calcBBX(point3d& lowerBound, point3d& upperBound) const;
    void crop(point3d lowerBound, point3d upperBound);

    void subSampleRandom(uint num_samples, Pointcloud& sample_cloud);

    // iterators ------------------

    typedef point3d_collection::iterator iterator;
    typedef point3d_collection::const_iterator const_iterator;
    iterator begin() { return points.begin(); }
    iterator end()   { return points.end(); }
    const_iterator begin() const { return points.begin(); }
    const_iterator end() const  { return points.end(); }
    point3d* back()  { return points.back(); }
    point3d* getPoint(unsigned int i);   // may return NULL
    point3d* getNormal(unsigned int i);   // may return NULL

    typedef std::vector<double>::iterator rem_iterator;
    typedef std::vector<double>::const_iterator rem_const_iterator;
    rem_iterator begin_rem() { return remissions.begin(); }
    rem_iterator end_rem() { return remissions.end(); }
    rem_const_iterator begin_rem() const { return remissions.begin(); }
    rem_const_iterator end_rem() const { return remissions.end(); }

    typedef point3d_collection::iterator norm_iterator;
    typedef point3d_collection::const_iterator norm_const_iterator;
    norm_iterator begin_norm() { return normals.begin(); }
    norm_iterator end_norm() { return normals.end(); }
    norm_const_iterator begin_norm() const { return normals.begin(); }
    norm_const_iterator end_norm() const { return normals.end(); }

    typedef std::vector<bool>::const_iterator border_const_iterator;
    border_const_iterator begin_border() const { return on_border.begin(); }
    border_const_iterator end_border() const { return on_border.end(); }

    // I/O methods

    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;
    std::ostream& appendBinary(std::ostream& s) const;


  protected:

    point3d_collection   points;
    point3d_collection   normals;
    std::vector<double>  remissions;
    std::vector<bool>    on_border;

    octomath::Pose6D         current_inv_transform;

    bool                 has_remissions;
    bool                 has_normals;
  };

}


#endif
