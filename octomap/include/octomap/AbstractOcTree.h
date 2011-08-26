#ifndef OCTOMAP_ABSTRACT_OCTREE
#define OCTOMAP_ABSTRACT_OCTREE

namespace octomap {
  
  /*!
   * This abstract class is an interface to all octrees
   */
  class AbstractOcTree {

  public:
    AbstractOcTree() {};
    virtual ~AbstractOcTree() {};

    virtual size_t size() const = 0;
    virtual size_t memoryUsage() const = 0;
    virtual void getMetricMin(double& x, double& y, double& z) = 0;
    virtual void getMetricMax(double& x, double& y, double& z) = 0;
    virtual void getMetricSize(double& x, double& y, double& z) = 0;

    virtual void prune() = 0;
    virtual void expand() = 0;


  protected:

  };


} // end namespace


#endif
