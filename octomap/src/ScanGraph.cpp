/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <octomap/math/Pose6D.h>
#include <octomap/ScanGraph.h>

namespace octomap {


  ScanNode::~ScanNode(){
    if (scan != NULL){
      delete scan;
      scan = NULL;
    }
  }

  std::ostream& ScanNode::writeBinary(std::ostream &s) const {

    // file structure:    pointcloud | pose | id

    scan->writeBinary(s);
    pose.writeBinary(s);
    s.write((char*)&id, sizeof(id));

    return s;
  }

  std::istream& ScanNode::readBinary(std::istream &s) {

    this->scan = new Pointcloud();
    this->scan->readBinary(s);

    this->pose.readBinary(s);

    s.read((char*)&this->id, sizeof(this->id));

    return s;
  }


  std::ostream& ScanNode::writePoseASCII(std::ostream &s) const {
    s << " " << this->id;  // export pose for human editor
    s << " ";
    this->pose.trans().write(s);
    s << " ";
    this->pose.rot().toEuler().write(s);
    s << std::endl;
    return s;
  }

  std::istream& ScanNode::readPoseASCII(std::istream &s) {
    unsigned int read_id;
    s >> read_id;
    if (read_id != this->id)
      OCTOMAP_ERROR("ERROR while reading ScanNode pose from ASCII. id %d does not match real id %d.\n", read_id, this->id);

    this->pose.trans().read(s);

    // read rotation from euler angles
    point3d rot;
    rot.read(s);
    this->pose.rot() = octomath::Quaternion(rot);
    return s;
  }


  std::ostream& ScanEdge::writeBinary(std::ostream &s) const {

    // file structure:    first_id | second_id | constraint | weight

    s.write((char*)&first->id, sizeof(first->id));
    s.write((char*)&second->id, sizeof(second->id));
    constraint.writeBinary(s);
    s.write((char*)&weight, sizeof(weight));
    return s;
  }

  std::istream& ScanEdge::readBinary(std::istream &s, ScanGraph& graph) {
    unsigned int first_id, second_id;
    s.read((char*)&first_id, sizeof(first_id));
    s.read((char*)&second_id, sizeof(second_id));

    this->first = graph.getNodeByID(first_id);
    if (this->first == NULL) OCTOMAP_ERROR("ERROR while reading ScanEdge. first node not found.\n");
    this->second = graph.getNodeByID(second_id);
    if (this->second == NULL) OCTOMAP_ERROR("ERROR while reading ScanEdge. second node not found.\n");

    this->constraint.readBinary(s);
    s.read((char*)&weight, sizeof(weight));

    return s;
  }


  std::ostream& ScanEdge::writeASCII(std::ostream &s) const {

    // file structure:    first_id | second_id | constraint | weight

    s << " " << first->id << " " << second->id;
    s << " ";
    constraint.write(s);
    s << " " << weight;
    s << std::endl;
    return s;
  }

  std::istream& ScanEdge::readASCII(std::istream &s, ScanGraph& graph) {

    unsigned int first_id, second_id;
    s >> first_id;
    s >> second_id;

    this->first = graph.getNodeByID(first_id);
    if (this->first == NULL) OCTOMAP_ERROR("ERROR while reading ScanEdge. first node %d not found.\n", first_id);
    this->second = graph.getNodeByID(second_id);
    if (this->second == NULL) OCTOMAP_ERROR("ERROR while reading ScanEdge. second node %d not found.\n", second_id);

    this->constraint.read(s);
    s >> weight;
    return s;
  }


  ScanGraph::~ScanGraph() {
    this->clear();
  }

  void ScanGraph::clear() {
    for (unsigned int i=0; i<nodes.size(); i++) {
      delete nodes[i];
    }
    nodes.clear();
    for (unsigned int i=0; i<edges.size(); i++) {
      delete edges[i];
    }
    edges.clear();
  }


  ScanNode* ScanGraph::addNode(Pointcloud* scan, pose6d pose) {
    if (scan != 0) {
      nodes.push_back(new ScanNode(scan, pose, nodes.size()));
      return nodes.back();
    }
    else {
      OCTOMAP_ERROR("scan is invalid.\n");
      return NULL;
    }
  }


  ScanEdge* ScanGraph::addEdge(ScanNode* first, ScanNode*  second, pose6d constraint) {

    if ((first != 0) && (second != 0)) {
      edges.push_back(new ScanEdge(first, second, constraint));
      //      OCTOMAP_DEBUG("ScanGraph::AddEdge %d --> %d\n", first->id, second->id);
      return edges.back();
    }
    else {
      OCTOMAP_ERROR("addEdge:: one or both nodes invalid.\n");
      return NULL;
    }
  }


  ScanEdge* ScanGraph::addEdge(unsigned int first_id, unsigned int second_id) {

    if ( this->edgeExists(first_id, second_id)) {
      OCTOMAP_ERROR("addEdge:: Edge exists!\n");
      return NULL;
    }

    ScanNode* first = getNodeByID(first_id);
    ScanNode* second = getNodeByID(second_id);

    if ((first != 0) && (second != 0)) {
      pose6d constr = first->pose.inv() * second->pose;
      return this->addEdge(first, second, constr);
    }
    else {
      OCTOMAP_ERROR("addEdge:: one or both scans invalid.\n");
      return NULL;
    }
  }


  void ScanGraph::connectPrevious() {
    if (nodes.size() >= 2) {
      ScanNode* first =  nodes[nodes.size()-2];
      ScanNode* second = nodes[nodes.size()-1];
      pose6d c =  (first->pose).inv() * second->pose;
      this->addEdge(first, second, c);
    }
  }


  void ScanGraph::exportDot(std::string filename) {
    std::ofstream outfile (filename.c_str());
    outfile << "graph ScanGraph" << std::endl;
    outfile << "{" << std::endl;
    for (unsigned int i=0; i<edges.size(); i++) {
      outfile << (edges[i]->first)->id
	      << " -- "
	      << (edges[i]->second)->id
	      << " [label="
	      << std::fixed << std::setprecision(2) << edges[i]->constraint.transLength()
	      << "]" << std::endl;
    }
    outfile << "}" << std::endl;
    outfile.close();
  }

  ScanNode* ScanGraph::getNodeByID(unsigned int id) {
    for (unsigned int i = 0; i < nodes.size(); i++) {
      if (nodes[i]->id == id) return nodes[i];
    }
    return NULL;
  }

  bool ScanGraph::edgeExists(unsigned int first_id, unsigned int second_id) {

    for (unsigned int i=0; i<edges.size(); i++) {
      if (
              (((edges[i]->first)->id == first_id) && ((edges[i]->second)->id == second_id))
              ||
              (((edges[i]->first)->id == second_id) && ((edges[i]->second)->id == first_id))) {
	return true;
      }
    }
    return false;
  }

  std::vector<unsigned int> ScanGraph::getNeighborIDs(unsigned int id) {
    std::vector<unsigned int> res;
    ScanNode* node = getNodeByID(id);
    if (node) {
      // check all nodes
      for (unsigned int i = 0; i < nodes.size(); i++) {
	if (node->id == nodes[i]->id) continue;
	if (edgeExists(id, nodes[i]->id)) {
	  res.push_back(nodes[i]->id);
	}
      }
    }
    return res;
  }

  std::vector<ScanEdge*> ScanGraph::getOutEdges(ScanNode* node) {
    std::vector<ScanEdge*> res;
    if (node) {
      for (std::vector<ScanEdge*>::iterator it = edges.begin(); it != edges.end(); it++) {
	if ((*it)->first == node) {
	  res.push_back(*it);
	}
      }
    }
    return res;
  }

  std::vector<ScanEdge*> ScanGraph::getInEdges(ScanNode* node) {
    std::vector<ScanEdge*> res;
    if (node) {
      for (std::vector<ScanEdge*>::iterator it = edges.begin(); it != edges.end(); it++) {
	if ((*it)->second == node) {
	  res.push_back(*it);
	}
      }
    }
    return res;
  }

  void ScanGraph::transformScans() {
    for(ScanGraph::iterator it=this->begin(); it != this->end(); it++) {
      ((*it)->scan)->transformAbsolute((*it)->pose);
    }
  }

  bool ScanGraph::writeBinary(const std::string& filename) const {
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);
    if (!binary_outfile.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
      return false;
    }    
    writeBinary(binary_outfile);
    binary_outfile.close();
    return true;
  }

  std::ostream& ScanGraph::writeBinary(std::ostream &s) const {

    // file structure:    n | node_1 | ... | node_n | m | edge_1 | ... | edge_m

    // write nodes  ---------------------------------
    unsigned int graph_size = this->size();
    if (graph_size) OCTOMAP_DEBUG("writing %d nodes to binary file...\n", graph_size);
    s.write((char*)&graph_size, sizeof(graph_size));

    for (ScanGraph::const_iterator it = this->begin(); it != this->end(); it++) {
      (*it)->writeBinary(s);
    }

    if (graph_size) OCTOMAP_DEBUG("done.\n");

    // write edges  ---------------------------------
    unsigned int num_edges = this->edges.size();
    if (num_edges) OCTOMAP_DEBUG("writing %d edges to binary file...\n", num_edges);
    s.write((char*)&num_edges, sizeof(num_edges));

    for (ScanGraph::const_edge_iterator it = this->edges_begin(); it != this->edges_end(); it++) {
      (*it)->writeBinary(s);
    }

    if (num_edges) OCTOMAP_DEBUG(" done.\n");

    return s;
  }

  bool ScanGraph::readBinary(const std::string& filename) {
    std::ifstream binary_infile(filename.c_str(), std::ios_base::binary);
    if (!binary_infile.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing read.");
      return false;
    }
    readBinary(binary_infile);
    binary_infile.close();
    return true;
  }

  std::istream& ScanGraph::readBinary(std::ifstream &s) {
    if (!s.is_open()){
      OCTOMAP_ERROR_STR("Could not read from input filestream in ScanGraph::readBinary");
      return s;
    } else if (!s.good()){
      OCTOMAP_WARNING_STR("Input filestream not \"good\" in ScanGraph::readBinary");
    }
    this->clear();

    // read nodes  ---------------------------------
    unsigned int graph_size = 0;
    s.read((char*)&graph_size, sizeof(graph_size));
    if (graph_size) OCTOMAP_DEBUG("reading %d nodes from binary file...\n", graph_size);

    if (graph_size > 0) {
      this->nodes.reserve(graph_size);

      for (unsigned int i=0; i<graph_size; i++) {

        ScanNode* node = new ScanNode();
        node->readBinary(s);
        if (!s.fail()) {
          this->nodes.push_back(node);
        }
        else {
          OCTOMAP_ERROR("ScanGraph::readBinary: ERROR.\n" );
          break;
        }
      }
    }
    if (graph_size) OCTOMAP_DEBUG("done.\n");

    // read edges  ---------------------------------
    unsigned int num_edges = 0;
    s.read((char*)&num_edges, sizeof(num_edges));
    if (num_edges) OCTOMAP_DEBUG("reading %d edges from binary file...\n", num_edges);

    if (num_edges > 0) {
      this->edges.reserve(num_edges);

      for (unsigned int i=0; i<num_edges; i++) {

        ScanEdge* edge = new ScanEdge();
        edge->readBinary(s, *this);
        if (!s.fail()) {
          this->edges.push_back(edge);
        }
        else {
          OCTOMAP_ERROR("ScanGraph::readBinary: ERROR.\n" );
          break;
        }
      }
    }
    if (num_edges) OCTOMAP_DEBUG("done.\n");
    return s;
  }

  void ScanGraph::readPlainASCII(const std::string& filename){
    std::ifstream infile(filename.c_str());
    if (!infile.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing read.");
      return;
    }
    readPlainASCII(infile);
    infile.close();
  }

  std::istream& ScanGraph::readPlainASCII(std::istream& s){
    std::string currentLine;
    ScanNode* currentNode = NULL;
    while (true){
      getline(s, currentLine);
      if (s.good() && !s.eof()){
        std::stringstream ss;
        ss << currentLine;
        // skip empty and comment lines:
        if (currentLine.size() == 0
            || (currentLine.compare(0,1, "#") == 0)
            || (currentLine.compare(0,1, " ") == 0)){

          continue;
        } else if(currentLine.compare(0,4,"NODE")==0){
          if (currentNode){
            this->nodes.push_back(currentNode);
            this->connectPrevious();
            OCTOMAP_DEBUG_STR("ScanNode "<< currentNode->pose << " done, size: "<< currentNode->scan->size());
          }

          currentNode = new ScanNode();
          currentNode->scan = new Pointcloud();

          float x, y, z, roll, pitch, yaw;
          std::string tmp;
          ss >> tmp >> x >> y >> z >> roll >> pitch >> yaw;
          pose6d pose(x, y, z, roll, pitch, yaw);
          //std::cout << "Pose "<< pose << " found.\n";
          currentNode->pose = pose;
        } else{
          if (currentNode == NULL){
            // TODO: allow "simple" pc files by setting initial Scan Pose to (0,0,0)
            OCTOMAP_ERROR_STR("Error parsing log file, no Scan to add point to!");
            break;
          }
          float x, y, z;
          ss >> x >> y >> z;

          //std::cout << "Point "<< x << "," <<y <<"," <<z << " found.\n";
          currentNode->scan->push_back(x,y,z);
        }
      } else{
        if (currentNode){
          this->nodes.push_back(currentNode);
          this->connectPrevious();
          OCTOMAP_DEBUG_STR("Final ScanNode "<< currentNode->pose << " done, size: "<< currentNode->scan->size());
        }
        break;
      }
    }

    return s;
  }

  std::ostream& ScanGraph::writeEdgesASCII(std::ostream &s) const {

    // file structure:    n | edge_1 | ... | edge_n

    OCTOMAP_DEBUG_STR("Writing " << this->edges.size() << " edges to ASCII file...");

    s <<  " " << this->edges.size();
    s << std::endl;

    for (ScanGraph::const_edge_iterator it = this->edges_begin(); it != this->edges_end(); it++) {
      (*it)->writeASCII(s);
    }
    s << std::endl;
    OCTOMAP_DEBUG_STR("Done.");

    return s;
  }


  std::istream& ScanGraph::readEdgesASCII(std::istream &s) {

    unsigned int num_edges = 0;
    s >> num_edges;
    OCTOMAP_DEBUG("Reading %d edges from ASCII file...\n", num_edges);

    if (num_edges > 0) {

      for (unsigned int i=0; i<this->edges.size(); i++) delete edges[i];
      this->edges.clear();

      this->edges.reserve(num_edges);

      for (unsigned int i=0; i<num_edges; i++) {

        ScanEdge* edge = new ScanEdge();
        edge->readASCII(s, *this);
        if (!s.fail()) {
          this->edges.push_back(edge);
        }
        else {
          OCTOMAP_ERROR("ScanGraph::readBinary: ERROR.\n" );
          break;
        }
      }
    }

    OCTOMAP_DEBUG("done.\n");

    return s;
  }


  std::ostream& ScanGraph::writeNodePosesASCII(std::ostream &s) const {

    OCTOMAP_DEBUG("Writing %d node poses to ASCII file...\n", this->size());

    for (ScanGraph::const_iterator it = this->begin(); it != this->end(); it++) {
      (*it)->writePoseASCII(s);
    }
    s << std::endl;
    OCTOMAP_DEBUG("done.\n");

    return s;
  }

  std::istream& ScanGraph::readNodePosesASCII(std::istream &s) {

    for (ScanGraph::const_iterator it = this->begin(); it != this->end(); it++) {
      (*it)->readPoseASCII(s);
    }

    for (ScanGraph::edge_iterator it = this->edges_begin(); it != this->edges_end(); it++) {
      ScanNode* first =  (*it)->first;
      ScanNode* second = (*it)->second;
      (*it)->constraint = (first->pose).inv() * second->pose;
    }

    // constraints and nodes are inconsistent, rewire graph
//     for (unsigned int i=0; i<this->edges.size(); i++) delete edges[i];
//     this->edges.clear();


//     ScanGraph::iterator first_it = this->begin();
//     ScanGraph::iterator second_it = first_it+1;

//     for ( ; second_it != this->end(); first_it++, second_it++) {
//       ScanNode* first = (*first_it);
//       ScanNode* second = (*second_it);
//       octomath::Pose6D c =  (first->pose).inv() * second->pose;
//       this->addEdge(first, second, c);
//     }


    return s;
  }


  void ScanGraph::cropEachScan(point3d lowerBound, point3d upperBound) {

    for (ScanGraph::iterator it = this->begin(); it != this->end(); it++) {
      ((*it)->scan)->crop(lowerBound, upperBound);
    }
  }


  void ScanGraph::crop(point3d lowerBound, point3d upperBound) {


    // for all node in graph...
    for (ScanGraph::iterator it = this->begin(); it != this->end(); it++) {
      pose6d scan_pose = (*it)->pose;
      Pointcloud* pc = new Pointcloud((*it)->scan);
      pc->transformAbsolute(scan_pose);
      pc->crop(lowerBound, upperBound);
      pc->transform(scan_pose.inv());
      delete (*it)->scan;
      (*it)->scan = pc;
    }
  }

  unsigned int ScanGraph::getNumPoints(unsigned int max_id) const {
    unsigned int retval = 0;
    
    for (ScanGraph::const_iterator it = this->begin(); it != this->end(); it++) {
      retval += (*it)->scan->size();
      if ((max_id > 0) && ((*it)->id == max_id)) break;
    }
    return retval;
  }


} // end namespace



