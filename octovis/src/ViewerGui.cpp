/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <iostream>
#include <fstream>
//#include <octomap/octomap_timing.h>

#include <octovis/ViewerGui.h>
#include <octovis/ColorOcTreeDrawer.h>
#include <octomap/MapCollection.h>
//Dummy object definition to ensure VS2012 does not drop the StaticMemberInitializer, causing this tree failing to register.
octomap::ColorOcTree colortreeTmp(0);


#define _MAXRANGE_URG 5.1
#define _MAXRANGE_SICK 50.0

namespace octomap{

ViewerGui::ViewerGui(const std::string& filename, QWidget *parent, unsigned int initDepth)
: QMainWindow(parent), m_scanGraph(NULL),
  m_trajectoryDrawer(NULL), m_pointcloudDrawer(NULL),
  m_cameraFollowMode(NULL),
  m_octreeResolution(0.1), m_laserMaxRange(-1.), m_occupancyThresh(0.5),
  m_max_tree_depth(initDepth > 0 && initDepth <= 16 ? initDepth : 16), 
  m_laserType(LASERTYPE_SICK),
  m_cameraStored(false),
  m_filename("") 
{

  ui.setupUi(this);
  m_glwidget = new ViewerWidget(this);
  this->setCentralWidget(m_glwidget);

  // Settings panel at the right side
  ViewerSettingsPanel* settingsPanel = new ViewerSettingsPanel(this);
  settingsPanel->setTreeDepth(initDepth);
  QDockWidget* settingsDock = new QDockWidget("Octree / Scan graph settings", this);
  settingsDock->setWidget(settingsPanel);
  this->addDockWidget(Qt::RightDockWidgetArea, settingsDock);
  ui.menuShow->addAction(settingsDock->toggleViewAction());

  // Camera settings panel at the right side
  ViewerSettingsPanelCamera* settingsCameraPanel = new ViewerSettingsPanelCamera(this);
  QDockWidget *settingsCameraDock = new QDockWidget("Camera settings", this);
  settingsCameraDock->setWidget(settingsCameraPanel);
  this->addDockWidget(Qt::RightDockWidgetArea, settingsCameraDock);
  this->tabifyDockWidget(settingsDock, settingsCameraDock);
  settingsDock->raise();
  ui.menuShow->addAction(settingsCameraDock->toggleViewAction());

  // status bar
  m_nodeSelected = new QLabel("Selected node coordinates", this);
  m_mapSizeStatus = new QLabel("Map size", this);
  m_mapMemoryStatus = new QLabel("Memory consumption", this);
  m_nodeSelected->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  m_mapSizeStatus->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  m_mapMemoryStatus->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  statusBar()->addPermanentWidget(m_nodeSelected);
  statusBar()->addPermanentWidget(m_mapSizeStatus);
  statusBar()->addPermanentWidget(m_mapMemoryStatus);

  m_cameraFollowMode = new CameraFollowMode();

  connect(this, SIGNAL(updateStatusBar(QString, int)), statusBar(), SLOT(showMessage(QString, int)));

  connect(settingsPanel, SIGNAL(treeDepthChanged(int)), this, SLOT(changeTreeDepth(int)));
  connect(settingsPanel, SIGNAL(addNextScans(unsigned)), this, SLOT(addNextScans(unsigned)));
  connect(settingsPanel, SIGNAL(gotoFirstScan()), this, SLOT(gotoFirstScan()));

  connect(settingsCameraPanel, SIGNAL(jumpToFrame(unsigned)), m_cameraFollowMode, SLOT(jumpToFrame(unsigned)));
  connect(settingsCameraPanel, SIGNAL(play()), m_cameraFollowMode, SLOT(play()));
  connect(settingsCameraPanel, SIGNAL(pause()), m_cameraFollowMode, SLOT(pause()));
  connect(settingsCameraPanel, SIGNAL(clearCameraPath()), m_cameraFollowMode, SLOT(clearCameraPath()));
  connect(settingsCameraPanel, SIGNAL(saveToCameraPath()), m_cameraFollowMode, SLOT(saveToCameraPath()));
  connect(settingsCameraPanel, SIGNAL(removeFromCameraPath()), m_cameraFollowMode, SLOT(removeFromCameraPath()));
  connect(settingsCameraPanel, SIGNAL(addToCameraPath()), m_cameraFollowMode, SLOT(addToCameraPath()));
  connect(settingsCameraPanel, SIGNAL(followCameraPath()), m_cameraFollowMode, SLOT(followCameraPath()));
  connect(settingsCameraPanel, SIGNAL(followRobotPath()), m_cameraFollowMode, SLOT(followRobotPath()));

  connect(m_cameraFollowMode, SIGNAL(changeNumberOfFrames(unsigned)), settingsCameraPanel, SLOT(setNumberOfFrames(unsigned)));
  connect(m_cameraFollowMode, SIGNAL(frameChanged(unsigned)), settingsCameraPanel, SLOT(setCurrentFrame(unsigned)));
  connect(m_cameraFollowMode, SIGNAL(stopped()), settingsCameraPanel, SLOT(setStopped()));
  connect(m_cameraFollowMode, SIGNAL(scanGraphAvailable(bool)), settingsCameraPanel, SLOT(setRobotTrajectoryAvailable(bool)));

  connect(m_cameraFollowMode, SIGNAL(deleteCameraPath(int)), m_glwidget, SLOT(deleteCameraPath(int)));
  connect(m_cameraFollowMode, SIGNAL(removeFromCameraPath(int,int)), m_glwidget, SLOT(removeFromCameraPath(int,int)));
  connect(m_cameraFollowMode, SIGNAL(appendToCameraPath(int, const octomath::Pose6D&)), m_glwidget, SLOT(appendToCameraPath(int, const octomath::Pose6D&)));
  connect(m_cameraFollowMode, SIGNAL(appendCurrentToCameraPath(int)), m_glwidget, SLOT(appendCurrentToCameraPath(int)));
  connect(m_cameraFollowMode, SIGNAL(addCurrentToCameraPath(int,int)), m_glwidget, SLOT(addCurrentToCameraPath(int,int)));
  connect(m_cameraFollowMode, SIGNAL(updateCameraPath(int,int)), m_glwidget, SLOT(updateCameraPath(int,int)));
  connect(m_cameraFollowMode, SIGNAL(playCameraPath(int,int)), m_glwidget, SLOT(playCameraPath(int,int)));
  connect(m_cameraFollowMode, SIGNAL(stopCameraPath(int)), m_glwidget, SLOT(stopCameraPath(int)));
  connect(m_cameraFollowMode, SIGNAL(jumpToCamFrame(int, int)), m_glwidget, SLOT(jumpToCamFrame(int, int)));
  connect(m_glwidget, SIGNAL(cameraPathStopped(int)), m_cameraFollowMode, SLOT(cameraPathStopped(int)));
  connect(m_glwidget, SIGNAL(cameraPathFrameChanged(int, int)), m_cameraFollowMode, SLOT(cameraPathFrameChanged(int, int)));

  connect(this, SIGNAL(changeNumberOfScans(unsigned)), settingsPanel, SLOT(setNumberOfScans(unsigned)));
  connect(this, SIGNAL(changeCurrentScan(unsigned)), settingsPanel, SLOT(setCurrentScan(unsigned)));
  connect(this, SIGNAL(changeResolution(double)), settingsPanel, SLOT(setResolution(double)));

  connect(settingsCameraPanel, SIGNAL(changeCamPosition(double, double, double, double, double, double)),
          m_glwidget, SLOT(setCamPosition(double, double, double, double, double, double)));
  connect(m_cameraFollowMode, SIGNAL(changeCamPose(const octomath::Pose6D&)),
          m_glwidget, SLOT(setCamPose(const octomath::Pose6D&)));

  connect(ui.actionReset_view, SIGNAL(triggered()), m_glwidget, SLOT(resetView()));
  connect(m_glwidget, SIGNAL(select(const QMouseEvent*)), this, SLOT(voxelSelected(const QMouseEvent*)));

  if (filename != ""){
    m_filename = filename;
    openFile();
  }
}

ViewerGui::~ViewerGui() {
  if (m_trajectoryDrawer){
    m_glwidget->removeSceneObject(m_trajectoryDrawer);
    delete m_trajectoryDrawer;
    m_trajectoryDrawer = NULL;
  }

  if (m_pointcloudDrawer){
    m_glwidget->removeSceneObject(m_pointcloudDrawer);
    delete m_pointcloudDrawer;
    m_pointcloudDrawer = NULL;
  }

  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin(); it != m_octrees.end(); ++it) {
    m_glwidget->removeSceneObject(it->second.octree_drawer);
    delete (it->second.octree_drawer);
    delete (it->second.octree);
  }
  m_octrees.clear();

  if(m_cameraFollowMode) {
    delete m_cameraFollowMode;
    m_cameraFollowMode = NULL;
  }
}

bool ViewerGui::isShown() {
  return  m_glwidget->isVisible();
}

void ViewerGui::showInfo(QString string, bool newline) {
  std::cerr << string.toLocal8Bit().data();
  if (newline) std::cerr << std::endl;
  else std::cerr << std::flush;
  int duration = 0;
  if (newline)
    duration = 3000;
  emit updateStatusBar(string, duration);
}

bool ViewerGui::getOctreeRecord(int id, OcTreeRecord*& otr) {
  std::map<int, OcTreeRecord>::iterator it = m_octrees.find(id);
  if( it != m_octrees.end() ) {
    otr = &(it->second);
    return true;
  }
  else {
    return false;
  }
}

void ViewerGui::addOctree(octomap::AbstractOcTree* tree, int id, octomap::pose6d origin) {
  // is id in use?
      OcTreeRecord* r;
      bool foundRecord = getOctreeRecord(id, r);
      if (foundRecord && r->octree->getTreeType().compare(tree->getTreeType()) !=0){
        // delete old drawer, create new
        delete r->octree_drawer;
        if (dynamic_cast<OcTree*>(tree)) {
          r->octree_drawer = new OcTreeDrawer();
          //        fprintf(stderr, "adding new OcTreeDrawer for node %d\n", id);
        }
        else if (dynamic_cast<ColorOcTree*>(tree)) {
          r->octree_drawer = new ColorOcTreeDrawer();
        } else{
          OCTOMAP_ERROR("Could not create drawer for tree type %s\n", tree->getTreeType().c_str());
        }

        delete r->octree;
        r->octree = tree;
        r->origin = origin;

      } else if (foundRecord && r->octree->getTreeType().compare(tree->getTreeType()) ==0) {
        // only swap out tree

        delete r->octree;
        r->octree = tree;
        r->origin = origin;
      } else {
        // add new record
        OcTreeRecord otr;
        otr.id = id;
        if (dynamic_cast<OcTree*>(tree)) {
          otr.octree_drawer = new OcTreeDrawer();
          //        fprintf(stderr, "adding new OcTreeDrawer for node %d\n", id);
        }
        else if (dynamic_cast<ColorOcTree*>(tree)) {
          otr.octree_drawer = new ColorOcTreeDrawer();
        } else{
          OCTOMAP_ERROR("Could not create drawer for tree type %s\n", tree->getTreeType().c_str());
        }
        otr.octree = tree;
        otr.origin = origin;
        m_octrees[id] = otr;
        m_glwidget->addSceneObject(otr.octree_drawer);
      }
}

void ViewerGui::addOctree(octomap::AbstractOcTree* tree, int id) {
  octomap::pose6d o; // initialized to (0,0,0) , (0,0,0,1) by default
  addOctree(tree, id, o);
}

void ViewerGui::showOcTree() {

  // update viewer stat
  double minX, minY, minZ, maxX, maxY, maxZ;
  minX = minY = minZ = -10; // min bbx for drawing
  maxX = maxY = maxZ = 10;  // max bbx for drawing
  double sizeX, sizeY, sizeZ;
  sizeX = sizeY = sizeZ = 0.;
  size_t memoryUsage = 0;
  size_t num_nodes = 0;
  size_t memorySingleNode = 0;


  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin(); it != m_octrees.end(); ++it) {
    // get map bbx
    double lminX, lminY, lminZ, lmaxX, lmaxY, lmaxZ;
    it->second.octree->getMetricMin(lminX, lminY, lminZ);
    it->second.octree->getMetricMax(lmaxX, lmaxY, lmaxZ);
    // transform to world coords using map origin
    octomap::point3d pmin(lminX, lminY, lminZ);
    octomap::point3d pmax(lmaxX, lmaxY, lmaxZ);
    pmin = it->second.origin.transform(pmin);
    pmax = it->second.origin.transform(pmax);
    lminX = pmin.x(); lminY = pmin.y(); lminZ = pmin.z();
    lmaxX = pmax.x(); lmaxY = pmax.y(); lmaxZ = pmax.z();
    // update global bbx
    if (lminX < minX) minX = lminX;
    if (lminY < minY) minY = lminY;
    if (lminZ < minZ) minZ = lminZ;
    if (lmaxX > maxX) maxX = lmaxX;
    if (lmaxY > maxY) maxY = lmaxY;
    if (lmaxZ > maxZ) maxZ = lmaxZ;
    double lsizeX, lsizeY, lsizeZ;
    // update map stats
    it->second.octree->getMetricSize(lsizeX, lsizeY, lsizeZ);
    if (lsizeX > sizeX) sizeX = lsizeX;
    if (lsizeY > sizeY) sizeY = lsizeY;
    if (lsizeZ > sizeZ) sizeZ = lsizeZ;
    memoryUsage += it->second.octree->memoryUsage();
    num_nodes += it->second.octree->size();
    memorySingleNode = std::max(memorySingleNode, it->second.octree->memoryUsageNode());
  }

  m_glwidget->setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));

  //if (m_octrees.size()) {
  QString size = QString("%L1 x %L2 x %L3 m^3; %L4 nodes").arg(sizeX).arg(sizeY).arg(sizeZ).arg(unsigned(num_nodes));
  QString memory = QString("Single node: %L1 B; ").arg(memorySingleNode)
            + QString ("Octree: %L1 B (%L2 MB)").arg(memoryUsage).arg((double) memoryUsage/(1024.*1024.), 0, 'f', 3);
  m_mapMemoryStatus->setText(memory);
  m_mapSizeStatus->setText(size);
  //}

  m_glwidget->update();

  // generate cubes -> display
  // timeval start;
  // timeval stop;
  // gettimeofday(&start, NULL);  // start timer
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin(); it != m_octrees.end(); ++it) {
    it->second.octree_drawer->setMax_tree_depth(m_max_tree_depth);
    it->second.octree_drawer->setOcTree(*it->second.octree, it->second.origin, it->second.id);
  }
  //    gettimeofday(&stop, NULL);  // stop timer
  //    double time_to_generate = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
  //    fprintf(stderr, "setOcTree took %f sec\n", time_to_generate);
  m_glwidget->update();
}


void ViewerGui::generateOctree() {

  if (m_scanGraph) {

    QApplication::setOverrideCursor(Qt::WaitCursor);

    showInfo("Generating OcTree... ");
    std::cerr << std::endl;

    //if (m_ocTree) delete m_ocTree;
    OcTree* tree = new octomap::OcTree(m_octreeResolution);

    octomap::ScanGraph::iterator it;
    unsigned numScans = m_scanGraph->size();
    unsigned currentScan = 1;
    for (it = m_scanGraph->begin(); it != m_nextScanToAdd; it++) {
      tree->insertPointCloud(**it, m_laserMaxRange);
      fprintf(stderr, "generateOctree:: inserting scan node with %d points, origin: %.2f  ,%.2f , %.2f.\n",
              (unsigned int) (*it)->scan->size(), (*it)->pose.x(), (*it)->pose.y(), (*it)->pose.z()  );

      std::cout << " S ("<<currentScan<<"/"<<numScans<<") " << std::flush;
      currentScan++;
    }

    this->addOctree(tree, DEFAULT_OCTREE_ID);
    this->showOcTree();

    showInfo("Done.", true);
    QApplication::restoreOverrideCursor();
  }
  else {
    std::cerr << "generateOctree called but no ScanGraph present!\n";
  }

}

// ==  incremental graph generation   =======================

void ViewerGui::gotoFirstScan(){
  if (m_scanGraph){
    showInfo("Inserting first scan node into tree... ", true);
    QApplication::setOverrideCursor(Qt::WaitCursor);

    m_nextScanToAdd = m_scanGraph->begin();

    // if (m_ocTree) delete m_ocTree;
    // m_ocTree = new octomap::OcTree(m_octreeResolution);
    OcTree* tree = new octomap::OcTree(m_octreeResolution);
    this->addOctree(tree, DEFAULT_OCTREE_ID);

    addNextScan();

    QApplication::restoreOverrideCursor();
    showOcTree();
  }
}

void ViewerGui::addNextScan(){

  if (m_scanGraph){
    showInfo("Inserting next scan node into tree... ", true);

    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (m_nextScanToAdd != m_scanGraph->end()){
      OcTreeRecord* r;
      if (!getOctreeRecord(DEFAULT_OCTREE_ID, r)) {
        fprintf(stderr, "ERROR: OctreeRecord for id %d not found!\n", DEFAULT_OCTREE_ID);
        return;
      }
      // not used with ColorOcTrees, omitting casts
      ((OcTree*) r->octree)->insertPointCloud(**m_nextScanToAdd, m_laserMaxRange);
      m_nextScanToAdd++;
    }

    QApplication::restoreOverrideCursor();
    showOcTree();

  }
}


void ViewerGui::addNextScans(unsigned scans){
  for (unsigned i = 0; i < scans; ++i){
    addNextScan();
  }
}


// ==  file I/O   ===========================================

void ViewerGui::openFile(){
  if (!m_filename.empty()){
    m_glwidget->clearAll();

    QString temp = QString(m_filename.c_str());
    QFileInfo fileinfo(temp);
    this->setWindowTitle(fileinfo.fileName());
    if (fileinfo.suffix() == "graph"){
      openGraph();
    }else if (fileinfo.suffix() == "bt"){
      openTree();
    }
    else if (fileinfo.suffix() == "ot")
    {
      openOcTree();
    }
    else if (fileinfo.suffix() == "hot"){
      openMapCollection();
    }
    else if (fileinfo.suffix() == "dat"){
      openPointcloud();
    }
    else {
      QMessageBox::warning(this, "Unknown file", "Cannot open file, unknown extension: "+fileinfo.suffix(), QMessageBox::Ok);
    }
  }
}


void ViewerGui::openGraph(bool completeGraph){

  QApplication::setOverrideCursor(Qt::WaitCursor);
  showInfo("Loading scan graph from file " + QString(m_filename.c_str()) );

  if (m_scanGraph) delete m_scanGraph;
  m_scanGraph = new octomap::ScanGraph();
  m_scanGraph->readBinary(m_filename);

  loadGraph(completeGraph);
}


void ViewerGui::openPointcloud(){

  QApplication::setOverrideCursor(Qt::WaitCursor);
  showInfo("Loading ASCII pointcloud from file "+QString(m_filename.c_str()) + "...");

  if (m_scanGraph) delete m_scanGraph;
  m_scanGraph = new octomap::ScanGraph();


  // read pointcloud from file
  std::ifstream s(m_filename.c_str());
  Pointcloud* pc = new Pointcloud();

  if (!s) {
    std::cout <<"ERROR: could not read " << m_filename << std::endl;
    return;
  }

  pc->read(s);

  pose6d laser_pose(0,0,0,0,0,0);
  m_scanGraph->addNode(pc, laser_pose);

  loadGraph(true);
  showInfo("Done.", true);
}


void ViewerGui::setOcTreeUISwitches() {
  ui.actionPointcloud->setChecked(false);
  ui.actionPointcloud->setEnabled(false);
  ui.actionOctree_cells->setChecked(true);
  ui.actionOctree_cells->setEnabled(true);
  ui.actionFree->setChecked(false);
  ui.actionFree->setEnabled(true);
  ui.actionOctree_structure->setEnabled(true);
  ui.actionOctree_structure->setChecked(false);
  ui.actionTrajectory->setEnabled(false);
  ui.actionConvert_ml_tree->setEnabled(true);
  ui.actionReload_Octree->setEnabled(true);
  ui.actionSettings->setEnabled(false);
  ui.actionSelected->setChecked(true);
  ui.actionSelected->setEnabled(true);
}

void ViewerGui::openTree(){
  OcTree* tree = new octomap::OcTree(m_filename);
  this->addOctree(tree, DEFAULT_OCTREE_ID);

  m_octreeResolution = tree->getResolution();
  emit changeResolution(m_octreeResolution);

  setOcTreeUISwitches();
  showOcTree();
  m_glwidget->resetView();
}

void ViewerGui::openOcTree(){
  AbstractOcTree* tree = AbstractOcTree::read(m_filename);

  if (tree){
    this->addOctree(tree, DEFAULT_OCTREE_ID);

    m_octreeResolution = tree->getResolution();
    emit changeResolution(m_octreeResolution);

    setOcTreeUISwitches();
    showOcTree();
    m_glwidget->resetView();

    if (tree->getTreeType() == "ColorOcTree"){
      // map color and height map share the same color array and QAction
      ui.actionHeight_map->setText ("Map color");  // rename QAction in Menu
      this->on_actionHeight_map_toggled(true); // enable color view
      ui.actionHeight_map->setChecked(true);
    }
  }
  else {
    QMessageBox::warning(this, "File error", "Cannot open OcTree file", QMessageBox::Ok);
  }
}


// EXPERIMENTAL
void ViewerGui::openMapCollection() {

  OCTOMAP_DEBUG("Opening hierarchy from %s...\n", m_filename.c_str());

  std::ifstream infile(m_filename.c_str(), std::ios_base::in |std::ios_base::binary);
  if (!infile.is_open()) {
    QMessageBox::warning(this, "File error", "Cannot open OcTree file", QMessageBox::Ok);
    return;
  }
  infile.close();

  MapCollection<MapNode<OcTree> > collection(m_filename);
  int i=0;
  for (MapCollection<MapNode<OcTree> >::iterator it = collection.begin();
      it != collection.end(); ++it) {
    OCTOMAP_DEBUG("Adding hierarchy node %s\n", (*it)->getId().c_str());
    OcTree* tree = (*it)->getMap();
    if (!tree)
      OCTOMAP_ERROR("Error while reading node %s\n", (*it)->getId().c_str());
    else {
      OCTOMAP_DEBUG("Read tree with %zu tree nodes\n", tree->size());
    }
    pose6d  origin = (*it)->getOrigin();
    this->addOctree(tree, i, origin);
    ++i;
  }
  setOcTreeUISwitches();
  showOcTree();
  m_glwidget->resetView();
  OCTOMAP_DEBUG("done\n");
}

void ViewerGui::loadGraph(bool completeGraph) {

  ui.actionSettings->setEnabled(true);
  ui.actionPointcloud->setEnabled(true);
  ui.actionPointcloud->setChecked(false);
  ui.actionTrajectory->setEnabled(true);
  ui.actionOctree_cells->setEnabled(true);
  ui.actionOctree_cells->setChecked(true);
  ui.actionOctree_structure->setEnabled(true);
  ui.actionOctree_structure->setChecked(false);
  ui.actionFree->setChecked(false);
  ui.actionFree->setEnabled(true);
  ui.actionReload_Octree->setEnabled(true);
  ui.actionConvert_ml_tree->setEnabled(true);

  unsigned graphSize = m_scanGraph->size();
  unsigned currentScan;

  if (completeGraph){
    fprintf(stderr, "loadGraph:: generating octree from complete graph.\n" );
    m_nextScanToAdd = m_scanGraph->end();
    generateOctree();
    currentScan = graphSize;
  }
  else{
    m_nextScanToAdd = m_scanGraph->begin();

    //if (m_ocTree) delete m_ocTree;
    //m_ocTree = new octomap::OcTree(m_octreeResolution);
    OcTree* tree = new octomap::OcTree(m_octreeResolution);
    this->addOctree(tree, DEFAULT_OCTREE_ID);

    addNextScan();

    currentScan = 1;
  }

  m_glwidget->resetView();
  QApplication::restoreOverrideCursor();

  emit changeNumberOfScans(graphSize);
  emit changeCurrentScan(currentScan);
  showInfo("Done (" +QString::number(currentScan)+ " of "+ QString::number(graphSize)+" nodes)", true);

  if (!m_trajectoryDrawer){
    m_trajectoryDrawer = new TrajectoryDrawer();
  }
  m_trajectoryDrawer->setScanGraph(*m_scanGraph);

  if (!m_pointcloudDrawer){
    m_pointcloudDrawer = new PointcloudDrawer();
  }
  m_pointcloudDrawer->setScanGraph(*m_scanGraph);

  m_cameraFollowMode->setScanGraph(m_scanGraph);

  if (ui.actionTrajectory->isChecked())
    m_glwidget->addSceneObject(m_trajectoryDrawer);

  if (ui.actionPointcloud->isChecked())
    m_glwidget->addSceneObject(m_pointcloudDrawer);
}

void ViewerGui::changeTreeDepth(int depth){
  // range check:
  if (depth < 1 || depth > 16)
    return;

  m_max_tree_depth = unsigned(depth);

  if (m_octrees.size() > 0)
    showOcTree();
}


void ViewerGui::on_actionExit_triggered(){
  this->close();
}

void ViewerGui::on_actionHelp_triggered(){
  m_glwidget->help();
}

void ViewerGui::on_actionSettings_triggered(){

  ViewerSettings dialog(this);
  dialog.setResolution(m_octreeResolution);
  dialog.setLaserType(m_laserType);
  dialog.setMaxRange(m_laserMaxRange);


  if (dialog.exec()){

    double oldResolution = m_octreeResolution;
    double oldLaserMaxRange = m_laserMaxRange;
    double oldType = m_laserType;

    m_octreeResolution = dialog.getResolution();
    m_laserType = dialog.getLaserType();
    m_laserMaxRange = dialog.getMaxRange();

    // apply new settings
    bool resolutionChanged = (std::abs(oldResolution - m_octreeResolution) > 1e-5);
    bool maxRangeChanged = (std::abs(oldLaserMaxRange - m_laserMaxRange) > 1e-5);

    if (resolutionChanged)
      emit changeResolution(m_octreeResolution);

    if (oldType != m_laserType){ // parameters changed, reload file:
      openFile();
    } else if (resolutionChanged || maxRangeChanged){
      generateOctree();
    }

  }
}

void ViewerGui::on_actionOpen_file_triggered(){
  QString filename = QFileDialog::getOpenFileName(this,
                                                  tr("Open data file"), "",
                                                  "All supported files (*.graph *.bt *.ot *.dat);;OcTree file (*.ot);;Bonsai tree file (*.bt);;Binary scan graph (*.graph);;Pointcloud (*.dat);;All files (*)");
  if (!filename.isEmpty()){
#ifdef _WIN32      
    m_filename = std::string(filename.toLocal8Bit().data());
#else       
    m_filename = filename.toStdString();
#endif
    openFile();
  }
}


void ViewerGui::on_actionOpen_graph_incremental_triggered(){
  QString filename = QFileDialog::getOpenFileName(this,
                                                  tr("Open graph file incrementally (at start)"), "",
                                                  "Binary scan graph (*.graph)");
  if (!filename.isEmpty()){
    m_glwidget->clearAll();

#ifdef _WIN32      
    m_filename = std::string(filename.toLocal8Bit().data());
#else       
    m_filename = filename.toStdString();
#endif

    openGraph(false);
  }
}

void ViewerGui::on_actionSave_file_triggered(){

  OcTreeRecord* r;
  if (!getOctreeRecord(DEFAULT_OCTREE_ID, r)) {
    fprintf(stderr, "ERROR: OctreeRecord for id %d not found!\n", DEFAULT_OCTREE_ID);
    QMessageBox::warning(this, tr("3D Mapping Viewer"),
                         "Error: No OcTree present.",
                         QMessageBox::Ok);
    return;
  }

  QString filename = QFileDialog::getSaveFileName(this, tr("Save octree file"),
                                                  "", tr("Full OcTree (*.ot);;Bonsai Tree file (*.bt);;"));

  if (filename != ""){
    QApplication::setOverrideCursor(Qt::WaitCursor);
    showInfo("Writing file... ", false);

    QFileInfo fileinfo(filename);
    std::string std_filename;
#ifdef _WIN32      
    std_filename = filename.toLocal8Bit().data();
#else       
    std_filename = filename.toStdString();
#endif

    AbstractOcTree* t = r->octree;

    if (fileinfo.suffix() == "bt") {
      AbstractOccupancyOcTree* ot = dynamic_cast<AbstractOccupancyOcTree*> (t);
      if (ot)
        ot->writeBinaryConst(std_filename);
      else{
        QMessageBox::warning(this, "Unknown tree type",
                             "Could not convert to occupancy tree for writing .bt file",
                             QMessageBox::Ok);
      }
    }
    else if (fileinfo.suffix() == "ot"){
      r->octree->write(std_filename);
    }
    else {
      QMessageBox::warning(this, "Unknown file",
                           "Cannot write file, unknown extension: "+fileinfo.suffix(),
                           QMessageBox::Ok);
    }

    QApplication::restoreOverrideCursor();
    showInfo("Done.", true);
  }
}

void ViewerGui::on_actionClear_nodes_in_selection_triggered(){
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  updateNodesInBBX(min, max, false);
}

void ViewerGui::on_actionClear_selection_triggered(){
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  setNodesInBBX(min, max, false);
}

void ViewerGui::on_actionClear_unknown_in_selection_triggered()
{
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  setNonNodesInBBX(min, max, false);
}

void ViewerGui::on_actionFill_unknown_in_selection_triggered()
{
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  setNonNodesInBBX(min, max, true);
}

void ViewerGui::on_actionFill_selection_triggered(){
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  setNodesInBBX(min, max, true);
}

void ViewerGui::on_actionFill_nodes_in_selection_triggered(){
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  updateNodesInBBX(min, max, true);
}

void ViewerGui::on_actionDelete_nodes_in_selection_triggered(){
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  for (std::map<int, OcTreeRecord>::iterator t_it = m_octrees.begin(); t_it != m_octrees.end(); ++t_it) {
    OcTree* octree = dynamic_cast<OcTree*>(t_it->second.octree);

    if (octree){
      for(OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max),
          end=octree->end_leafs_bbx(); it!= end; ++it){
        octree->deleteNode(it.getKey(), it.getDepth());
      }
    } else{
      QMessageBox::warning(this, "Not implemented", "Functionality not yet implemented for this octree type",
                           QMessageBox::Ok);

    }
  }

  showOcTree();
}

void ViewerGui::on_actionDelete_nodes_outside_of_selection_triggered(){
  point3d min, max;
  m_glwidget->selectionBox().getBBXMin(min.x(), min.y(), min.z());
  m_glwidget->selectionBox().getBBXMax(max.x(), max.y(), max.z());

  for (std::map<int, OcTreeRecord>::iterator t_it = m_octrees.begin(); t_it != m_octrees.end(); ++t_it) {
    OcTree* octree = dynamic_cast<OcTree*>(t_it->second.octree);

    if (octree){
      octomap::OcTreeKey minKey, maxKey;

      if (!octree->coordToKeyChecked(min, minKey) || !octree->coordToKeyChecked(max, maxKey)){
        return;
      }

      for(OcTree::leaf_iterator it = octree->begin_leafs(),
          end=octree->end_leafs(); it!= end; ++it){
        // check if outside of bbx:
        OcTreeKey k = it.getKey();
        if  (k[0] < minKey[0] || k[1] < minKey[1] || k[2] < minKey[2]
                                                                   || k[0] > maxKey[0] || k[1] > maxKey[1] || k[2] > maxKey[2])
        {
          octree->deleteNode(k, it.getDepth());
        }
      }
    } else
      QMessageBox::warning(this, "Not implemented", "Functionality not yet implemented for this octree type",
                           QMessageBox::Ok);
  }

  showOcTree();
}

void ViewerGui::updateNodesInBBX(const point3d& min, const point3d& max, bool occupied){
  for (std::map<int, OcTreeRecord>::iterator t_it = m_octrees.begin(); t_it != m_octrees.end(); ++t_it) {
    OcTree* octree = dynamic_cast<OcTree*>(t_it->second.octree);

    if (octree){
      float logodds;
      if (occupied)
        logodds = octree->getClampingThresMaxLog();
      else
        logodds = octree->getClampingThresMinLog();

      for(OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max),
          end=octree->end_leafs_bbx(); it!= end; ++it)
      {
        // directly set values of leafs:
        it->setLogOdds(logodds);
      }

      // update inner nodes to make tree consistent:
      octree->updateInnerOccupancy();

    } else
      QMessageBox::warning(this, "Not implemented", "Functionality not yet implemented for this octree type",
                           QMessageBox::Ok);

  }

  showOcTree();
}


void ViewerGui::setNodesInBBX(const point3d& min, const point3d& max, bool occupied){
  for (std::map<int, OcTreeRecord>::iterator t_it = m_octrees.begin(); t_it != m_octrees.end(); ++t_it) {
    OcTree* octree = dynamic_cast<OcTree*>(t_it->second.octree);

    if (octree){
      float logodds = octree->getClampingThresMaxLog() - octree->getClampingThresMinLog();
      if (!occupied)
        logodds *= -1;

      OcTreeKey minKey(0,0,0);
      OcTreeKey maxKey(0,0,0);
      octree->coordToKeyChecked(min, minKey);
      octree->coordToKeyChecked(max, maxKey);
      OcTreeKey k;
      for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
        for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
          for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
            octree->updateNode(k, logodds);
          }
        }
      }
    }

  }

  showOcTree();
}

void ViewerGui::setNonNodesInBBX(const point3d& min, const point3d& max, bool occupied) {
  for (std::map<int, OcTreeRecord>::iterator t_it = m_octrees.begin(); t_it != m_octrees.end(); ++t_it) {
    OcTree* octree = dynamic_cast<OcTree*>(t_it->second.octree);

    if (octree){
      float logodds = octree->getClampingThresMaxLog() - octree->getClampingThresMinLog();
      if (!occupied)
        logodds *= -1;

      OcTreeKey minKey(0,0,0);
      OcTreeKey maxKey(0,0,0);
      octree->coordToKeyChecked(min, minKey);
      octree->coordToKeyChecked(max, maxKey);
      OcTreeKey k;
      for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
        for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
          for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
            OcTreeNode* n = octree->search(k);
            if(!n)
              octree->updateNode(k, logodds);
          }
        }
      }
    }

  }

  showOcTree();
}

void ViewerGui::on_actionExport_view_triggered(){
  m_glwidget->openSnapshotFormatDialog();
  m_glwidget->saveSnapshot(false);
}

void ViewerGui::on_actionExport_sequence_triggered(bool checked){
  if(checked) {
    if(m_glwidget->openSnapshotFormatDialog()) {
      m_glwidget->saveSnapshot(false);
      m_glwidget->setSnapshotCounter(0);
      connect(m_glwidget, SIGNAL(drawFinished(bool)), m_glwidget, SLOT(saveSnapshot(bool)));
    } else {
      ui.actionExport_sequence->setChecked(false);
    }
  } else {
    disconnect(m_glwidget, SIGNAL(drawFinished(bool)), m_glwidget, SLOT(saveSnapshot(bool)));
  }
}

void ViewerGui::on_actionPrintout_mode_toggled(bool checked){
  if (checked) {
    ui.actionHeight_map->setChecked(false);
    ui.actionSemanticColoring->setChecked(false);
  }

  m_glwidget->enablePrintoutMode(checked);
}

void ViewerGui::on_actionSelection_box_toggled(bool checked){

  ui.menuDelete_nodes->setEnabled(checked);
  ui.menuFill_selection->setEnabled(checked);
  ui.menuChange_nodes_in_selection->setEnabled(checked);


  m_glwidget->enableSelectionBox(checked);


  m_glwidget->update();
}

void ViewerGui::on_actionHeight_map_toggled(bool checked){
  if (checked) {
    ui.actionPrintout_mode->setChecked(false);
    ui.actionSemanticColoring->setChecked(false);
  }
  m_glwidget->enableHeightColorMode(checked);
}

void ViewerGui::on_actionSemanticColoring_toggled(bool checked) {
  if (checked) {
    ui.actionHeight_map->setChecked(false);
    ui.actionPrintout_mode->setChecked(false);
  }

  m_glwidget->enableSemanticColoring(checked);
}


void ViewerGui::on_actionStore_camera_triggered(){
  m_glwidget->camera()->deletePath(0);
  m_glwidget->camera()->addKeyFrameToPath(0);
  m_cameraStored = true;
  ui.actionRestore_camera->setEnabled(true);
}

void ViewerGui::on_actionRestore_camera_triggered(){
  if (m_cameraStored){
    m_glwidget->camera()->playPath(0);
  }
}

void ViewerGui::on_actionPointcloud_toggled(bool checked){
  if (m_pointcloudDrawer){
    if (checked)
      m_glwidget->addSceneObject(m_pointcloudDrawer);
    else
      m_glwidget->removeSceneObject(m_pointcloudDrawer);
  }
}

void ViewerGui::on_actionTrajectory_toggled(bool checked){
  if (m_trajectoryDrawer){
    if (checked)
      m_glwidget->addSceneObject(m_trajectoryDrawer);
    else
      m_glwidget->removeSceneObject(m_trajectoryDrawer);
  }
}

void ViewerGui::on_actionAxes_toggled(bool checked){
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
      it != m_octrees.end(); ++it) {
    it->second.octree_drawer->enableAxes(checked);
  }
  m_glwidget->update();
}

void ViewerGui::on_actionHideBackground_toggled(bool checked) {
  OcTreeRecord* r;
  if (getOctreeRecord(DEFAULT_OCTREE_ID, r)) {
    if (checked) m_glwidget->removeSceneObject(r->octree_drawer);
    else         m_glwidget->addSceneObject(r->octree_drawer);
    m_glwidget->update();
  }
}

void ViewerGui::on_actionAlternateRendering_toggled(bool checked) {
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin(); it != m_octrees.end(); ++it) {
    //std::cout << "Setting Octree " << it->first << " to " << (checked ? "alternate" : "regular") << " rendering.";
    it->second.octree_drawer->setAlternativeDrawing(checked);
  }
}

void ViewerGui::on_actionClear_triggered() {
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
      it != m_octrees.end(); ++it) {
    m_glwidget->removeSceneObject(it->second.octree_drawer);
    delete (it->second.octree_drawer);
    delete (it->second.octree);
  }
  m_octrees.clear();
  showOcTree();
}

void ViewerGui::voxelSelected(const QMouseEvent* e){
  QPoint pixel_coord = e->pos();
  qglviewer::Vec origin;
  qglviewer::Vec direction;
  m_glwidget->camera()->convertClickToLine(pixel_coord, origin, direction);
  const point3d origin3d{(float)origin.x,(float)origin.y,(float)origin.z};
  const point3d direction3d{(float)direction.x,(float)direction.y,(float)direction.z};
  point3d end3d; // voxel coords hit by ray
  QString message = QString("--, --, -- m");
  std::list<octomap::OcTreeVolume> selection;

  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin(); it != m_octrees.end(); ++it)
  {
    AbstractOcTree* tree = it->second.octree;
    bool ray_hit = false;
    if (OcTree* occupancytree = dynamic_cast<OcTree*>(tree))
    {
      ray_hit = occupancytree->castRay(origin3d, direction3d, end3d, true); // ? append ray distance arg to avoid raycast to inf warnings
    }
    else if (ColorOcTree* occupancytree = dynamic_cast<ColorOcTree*>(tree))
    {
      ray_hit = occupancytree->castRay(origin3d, direction3d, end3d, true);
    }
    else
    {
      OCTOMAP_ERROR("Could not select nodes of this tree type %s\n", tree->getTreeType().c_str());
      continue;
    }
    if (ray_hit)
    {
      message = QString("%L1, %L2, %L3 m").arg(end3d.x()).arg(end3d.y()).arg(end3d.z());
      OcTreeVolume voxel = OcTreeVolume(end3d, tree->getResolution());
      selection.push_back(voxel);
      it->second.octree_drawer->setOcTreeSelection(selection);
    }
    else it->second.octree_drawer->clearOcTreeSelection();
  }
  m_nodeSelected->setText(message);
  m_glwidget->update();
}

void ViewerGui::on_actionTest_triggered(){

}

void ViewerGui::on_actionReload_Octree_triggered(){
  if (m_scanGraph) {
    generateOctree();
  } else {
    openFile();
  }
}

void ViewerGui::on_actionConvert_ml_tree_triggered(){
  QApplication::setOverrideCursor(Qt::WaitCursor);
  if (m_octrees.size()) {
    showInfo("Converting OcTree to maximum Likelihood map... ");
    for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
        it != m_octrees.end(); ++it) {
      AbstractOcTree* t = it->second.octree;
      if (dynamic_cast<OcTree*>(t)) {
        ((OcTree*) t)->toMaxLikelihood();
      }
      else if (dynamic_cast<OcTree*>(t)) {
        ((ColorOcTree*) t)->toMaxLikelihood();
      }
    }
    showInfo("Done.", true);
    showOcTree();
    QApplication::restoreOverrideCursor();
  }
}


void ViewerGui::on_actionPrune_tree_triggered(){
  QApplication::setOverrideCursor(Qt::WaitCursor);
  if (m_octrees.size()) {
    showInfo("Pruning OcTree... ");
    for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
        it != m_octrees.end(); ++it) {
      it->second.octree->prune();
    }
    showOcTree();
    showInfo("Done.", true);
  }
  QApplication::restoreOverrideCursor();
}


void ViewerGui::on_actionExpand_tree_triggered(){

  QApplication::setOverrideCursor(Qt::WaitCursor);

  // if (m_ocTree) {
  if (m_octrees.size()) {
    showInfo("Expanding OcTree... ");
    for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
        it != m_octrees.end(); ++it) {
      it->second.octree->expand();
    }
    showOcTree();

    showInfo("Done.", true);
  }
  QApplication::restoreOverrideCursor();
}


void ViewerGui::on_actionOctree_cells_toggled(bool enabled) {
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
      it != m_octrees.end(); ++it) {
    it->second.octree_drawer->enableOcTreeCells(enabled);
  }
  m_glwidget->update();
}

void ViewerGui::on_actionOctree_structure_toggled(bool enabled) {
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
      it != m_octrees.end(); ++it) {
    it->second.octree_drawer->enableOcTree(enabled);
  }
  m_glwidget->update();
}

void ViewerGui::on_actionFree_toggled(bool enabled) {
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
      it != m_octrees.end(); ++it) {
    it->second.octree_drawer->enableFreespace(enabled);
  }
  m_glwidget->update();

}

void ViewerGui::on_actionSelected_toggled(bool enabled) {
  for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin();
      it != m_octrees.end(); ++it) {
        if(it->second.octree_drawer)
          it->second.octree_drawer->enableSelection(enabled);
  }
}


void ViewerGui::on_action_bg_black_triggered() {
  m_glwidget->setBackgroundColor( QColor(0,0,0) );
}

void ViewerGui::on_action_bg_white_triggered() {
  m_glwidget->setBackgroundColor( QColor(255,255,255) );
}

void ViewerGui::on_action_bg_gray_triggered() {
  m_glwidget->setBackgroundColor( QColor(117,117,117) );
}

void ViewerGui::on_savecampose_triggered() {
  QString filename = QFileDialog::getSaveFileName(this, "Save Viewer State", "camera.xml", "Camera/State file (*.xml)");
  if (!filename.isEmpty()) {
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    saveCameraPosition(filename.toLatin1().constData());
#else  // QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    saveCameraPosition(filename.toAscii().constData());
#endif // QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  }
}

void ViewerGui::on_loadcampose_triggered() {
  QString filename = QFileDialog::getOpenFileName(this, "Load Viewer State", "camera.xml", "Camera/State file (*.xml)");
  if (!filename.isEmpty()) {
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    loadCameraPosition(filename.toLatin1().constData());
#else  // QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    loadCameraPosition(filename.toAscii().constData());
#endif // QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  }
}



void ViewerGui::saveCameraPosition(const char* filename) const {
  // HACK get non-const pointer to myself
  ViewerWidget* aux = const_cast<ViewerWidget*>( m_glwidget);
  aux->setStateFileName(QString(filename));
  aux->saveStateToFile();
  aux->setStateFileName(QString::null);
}

void ViewerGui::loadCameraPosition(const char* filename) {
  m_glwidget->setStateFileName(QString(filename));
  m_glwidget->restoreStateFromFile();
  m_glwidget->setStateFileName(QString::null);
}


}


