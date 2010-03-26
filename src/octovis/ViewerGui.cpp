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

#include <iostream>
#include <fstream>

#include "ViewerGui.h"
#define _MAXRANGE_URG 5.1
#define _MAXRANGE_SICK 50.0

namespace octomap{

  ViewerGui::ViewerGui(const std::string& filename, QWidget *parent)
    : QMainWindow(parent), m_scanGraph(NULL), m_ocTree(NULL), 
      m_trajectoryDrawer(NULL), m_pointcloudDrawer(NULL), 
      m_octreeDrawer(NULL), m_cameraFollowMode(NULL),
      m_octreeResolution(0.1), m_occupancyThresh(0.5), 
      m_max_tree_depth(16), m_laserType(LASERTYPE_SICK),
      m_cameraStored(false), m_filename("") {

    ui.setupUi(this);
    m_glwidget = new ViewerWidget(this);
    this->setCentralWidget(m_glwidget);

    m_octreeDrawer = new OcTreeDrawer();
    m_glwidget->addSceneObject(m_octreeDrawer);

    // Settings panel at the right side
    ViewerSettingsPanel* settingsPanel = new ViewerSettingsPanel(this);
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
    m_mapSizeStatus = new QLabel("Map size", this);
    m_mapMemoryStatus = new QLabel("Memory consumption", this);
    m_mapSizeStatus->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    m_mapMemoryStatus->setFrameStyle(QFrame::Panel | QFrame::Sunken);
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

    if (m_octreeDrawer){
      m_glwidget->removeSceneObject(m_octreeDrawer);
      delete m_octreeDrawer;
      m_octreeDrawer = NULL;
    }

    if(m_cameraFollowMode) {
    	delete m_cameraFollowMode;
    	m_cameraFollowMode = NULL;
    }
  }

  void ViewerGui::showInfo(QString string, bool newline) {

    std::cerr << string.toStdString();
    if (newline) std::cerr << std::endl;
    else std::cerr << std::flush;
    int duration = 0;
    if (newline)
      duration = 3000;
    emit updateStatusBar(string, duration);
  }


void ViewerGui::generateOctree() {

  if (m_scanGraph) {

    QApplication::setOverrideCursor(Qt::WaitCursor);

    showInfo("Generating OcTree... ");

    if (m_ocTree) delete m_ocTree;
    m_ocTree = new octomap::OcTree(m_octreeResolution);

    octomap::ScanGraph::iterator it;
    unsigned numScans = m_scanGraph->size();
    unsigned currentScan = 1;
    for (it = m_scanGraph->begin(); it != m_nextScanToAdd; it++) {
      m_ocTree->insertScan(**it);
      std::cout << " S ("<<currentScan<<"/"<<numScans<<") " << std::flush;
      currentScan++;
    }

    showOcTree();

    showInfo("Done.", true);
    QApplication::restoreOverrideCursor();
  }
  else {
    std::cerr << "generateOctree called but no ScanGraph present!\n";
  }

}

void ViewerGui::addNextScans(unsigned scans){
  for (unsigned i = 0; i < scans; ++i){
    addNextScan();
  }
}

void ViewerGui::gotoFirstScan(){
  if (m_scanGraph){
    showInfo("Inserting first scan node into tree... ", true);
    QApplication::setOverrideCursor(Qt::WaitCursor);

    m_nextScanToAdd = m_scanGraph->begin();
    
    if (m_ocTree) delete m_ocTree;
    m_ocTree = new octomap::OcTree(m_octreeResolution);
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
      m_ocTree->insertScan(**m_nextScanToAdd);
      m_nextScanToAdd++;
    }

    QApplication::restoreOverrideCursor();
    showOcTree();

  }
}


void ViewerGui::showOcTree() {

  if (m_ocTree) {
    if (!m_octreeDrawer){
      m_octreeDrawer = new OcTreeDrawer();
      m_glwidget->addSceneObject(m_octreeDrawer);
    }
    m_octreeDrawer->setMax_tree_depth(m_max_tree_depth);
    m_octreeDrawer->setOcTree(*m_ocTree);

    double minX, minY, minZ, maxX, maxY, maxZ;
    m_ocTree->getMetricMin(minX, minY, minZ);
    m_ocTree->getMetricMax(maxX, maxY, maxZ);

    m_glwidget->setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));

    double sizeX, sizeY, sizeZ;
    m_ocTree->getMetricSize(sizeX, sizeY, sizeZ);

    QString size = QString("%L1m x %L2m x %L3m").arg(sizeX).arg(sizeY).arg(sizeZ);
    unsigned memoryUsage = m_ocTree->memoryUsage();
    QString memory = QString("%L1 nodes; ").arg(m_ocTree->size())
      + QString ("%L1 B (%L2 MB)").arg(memoryUsage).arg((double) memoryUsage/(1024.*1024.), 0, 'f', 3);
    m_mapMemoryStatus->setText(memory);
    m_mapSizeStatus->setText(size);
    m_glwidget->updateGL();
  }
  else {
    QMessageBox::warning(this, "Tree not present",
			 "Trying to show OcTree but no tree present. This should not happen.",
			 QMessageBox::Ok);
  }
}



void ViewerGui::openFile(){
  
  if (m_filename != ""){

    m_glwidget->clearAll();

    QFileInfo fileinfo(QString::fromStdString(m_filename));
    if (fileinfo.suffix() == "graph"){
      openGraph();
    }
    else if (fileinfo.suffix() == "bt"){
      openTree();
    }
    else if (fileinfo.suffix() == "ot"){
      openOcTree();
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
  showInfo("Loading scan graph from file "+QString::fromStdString(m_filename)+"...");

  if (m_scanGraph) delete m_scanGraph;
  m_scanGraph = new octomap::ScanGraph();
  m_scanGraph->readBinary(m_filename);

  loadGraph(completeGraph);
}


void ViewerGui::openPointcloud(){
  
  QApplication::setOverrideCursor(Qt::WaitCursor);
  showInfo("Loading ASCII pointcloud from file "+QString::fromStdString(m_filename)+"...");

  if (m_scanGraph) delete m_scanGraph;
  m_scanGraph = new octomap::ScanGraph();


  // read pointcloud from file
  std::ifstream s(m_filename.c_str());
  Pointcloud pc;

  if (!s) {
    std::cout <<"ERROR: could not read " << m_filename << std::endl;
    return;
  }

  pc.read(s);

//  point3d p;
//  while (!s.eof()) {
//    for (unsigned int i=0; i<3; i++){
//      s >> p(i);
//    }
//    pc.push_back(p);
//  }

  pose6d laser_pose(0,0,0,0,0,0);
  m_scanGraph->addNode(&pc, laser_pose);

  loadGraph(true);
  showInfo("Done.", true);
}


void ViewerGui::openTree(){
  if (m_ocTree)
    delete m_ocTree;

  m_ocTree = new octomap::OcTree(m_filename);
  m_octreeResolution = m_ocTree->getResolution();
  emit changeResolution(m_octreeResolution);

  ui.actionPointcloud->setChecked(false);
  ui.actionPointcloud->setEnabled(false);
  ui.actionOctree_cells->setChecked(true);
  ui.actionOctree_cells->setEnabled(true);
  ui.actionFree->setChecked(false);
  ui.actionFree->setEnabled(true);
  ui.actionOctree_structure->setEnabled(true);
  ui.actionOctree_structure->setChecked(false);
  ui.actionTrajectory->setEnabled(false);
  ui.actionConvert_ml_tree->setEnabled(false);
  ui.actionReload_Octree->setEnabled(false);
  ui.actionSettings->setEnabled(false);

  showOcTree();
  m_glwidget->resetView();

}

void ViewerGui::openOcTree(){
  if (m_ocTree){
    delete m_ocTree;
    m_ocTree = NULL;
  }
  //
  //m_ocTree = OcTreeFileIO::read<octomap::OcTreeNode>(m_filename);
  octomap::OcTreeBase<octomap::OcTreeNode>* tree = OcTreeFileIO::read<octomap::OcTreeNode>(m_filename);
  if (tree){
    m_ocTree = dynamic_cast<OcTree*>(tree);
  }

  if (m_ocTree){
    m_octreeResolution = m_ocTree->getResolution();
    emit changeResolution(m_octreeResolution);

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
    ui.actionReload_Octree->setEnabled(false);
    ui.actionSettings->setEnabled(false);

    showOcTree();
    m_glwidget->resetView();
  } else {
    QMessageBox::warning(this, "File error", "Cannot open OcTree file", QMessageBox::Ok);
  }

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
    m_nextScanToAdd = m_scanGraph->end();
    generateOctree();
    currentScan = graphSize;
  } 
  else{
    m_nextScanToAdd = m_scanGraph->begin();
    if (m_ocTree) delete m_ocTree;
    m_ocTree = new octomap::OcTree(m_octreeResolution);
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

  if (m_ocTree)
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


  if (dialog.exec()){

    double oldResolution = m_octreeResolution;
    double oldType = m_laserType;

    m_octreeResolution = dialog.getResolution();
    m_laserType = dialog.getLaserType();
  
    // apply new settings
    bool resolutionChanged = (fabs (oldResolution - m_octreeResolution) > 1e-5);

    if (resolutionChanged)
      emit changeResolution(m_octreeResolution);

    if (oldType != m_laserType){ // parameters changed, reload file:
      openFile();
    } else if (resolutionChanged){
      generateOctree();
    }

  }
}

void ViewerGui::on_actionOpen_file_triggered(){
  QString filename = QFileDialog::getOpenFileName(this,
	      tr("Open data file"), "",
	  "All supported files (*.graph *.bt *.ot *.dat);;Binary scan graph (*.graph);;Bonsai tree (*.bt);;OcTree (*.ot);;Pointcloud (*.dat);;All files (*)");
  if (filename != ""){
    m_filename = filename.toStdString();
    openFile();
  }
}


void ViewerGui::on_actionOpen_graph_incremental_triggered(){
  QString filename = QFileDialog::getOpenFileName(this,
	      tr("Open graph file incrementally (at start)"), "",
	  "binary scan graph (*.graph)");
  if (filename != ""){
    m_glwidget->clearAll();

    m_filename = filename.toStdString();
    openGraph(false);
  }
}

void ViewerGui::on_actionSave_file_triggered(){

  if (m_ocTree) {
    QString filename = QFileDialog::getSaveFileName(this, tr("Save octree file"),
						    "", tr("Bonsai Tree file (*.bt);;Full OcTree (*.ot)"));

    if (filename != ""){
      QApplication::setOverrideCursor(Qt::WaitCursor);
      showInfo("Writing file... ", false);

      QFileInfo fileinfo(filename);
      if (fileinfo.suffix() == "bt")
        m_ocTree->writeBinaryConst(filename.toStdString());
      else if (fileinfo.suffix() == "ot"){
        OcTreeFileIO::write( m_ocTree, filename.toStdString());
        //writer.write((OcTreeBase<octomap::OcTreeDataNode<float> >*) m_ocTree, filename.toStdString());
        //m_ocTree->write(filename.toStdString());
      }
      else{
        QMessageBox::warning(this, "Unknown file", "Cannot write file, unknown extension: "+fileinfo.suffix(), QMessageBox::Ok);
      }

      QApplication::restoreOverrideCursor();
      showInfo("Done.", true);
    }

  } else{
    QMessageBox::warning(this, tr("3D Mapping Viewer"),
                         "Error: No OcTree present.",
                         QMessageBox::Ok);

  }


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
  if (checked)
    ui.actionHeight_map->setChecked(false);
  m_glwidget->enablePrintoutMode(checked);
}

void ViewerGui::on_actionHeight_map_toggled(bool checked){
  if (checked)
    ui.actionPrintout_mode->setChecked(false);

  m_glwidget->enableHeightColorMode(checked);
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

void ViewerGui::on_actionTest_triggered(){

}

void ViewerGui::on_actionReload_Octree_triggered(){
  generateOctree();
}

void ViewerGui::on_actionConvert_ml_tree_triggered(){

  QApplication::setOverrideCursor(Qt::WaitCursor);

  if (m_ocTree) {
    showInfo("Converting OcTree to maximum Likelihood map... ");
    m_ocTree->toMaxLikelihood();

    showOcTree();

    showInfo("Done.", true);
  }

  QApplication::restoreOverrideCursor();

}

void ViewerGui::on_actionPrune_tree_triggered(){

  QApplication::setOverrideCursor(Qt::WaitCursor);

  if (m_ocTree) {
    showInfo("Pruning OcTree... ");
    m_ocTree->prune();

    showOcTree();

    showInfo("Done.", true);
  }

  QApplication::restoreOverrideCursor();
}

void ViewerGui::on_actionExpand_tree_triggered(){

  QApplication::setOverrideCursor(Qt::WaitCursor);

    if (m_ocTree) {
      showInfo("Expanding OcTree... ");
      m_ocTree->expand();

      showOcTree();

      showInfo("Done.", true);
    }

    QApplication::restoreOverrideCursor();
}

void ViewerGui::on_actionOctree_cells_toggled(bool enabled) {
  if(m_octreeDrawer) {
    m_octreeDrawer->enableOcTreeCells(enabled);
    m_glwidget->updateGL();
  }
}

void ViewerGui::on_actionOctree_structure_toggled(bool enabled) {
  if(m_octreeDrawer) {
    m_octreeDrawer->enableOcTree(enabled);
    m_glwidget->updateGL();
  }
}

void ViewerGui::on_actionFree_toggled(bool enabled) {
  if(m_octreeDrawer) {
    m_octreeDrawer->enableFreespace(enabled);
    m_glwidget->updateGL();
  }
}

void ViewerGui::on_actionChanged_free_only_toggled(bool enabled) {
  if(m_octreeDrawer) {
    m_octreeDrawer->enableFreespaceDeltaOnly(enabled);
    m_glwidget->updateGL();
  }
}

}


