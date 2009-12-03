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
  : QMainWindow(parent), m_trajectoryDrawer(NULL),m_pointcloudDrawer(NULL),m_octreeResolution(0.1), m_occupancyThresh(0.5), m_max_tree_depth(16), m_laserType(LASERTYPE_SICK),
    m_cameraStored(false),m_filename("")
{
	ui.setupUi(this);
	m_glwidget = new ViewerWidget(this);
	this->setCentralWidget(m_glwidget);

	// Settings panel at the right side:
	ViewerSettingsPanel* settingsPanel = new ViewerSettingsPanel(this);
	QDockWidget* settingsDock = new QDockWidget("Settings panel", this);
	settingsDock->setWidget(settingsPanel);
	this->addDockWidget(Qt::RightDockWidgetArea, settingsDock);
	ui.menuShow->addAction(settingsDock->toggleViewAction());

	// status bar:
	m_mapSizeStatus = new QLabel("Map size", this);
	m_mapMemoryStatus = new QLabel("Memory consumption", this);
	m_mapSizeStatus->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	m_mapMemoryStatus->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	statusBar()->addPermanentWidget(m_mapSizeStatus);
	statusBar()->addPermanentWidget(m_mapMemoryStatus);



	connect(this, SIGNAL(updateStatusBar(QString, int)), statusBar(), SLOT(showMessage(QString, int)));

	connect(settingsPanel, SIGNAL(treeDepthChanged(int)), this, SLOT(changeTreeDepth(int)));
	connect(settingsPanel, SIGNAL(addNextScans(unsigned)), this, SLOT(addNextScans(unsigned)));
	connect(settingsPanel, SIGNAL(gotoFirstScan()), this, SLOT(gotoFirstScan()));
	connect(this, SIGNAL(changeNumberOfScans(unsigned)), settingsPanel, SLOT(setNumberOfScans(unsigned)));
	connect(this, SIGNAL(changeCurrentScan(unsigned)), settingsPanel, SLOT(setCurrentScan(unsigned)));
	connect(this, SIGNAL(changeResolution(double)), settingsPanel, SLOT(setResolution(double)));

	connect(settingsPanel, SIGNAL(changeCamPosition(double, double, double, double, double, double)), m_glwidget, SLOT(setCamPosition(double, double, double, double, double, double)));

	connect(ui.actionOctree_cells, SIGNAL(toggled(bool)), m_glwidget, SLOT(enableOcTreeCells(bool)));
	connect(ui.actionOctree_structure, SIGNAL(toggled(bool)), m_glwidget, SLOT(enableOcTree(bool)));
	connect(ui.actionFree, SIGNAL(toggled(bool)), m_glwidget, SLOT(enableFreespace(bool)));
	connect(ui.actionChanged_free_only, SIGNAL(toggled(bool)), m_glwidget, SLOT(enableFreespaceDeltaOnly(bool)));
	connect(ui.actionReset_view, SIGNAL(triggered()), m_glwidget, SLOT(resetView()));



	if (filename != ""){
	  m_filename = filename;
	  openFile();
	}
}

ViewerGui::~ViewerGui()
{
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


void ViewerGui::regenerateView(){
  // void the previously stored / cached OcTrees:
  m_ocTree.reset();
  generateDeltaOctree();
}


void ViewerGui::generateDeltaOctree() {

  if (m_scanGraph){

    QApplication::setOverrideCursor(Qt::WaitCursor);

    // tree is not set => re-generete
    if (!m_ocTree) {
      showInfo("Generating delta OcTree... ");

      m_ocTree.reset(new octomap::OcTree(m_octreeResolution));
      octomap::ScanGraph::iterator it;
      unsigned numScans = m_scanGraph->size();
      unsigned currentScan = 1;
      for (it = m_scanGraph->begin(); it != m_nextScanToAdd; it++) {
        m_ocTree->insertScan(**it);
        std::cout << " S ("<<currentScan<<"/"<<numScans<<") " << std::flush;
        currentScan++;
      }
    } else {
      showInfo("Reusing delta OcTree... ");
    }

    if (ui.actionPruned->isChecked()) {
      showInfo("pruning... ");
      m_ocTree->prune();
    }
    showOcTree();

    showInfo("Done.", true);
    QApplication::restoreOverrideCursor();
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
    m_ocTree.reset(new octomap::OcTree(m_octreeResolution));
    addNextScan();

    QApplication::restoreOverrideCursor();
    showOcTree();
  }

}

void ViewerGui::addNextScan(){
  if (m_scanGraph){
    showInfo("Inserting next scan node into tree... ", true);
    // OcTree is no longer pure (new scans are delta)
    ui.actionAs_pure_binary_OcTree->setChecked(false);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    if (m_nextScanToAdd != m_scanGraph->end()){
      m_ocTree->insertScan(**m_nextScanToAdd);
      m_nextScanToAdd++;
    }
    // re-prune when pruning activated
    if (ui.actionPruned->isChecked())
      m_ocTree->prune();

    QApplication::restoreOverrideCursor();
    showOcTree();

  }
}

void ViewerGui::generateBinaryOctree() {
  QApplication::setOverrideCursor(Qt::WaitCursor);

  if (m_ocTree) {
    showInfo("Purifying OcTree... ");
    m_ocTree->deltaToBinary();

    if (ui.actionPruned->isChecked()) {
      showInfo("pruning... ");
      m_ocTree->prune();
    }
    showOcTree();

    showInfo("Done.", true);
  }

  QApplication::restoreOverrideCursor();
}


void ViewerGui::showOcTree() {

  if (m_ocTree) {

    std::list<octomap::OcTreeVolume> occupied_voxels;
    std::list<octomap::OcTreeVolume> free_voxels;
    std::list<octomap::OcTreeVolume> grid_voxels;
    std::list<octomap::OcTreeVolume> occupied_delta_voxels;
    std::list<octomap::OcTreeVolume> free_delta_voxels;
    std::list<octomap::OcTreeVolume> free_changed_voxels;

    m_ocTree->getOccupied(m_max_tree_depth, m_occupancyThresh, occupied_voxels, occupied_delta_voxels);

    if (m_ocTree->size() < 5 * 1e6) {
      m_ocTree->getFreespace (m_max_tree_depth, m_occupancyThresh, free_voxels, free_delta_voxels);
      m_ocTree->getVoxels(m_max_tree_depth, grid_voxels);
      // FIXME m_ocTree->getChangedFreespace(m_max_tree_depth, m_occupancyThresh, free_changed_voxels);
    }

    double minX, minY, minZ, maxX, maxY, maxZ;
    m_ocTree->getMetricMin(minX, minY, minZ);
    m_ocTree->getMetricMax(maxX, maxY, maxZ);

    m_glwidget->setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));

    m_glwidget->setOcTreeVoxels(occupied_voxels,
				free_voxels,
				occupied_delta_voxels,
				free_delta_voxels,
				grid_voxels, free_changed_voxels);

    double sizeX, sizeY, sizeZ;
    m_ocTree->getMetricSize(sizeX, sizeY, sizeZ);

    QString size = QString("%L1m x %L2m x %L3m").arg(sizeX).arg(sizeY).arg(sizeZ);
    unsigned memoryUsage = m_ocTree->memoryUsage();
    QString memory = QString("%L1 nodes; ").arg(m_ocTree->size())
      + QString ("%L1 B (%L2 MB)").arg(memoryUsage).arg((double) memoryUsage/(1024.*1024.), 0, 'f', 3);
    m_mapMemoryStatus->setText(memory);
    m_mapSizeStatus->setText(size);



  }
  else {
    QMessageBox::warning(this, "Tree not present",
			 "Trying to show OcTree but no tree present. This should not happen.",
			 QMessageBox::Ok);
  }
}



void ViewerGui::openFile(){
  if (m_filename != ""){
    m_ocTree.reset();
    m_scanGraph.reset();
    m_glwidget->clearAll();


    QFileInfo fileinfo(QString::fromStdString(m_filename));
    if (fileinfo.suffix() == "graph"){
      openGraph();
    }
    else if (fileinfo.suffix() == "bt"){
      openTree();
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

  m_scanGraph = boost::shared_ptr<octomap::ScanGraph>(new octomap::ScanGraph());
  m_scanGraph->readBinary(m_filename);

  loadGraph(completeGraph);
}


void ViewerGui::openPointcloud(){
  
  QApplication::setOverrideCursor(Qt::WaitCursor);
  showInfo("Loading ASCII pointcloud from file "+QString::fromStdString(m_filename)+"...");

  m_scanGraph = boost::shared_ptr<octomap::ScanGraph>(new octomap::ScanGraph());


  // read pointcloud from file
  std::ifstream s(m_filename.c_str());
  Pointcloud pc;

  if (!s) {
    std::cout <<"ERROR: could not read " << m_filename << std::endl;
    return;
  }

  std::string tmp;
  point3d p;
  while (!s.eof()) {
    for (unsigned int i=0; i<3; i++){
      s >> p(i);   
    }
    pc.push_back(p);
  }    

  pose6d laser_pose(0,0,0,0,0,0);
  m_scanGraph->addNode(&pc, laser_pose);

  loadGraph(true);
  showInfo("Done.", true);
}


void ViewerGui::openTree(){

  // TODO: Move tree type probing (and filestreams) to convenience fct. in OcTree

  std::ifstream binary_test( m_filename.c_str(), std::ios_base::binary);
  int tree_type = -1;
  binary_test.read((char*)&tree_type, sizeof(tree_type));
  binary_test.close();

  if (tree_type == octomap::OcTree::TREETYPE){
    m_ocTree.reset(new octomap::OcTree(m_octreeResolution));
    m_ocTree->readBinary(m_filename);

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
    ui.actionAs_pure_binary_OcTree->setChecked(true);
    ui.actionAs_pure_binary_OcTree->setEnabled(false);
    ui.actionPruned->setChecked(true);
    ui.actionPruned->setEnabled(false);
    ui.actionSettings->setEnabled(false);

    showOcTree();
    m_glwidget->resetView();
  }

}


void ViewerGui::loadGraph(bool completeGraph){
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
  ui.actionAs_pure_binary_OcTree->setEnabled(true);
  ui.actionAs_pure_binary_OcTree->setChecked(false);
  ui.actionPruned->setEnabled(true);
  ui.actionPruned->setChecked(false);

  m_ocTree.reset();

  unsigned graphSize = m_scanGraph->size();
  unsigned currentScan;
  if (completeGraph){
    // generate delta by default
    m_nextScanToAdd = m_scanGraph->end();
    generateDeltaOctree();
    currentScan = graphSize;
  } else{
    m_nextScanToAdd = m_scanGraph->begin();
    m_ocTree.reset(new octomap::OcTree(m_octreeResolution));
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
        regenerateView();

        if (ui.actionAs_pure_binary_OcTree->isChecked()) {
          generateBinaryOctree();
        }
    }

  }
}

void ViewerGui::on_actionOpen_file_triggered(){
  QString filename = QFileDialog::getOpenFileName(this,
	      tr("Open log file"), "",
	  "All supported files (*.graph *.bt *.dat);;binary scan graph (*.graph);;bonsai tree (*.bt);;pointcloud (*.dat);;All files (*)");
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

  if (ui.actionAs_pure_binary_OcTree->isChecked() && m_ocTree) {
    QString filename = QFileDialog::getSaveFileName(this, tr("Save binary tree file"),
						    "", tr("Bonsai tree files (*.bt)"));

    if (filename != ""){
      QApplication::setOverrideCursor(Qt::WaitCursor);
      showInfo("Writing file... ", false);

      m_ocTree->writeBinary(filename.toStdString());

      QApplication::restoreOverrideCursor();
      showInfo("Done.", true);
    }

  } else{
    QMessageBox::warning(this, tr("3D Mapping Viewer"),
                                    "Error: No OcTree present, or OcTree is not pure binary. \nPlease convert the current tree first.",
                                    QMessageBox::Ok);

  }


}

void ViewerGui::on_actionExport_view_triggered(){
  m_glwidget->openSnapshotFormatDialog();
  m_glwidget->saveSnapshot(false);
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

void ViewerGui::on_actionAs_pure_binary_OcTree_triggered(bool checked){
  // binary has been enabled
  if (checked) {
    generateBinaryOctree();
  } else{
    m_ocTree.reset(); // "unpurification": complete re-generation
    generateDeltaOctree();

  }
}

void ViewerGui::on_actionPruned_triggered(bool checked){
  // "unpruning": completely re-generate the tree unpruned
  if (!checked)
    m_ocTree.reset(); // void previous (pruned) tree

  generateDeltaOctree();


  // should it also be converted to binary?
  if (ui.actionAs_pure_binary_OcTree->isChecked()) {
    generateBinaryOctree();
  }
}

}




