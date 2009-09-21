/*
 * ViewerGui.h
 *
 *  Created on: May 28, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#ifndef VIEWERGUI_H
#define VIEWERGUI_H

#include <QtGui/QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QDockWidget>
#include <string>
#include "ViewerWidget.h"
#include "ViewerSettings.h"
#include "ViewerSettingsPanel.h"
#include "ui_ViewerGui.h"

class ViewerGui : public QMainWindow {
  Q_OBJECT

 public:
  ViewerGui(const std::string& filename="", QWidget *parent = 0);
  ~ViewerGui();

  static const unsigned int LASERTYPE_URG  = 0;
  static const unsigned int LASERTYPE_SICK = 1;


 public slots:
  void changeTreeDepth(int depth);
  void addNextScans(unsigned scans);
  void gotoFirstScan();


 private slots:

  // auto-connected Slots (by name))

  void on_actionExit_triggered();
  void on_actionOpen_file_triggered();
  void on_actionOpen_graph_incremental_triggered();
  void on_actionSave_file_triggered();
  void on_actionExport_view_triggered();
  void on_actionHelp_triggered();
  void on_actionSettings_triggered();
  void on_actionPruned_triggered(bool checked);
  void on_actionAs_pure_binary_OcTree_triggered(bool checked);
  void on_actionPrintout_mode_toggled(bool checked);
  void on_actionHeight_map_toggled(bool checked);
  void on_actionStore_camera_triggered();
  void on_actionRestore_camera_triggered();

  // use it for testcases etc.
  void on_actionTest_triggered();

  signals:
   void updateStatusBar(QString message, int duration);
   void changeNumberOfScans(unsigned scans);
   void changeCurrentScan(unsigned scans);
   void changeResolution(double resolution);

 private:
  /**
   * (Re-)load the data file stored in m_fileName.
   * Depending on the extension, the respective load function is used.
   */
  void openFile();

  /**
   * Reads in a .log file, generates and optimizes a ScanGraph. Afterwards,
   * loadGraph() is called.
   */
  void openLog();

  /**
   * Opens a .graph file and generates a ScanGraph from it. Afterwards,
   * loadGraph() is called.
   */
  void openGraph(bool completeGraph = true);

  /**
   * Finishes loading a ScanGraph, either from .log or .graph.
   */
  void loadGraph(bool completeGraph = true);

  /**
   * Adds a scan from the graph to the OcTree
   */
  void addNextScan();

  /**
   * Opens a .pc PointCloud
   */
  void openPC();


  // open binary file containing an octree
  void openTree();

  /**
   * (Re)-generates the currently viewed OcTree implementation,
   * depending on which one is currently active (e.g. after a
   * resolution change)
   */
  void regenerateView();

  /*!
   * (Re-)generates delta OcTree from internally stored ScanGraph
   */
  void generateDeltaOctree();

  /*!
   * (Re-)generates OcTree
   */
  void generateBinaryOctree();
  void showOcTree();

  void showInfo(QString string, bool newline=false);


  boost::shared_ptr<octomap::ScanGraph> m_scanGraph;
  boost::shared_ptr<octomap::OcTree> m_ocTree;
  octomap::ScanGraph::iterator m_nextScanToAdd;

  Ui::ViewerGuiClass ui;
  ViewerWidget* m_glwidget;
  double m_octreeResolution;
  double m_occupancyThresh;
  unsigned int m_max_tree_depth;
  unsigned int m_laserType; // SICK or Hokuyo /URG
  bool m_cameraStored;
  QLabel* m_mapSizeStatus;
  QLabel* m_mapMemoryStatus;

  //! Filename of last loaded file, in case it is necessary to reload it (no matter what kind of log)
  std::string m_filename;

};

#endif // VIEWERGUI_H
