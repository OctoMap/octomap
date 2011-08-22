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

#include <octovis/ViewerSettingsPanel.h>

ViewerSettingsPanel::ViewerSettingsPanel(QWidget *parent)
    : QWidget(parent), m_currentScan(0), m_numberScans(0), m_treeDepth(_TREE_MAX_DEPTH), m_resolution(0.1)
{
	ui.setupUi(this);
	connect(ui.treeDepth, SIGNAL(valueChanged(int)), this, SLOT(setTreeDepth(int)));

	scanProgressChanged();
	leafSizeChanged();
}

ViewerSettingsPanel::~ViewerSettingsPanel()
{

}

void ViewerSettingsPanel::on_nextScanButton_clicked(){
  if (m_currentScan < m_numberScans){
    m_currentScan++;
    scanProgressChanged();
    emit addNextScans(1);
  }
}

void ViewerSettingsPanel::on_fastFwdScanButton_clicked(){
  unsigned increase = int(m_numberScans)-int(m_currentScan);
  if (increase > 5) increase = 5;
  m_currentScan += increase;
  scanProgressChanged();
  emit addNextScans(increase);
}

void ViewerSettingsPanel::on_lastScanButton_clicked(){
  unsigned increase = int(m_numberScans)-int(m_currentScan);
  m_currentScan += increase;
  scanProgressChanged();
  emit addNextScans(increase);
}

void ViewerSettingsPanel::on_firstScanButton_clicked(){
  m_currentScan = 1;
  scanProgressChanged();
  emit gotoFirstScan();
}

void ViewerSettingsPanel::scanProgressChanged(){
  if (int(m_numberScans) > 1)
    ui.scanProgressBar->setMaximum(int(m_numberScans));
  else 
    ui.scanProgressBar->setMaximum(1);

  if (m_currentScan == m_numberScans){
    ui.nextScanButton->setEnabled(false);
    ui.fastFwdScanButton->setEnabled(false);
    ui.lastScanButton->setEnabled(false);

  } else{
    ui.nextScanButton->setEnabled(true);
    ui.fastFwdScanButton->setEnabled(true);
    ui.lastScanButton->setEnabled(true);
  }

  if (m_currentScan < 2){
    ui.firstScanButton->setEnabled(false);
  } else{
    ui.firstScanButton->setEnabled(true);
  }

  ui.scanProgressBar->setValue(m_currentScan);
  // queue a redraw:
  ui.scanProgressBar->update();
}

void ViewerSettingsPanel::setNumberOfScans(unsigned scans){
  m_numberScans = scans;
  scanProgressChanged();
}

void ViewerSettingsPanel::setCurrentScan(unsigned scan){
  m_currentScan = scan;
  scanProgressChanged();
}

void ViewerSettingsPanel::setResolution(double resolution){
  m_resolution = resolution;
  leafSizeChanged();
}

void ViewerSettingsPanel::setTreeDepth(int depth){
  emit treeDepthChanged(depth);
  m_treeDepth = depth;
  leafSizeChanged();
}

void ViewerSettingsPanel::leafSizeChanged(){
  double leafSize = m_resolution * pow(2.0, (int) (_TREE_MAX_DEPTH-m_treeDepth));
  ui.leafSize->setText(QString::number(leafSize)+" m");
}
