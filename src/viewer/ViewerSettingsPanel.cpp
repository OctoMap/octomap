#include "ViewerSettingsPanel.h"

ViewerSettingsPanel::ViewerSettingsPanel(QWidget *parent)
    : QWidget(parent), m_currentScan(0), m_numberScans(0), m_treeDepth(_TREE_MAX_DEPTH), m_resolution(0.1)
{
	ui.setupUi(this);
	connect(ui.treeDepth, SIGNAL(valueChanged(int)), this, SLOT(setTreeDepth(int)));
	connect(ui.posX, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
	connect(ui.posY, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
	connect(ui.posZ, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
	connect(ui.lookX, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
	connect(ui.lookY, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
	connect(ui.lookZ, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));

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
  unsigned increase = std::min(5, int(m_numberScans)-int(m_currentScan));
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
  ui.scanProgressBar->setMaximum(std::max(1,int(m_numberScans)));

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

void ViewerSettingsPanel::positionEditDone(double){
  emit changeCamPosition(ui.posX->value(), ui.posY->value(), ui.posZ->value(), ui.lookX->value(), ui.lookY->value(), ui.lookZ->value());
}

void ViewerSettingsPanel::leafSizeChanged(){
  double leafSize = m_resolution * pow(2,_TREE_MAX_DEPTH-m_treeDepth);
  ui.leafSize->setText(QString::number(leafSize)+" m");

}
