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

#include <octovis/ViewerSettingsPanelCamera.h>
#include <iostream>

ViewerSettingsPanelCamera::ViewerSettingsPanelCamera(QWidget *parent)
: QWidget(parent), m_currentFrame(1), m_numberFrames(0), m_robotTrajectoryAvailable(false)
{
  ui.setupUi(this);
  connect(ui.posX, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
  connect(ui.posY, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
  connect(ui.posZ, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
  connect(ui.lookX, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
  connect(ui.lookY, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));
  connect(ui.lookZ, SIGNAL(valueChanged(double)), this, SLOT(positionEditDone(double)));

  ui.followTrajectoryButton->setEnabled(m_robotTrajectoryAvailable);
  dataChanged();
}

ViewerSettingsPanelCamera::~ViewerSettingsPanelCamera()
{

}

QSize ViewerSettingsPanelCamera::sizeHint() const {
  return QSize(250, 180);
}

void ViewerSettingsPanelCamera::positionEditDone(double){
  emit changeCamPosition(ui.posX->value(), ui.posY->value(), ui.posZ->value(), ui.lookX->value(), ui.lookY->value(), ui.lookZ->value());
}

void ViewerSettingsPanelCamera::setNumberOfFrames(unsigned frames){
  m_numberFrames = frames;
  dataChanged();
}

void ViewerSettingsPanelCamera::setCurrentFrame(unsigned frame){
  m_currentFrame = frame;
  dataChanged();
}

void ViewerSettingsPanelCamera::setRobotTrajectoryAvailable(bool available) {
  m_robotTrajectoryAvailable = available;
  if(!available) ui.followTrajectoryButton->setChecked(false);
  ui.followTrajectoryButton->setEnabled(available);
}

void ViewerSettingsPanelCamera::gotoFrame(unsigned int frame) {
  if(frame > 0 && frame <= m_numberFrames) {
    m_currentFrame = frame;
    emit jumpToFrame(m_currentFrame);
    dataChanged();
  }
}

void ViewerSettingsPanelCamera::on_nextScanButton_clicked(){
  gotoFrame(m_currentFrame + 1);
}

void ViewerSettingsPanelCamera::on_previousScanButton_clicked(){
  gotoFrame(m_currentFrame - 1);
}

void ViewerSettingsPanelCamera::on_firstScanButton_clicked(){
  gotoFrame(1);
}

void ViewerSettingsPanelCamera::on_lastScanButton_clicked(){
  gotoFrame(m_numberFrames);
}

void ViewerSettingsPanelCamera::on_followCameraPathButton_clicked(){
  emit followCameraPath();
}

void ViewerSettingsPanelCamera::on_followTrajectoryButton_clicked(){
  emit followRobotPath();
}

void ViewerSettingsPanelCamera::on_cameraPathAdd_clicked(){
  emit addToCameraPath();
}

void ViewerSettingsPanelCamera::on_cameraPathRemove_clicked(){
  emit removeFromCameraPath();
}

void ViewerSettingsPanelCamera::on_cameraPathClear_clicked(){
  emit clearCameraPath();
}

void ViewerSettingsPanelCamera::on_cameraPathSave_clicked(){
  emit saveToCameraPath();
}

void ViewerSettingsPanelCamera::on_playScanButton_clicked(){
  if(ui.playScanButton->isChecked()) {
    ui.scanProgressSlider->setEnabled(false);
    ui.followGroupBox->setEnabled(false);
    emit play();
  } else {
    ui.scanProgressSlider->setEnabled(true);
    ui.followGroupBox->setEnabled(true);
    emit pause();
  }
  dataChanged();
}

void ViewerSettingsPanelCamera::on_scanProgressSlider_sliderMoved(int value) {
  gotoFrame(value);
}

void ViewerSettingsPanelCamera::setStopped(){
  ui.followGroupBox->setEnabled(true);
  ui.scanProgressSlider->setEnabled(true);
  ui.playScanButton->setChecked(false);
  dataChanged();
}


void ViewerSettingsPanelCamera::dataChanged(){
  unsigned int max = std::max(0,int(m_numberFrames));
  unsigned int cur = std::min(max, m_currentFrame);

  ui.scanProgressSlider->setMaximum(max);
  ui.scanProgressSlider->setMinimum(1);

  if(ui.playScanButton->isChecked()) {
    ui.firstScanButton->setEnabled(false);
    ui.nextScanButton->setEnabled(false);
    ui.previousScanButton->setEnabled(false);
    ui.lastScanButton->setEnabled(false);
  } else {
    if (m_currentFrame >= max){
      ui.nextScanButton->setEnabled(false);
      ui.playScanButton->setEnabled(false);
      ui.lastScanButton->setEnabled(false);
    } else {
      ui.nextScanButton->setEnabled(true);
      ui.playScanButton->setEnabled(true);
      ui.lastScanButton->setEnabled(true);
    }

    if (m_currentFrame < 2){
      ui.firstScanButton->setEnabled(cur > 0);
      ui.previousScanButton->setEnabled(false);
    } else{
      ui.firstScanButton->setEnabled(true);
      ui.previousScanButton->setEnabled(true);
    }

    if (max > 1) {
      ui.playScanButton->setEnabled(true);
    } else {
      ui.playScanButton->setEnabled(false);
    }
  }

  if(followRobotTrajectory() || ui.playScanButton->isChecked()) {
    ui.cameraPathAdd->setEnabled(false);
    ui.cameraPathRemove->setEnabled(false);
    ui.cameraPathSave->setEnabled(false);
    ui.cameraPathClear->setEnabled(false);
  } else {
    ui.cameraPathAdd->setEnabled(true);
    ui.cameraPathRemove->setEnabled(m_numberFrames > 0);
    ui.cameraPathSave->setEnabled(m_numberFrames > 0);
    ui.cameraPathClear->setEnabled(m_numberFrames > 0);
  }

  if(max > 0 && !ui.playScanButton->isChecked()) {
    ui.scanProgressSlider->setEnabled(true);
  } else {
    ui.scanProgressSlider->setEnabled(false);
  }

  ui.scanProgressSlider->setValue(cur);
  ui.scanProgressLabel->setText(QString("%1/%2").arg(cur).arg(max));

  // queue a redraw:
  ui.scanProgressSlider->update();

}

bool ViewerSettingsPanelCamera::followRobotTrajectory(){
  return ui.followTrajectoryButton->isChecked();
}


