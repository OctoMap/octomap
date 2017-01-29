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

#ifndef VIEWERSETTINGSPANELFLYMODE_H
#define VIEWERSETTINGSPANELFLYMODE_H

#include <qglobal.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets/QWidget>
#else
#include <QtGui/QWidget>
#endif
#include "ui_ViewerSettingsPanelCamera.h"

class ViewerSettingsPanelCamera : public QWidget
{
    Q_OBJECT

public:
    ViewerSettingsPanelCamera(QWidget *parent = 0);
    ~ViewerSettingsPanelCamera();
    QSize sizeHint() const;

public slots:
    void setNumberOfFrames(unsigned frames);
    void setCurrentFrame(unsigned frame);
    void setRobotTrajectoryAvailable(bool available);
    void setStopped();

private slots:
    void on_firstScanButton_clicked();
    void on_lastScanButton_clicked();
    void on_nextScanButton_clicked();
    void on_previousScanButton_clicked();
    void on_playScanButton_clicked();
    void on_scanProgressSlider_sliderMoved(int value);
    void on_followCameraPathButton_clicked();
    void on_followTrajectoryButton_clicked();
    void on_cameraPathAdd_clicked();
    void on_cameraPathRemove_clicked();
    void on_cameraPathSave_clicked();
    void on_cameraPathClear_clicked();
    void positionEditDone(double);


signals:
	void changeCamPosition(double x, double y, double z, double lookX, double lookY, double lookZ);
	void jumpToFrame(unsigned int frame);
	void play();
	void pause();
	void clearCameraPath();
	void saveToCameraPath();
	void removeFromCameraPath();
	void addToCameraPath();
	void followCameraPath();
	void followRobotPath();

private:
    void dataChanged();
    void gotoFrame(unsigned int frame);
    bool followRobotTrajectory();
    Ui::ViewerSettingsPanelCameraClass ui;
    unsigned int m_currentFrame;
    unsigned int m_numberFrames;
    bool m_robotTrajectoryAvailable;
};

#endif // VIEWERSETTINGSPANELFLYMODE_H
