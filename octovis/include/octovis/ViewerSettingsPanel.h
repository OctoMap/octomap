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

#ifndef VIEWERSETTINGSPANEL_H
#define VIEWERSETTINGSPANEL_H

#include <math.h>
#include <QWidget>
#include "ui_ViewerSettingsPanel.h"

#define _TREE_MAX_DEPTH 16

class ViewerSettingsPanel : public QWidget
{
    Q_OBJECT

public:
    ViewerSettingsPanel(QWidget *parent = 0);
    ~ViewerSettingsPanel();

public slots:
    void setNumberOfScans(unsigned scans);
    void setCurrentScan(unsigned scan);
    void setResolution(double resolution);
    void setTreeDepth(int depth);

private slots:
    void on_firstScanButton_clicked();
    void on_lastScanButton_clicked();
    void on_nextScanButton_clicked();
    void on_fastFwdScanButton_clicked();

signals:
  void treeDepthChanged(int depth);
  void addNextScans(unsigned scans);
  void gotoFirstScan();


private:
    void scanProgressChanged();
    void leafSizeChanged();
    Ui::ViewerSettingsPanelClass ui;
    unsigned m_currentScan;
    unsigned m_numberScans;
    unsigned m_treeDepth;
    double m_resolution;
};

#endif // VIEWERSETTINGSPANEL_H
