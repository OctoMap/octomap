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

#ifndef VIEWERSETTINGSPANEL_H
#define VIEWERSETTINGSPANEL_H

#include <math.h>
#include <qglviewer.h>
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

private slots:
    void on_firstScanButton_clicked();
    void on_lastScanButton_clicked();
    void on_nextScanButton_clicked();
    void on_fastFwdScanButton_clicked();
    void setTreeDepth(int depth);

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
