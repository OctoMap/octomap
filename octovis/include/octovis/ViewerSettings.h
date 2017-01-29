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

#ifndef VIEWERSETTINGS_H
#define VIEWERSETTINGS_H

#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets/QDialog>
#else  // QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtGui/QDialog>
#endif // QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include "ui_ViewerSettings.h"

class ViewerSettings : public QDialog
{
    Q_OBJECT

public:
    ViewerSettings(QWidget *parent = 0);
    ~ViewerSettings();
    double getResolution(){return ui.resolution->value(); };
    void setResolution(double resolution){ui.resolution->setValue(resolution);};
    unsigned int getLaserType(){return ui.laserType->currentIndex(); };
    void setLaserType(int type){ui.laserType->setCurrentIndex(type); };
    double getMaxRange(){return ui.maxRange->value(); };
    void setMaxRange(double range){ui.maxRange->setValue(range); };

private:
    Ui::ViewerSettingsClass ui;
};

#endif // VIEWERSETTINGS_H
