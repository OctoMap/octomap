#ifndef VIEWERSETTINGS_H
#define VIEWERSETTINGS_H

#include <QtGui/QDialog>
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
    double getOccupancyThresh(){return ui.threshold->value(); };
    void setOccupancyThresh(double thresh){ui.threshold->setValue(thresh);};

private:
    Ui::ViewerSettingsClass ui;
};

#endif // VIEWERSETTINGS_H
