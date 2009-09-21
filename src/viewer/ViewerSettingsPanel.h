#ifndef VIEWERSETTINGSPANEL_H
#define VIEWERSETTINGSPANEL_H

#include <math.h>
#include <QGLViewer/qglviewer.h>
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
    void positionEditDone(double);

signals:
  void treeDepthChanged(int depth);
  void addNextScans(unsigned scans);
  void gotoFirstScan();
  void changeCamPosition(double x, double y, double z, double lookX, double lookY, double lookZ);


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
