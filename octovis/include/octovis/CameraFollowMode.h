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

#ifndef CAMERAFOLLOWMODE_H_
#define CAMERAFOLLOWMODE_H_

#include "SceneObject.h"

class CameraFollowMode : public QObject {
	Q_OBJECT

public:
	CameraFollowMode(octomap::ScanGraph *graph = NULL);
	virtual ~CameraFollowMode();
	void setScanGraph(octomap::ScanGraph *graph);

public slots:
	void jumpToFrame(unsigned int frame);
	void cameraPathStopped(int id);
	void cameraPathFrameChanged(int id, int current_camera_frame);
	void play();
	void pause();
	void clearCameraPath();
	void saveToCameraPath();
	void addToCameraPath();
	void removeFromCameraPath();
	void followCameraPath();
	void followRobotPath();

signals:
	void changeCamPose(const octomath::Pose6D& pose);
	void interpolateCamPose(const octomath::Pose6D& old_pose, const octomath::Pose6D& new_pose, double u);
	void stopped();
	void frameChanged(unsigned int frame);
	void deleteCameraPath(int id);
	void removeFromCameraPath(int id, int frame);
	void updateCameraPath(int id, int frame);
    void appendToCameraPath(int id, const octomath::Pose6D& pose);
    void appendCurrentToCameraPath(int id);
    void addCurrentToCameraPath(int id, int frame);
    void playCameraPath(int id, int start_frame);
    void stopCameraPath(int id);
    void jumpToCamFrame(int id, int frame);
    void changeNumberOfFrames(unsigned count);
    void scanGraphAvailable(bool available);


protected:
	octomap::ScanGraph *m_scan_graph;
	unsigned int m_current_scan;
	unsigned int m_current_cam_frame;
	unsigned int m_number_cam_frames;
	unsigned int m_start_frame;
	bool m_followRobotTrajectory;
};

#endif /* CAMERAFOLLOWMODE_H_ */
