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

#include <octovis/CameraFollowMode.h>
#include <iostream>

#define CAMERA_PATH_ID 13
#define ROBOT_TRAJECTORY_ID 14

CameraFollowMode::CameraFollowMode(octomap::ScanGraph *graph)
: QObject(), m_scan_graph(graph), m_current_scan(1), m_current_cam_frame(1), m_number_cam_frames(0),
  m_followRobotTrajectory(false) {
}

CameraFollowMode::~CameraFollowMode() {
}


void CameraFollowMode::jumpToFrame(unsigned int frame) {
  if(m_followRobotTrajectory) {
    if(frame <= m_scan_graph->size()) {
      m_current_scan = frame;
      octomath::Pose6D pose = m_scan_graph->getNodeByID(frame-1)->pose;
      emit changeCamPose(pose);
    }
  } else {
    m_current_cam_frame = frame;
    emit jumpToCamFrame(CAMERA_PATH_ID, m_current_cam_frame-1);
  }
}

void CameraFollowMode::play() {
  if(m_followRobotTrajectory) {
    emit deleteCameraPath(ROBOT_TRAJECTORY_ID);
    //emit appendCurrentToCameraPath(ROBOT_TRAJECTORY_ID);
    m_start_frame = m_current_scan;
    for(unsigned int i = m_start_frame; i <= m_scan_graph->size(); i++) {
      octomap::ScanNode* scanNode = m_scan_graph->getNodeByID(i-1);
      if (scanNode)
        emit appendToCameraPath(ROBOT_TRAJECTORY_ID, scanNode->pose);
      else{
        std::cerr << "Error in " << __FILE__ << ":" << __LINE__ <<" : invalid node ID "<< i-1 << std::endl;
      }
    }
    emit playCameraPath(ROBOT_TRAJECTORY_ID, 0);
  } else {
    m_start_frame = m_current_cam_frame;
    emit playCameraPath(CAMERA_PATH_ID, m_start_frame-1);
  }
}

void CameraFollowMode::pause() {
  emit stopCameraPath(CAMERA_PATH_ID);
  emit stopCameraPath(ROBOT_TRAJECTORY_ID);
}


void CameraFollowMode::setScanGraph(octomap::ScanGraph *graph) {
  m_scan_graph = graph;
  emit scanGraphAvailable(true);
}

void CameraFollowMode::cameraPathStopped(int id) {
  if(id == CAMERA_PATH_ID || id == ROBOT_TRAJECTORY_ID) {
    emit stopped();
  }
}

void CameraFollowMode::cameraPathFrameChanged(int id, int current_camera_frame) {
  if(m_followRobotTrajectory) {
    m_current_scan = m_start_frame + current_camera_frame;
    emit frameChanged(m_current_scan);
  } else {
    m_current_cam_frame = m_start_frame + current_camera_frame;
    emit frameChanged(m_current_cam_frame);
  }
}

void CameraFollowMode::clearCameraPath() {
  emit deleteCameraPath(CAMERA_PATH_ID);
  m_current_cam_frame = 1;
  m_number_cam_frames = 0;
  emit frameChanged(m_current_cam_frame);
  emit changeNumberOfFrames(m_number_cam_frames);
}

void CameraFollowMode::saveToCameraPath() {
  emit updateCameraPath(CAMERA_PATH_ID, m_current_cam_frame-1);
}

void CameraFollowMode::addToCameraPath() {
  emit addCurrentToCameraPath(CAMERA_PATH_ID, m_current_cam_frame-1);
  m_number_cam_frames++;
  if(m_number_cam_frames == 1) m_current_cam_frame = 1;
  else m_current_cam_frame++;
  emit frameChanged(m_current_cam_frame);
  emit changeNumberOfFrames(m_number_cam_frames);
}

void CameraFollowMode::removeFromCameraPath() {
  if(m_number_cam_frames>0) {
    emit removeFromCameraPath(CAMERA_PATH_ID, m_current_cam_frame-1);
    m_number_cam_frames--;
    emit changeNumberOfFrames(m_number_cam_frames);
  }
}

void CameraFollowMode::followCameraPath() {
  m_followRobotTrajectory = false;
  emit frameChanged(m_current_cam_frame);
  emit changeNumberOfFrames(m_number_cam_frames);
}

void CameraFollowMode::followRobotPath() {
  if(m_scan_graph) {
    m_followRobotTrajectory = true;
    emit frameChanged(m_current_scan);
    emit changeNumberOfFrames(m_scan_graph->size());
  }
}
