#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"

#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/sparse_graph/Transform.h"
#include "camodocal/gpl/EigenQuaternionParameterization.h"
#include "camodocal/gpl/EigenUtils.h"
#include "camodocal/camera_models/CostFunctionFactory.h"

#include "ceres/ceres.h"

namespace camodocal
{

class StereoCalibration
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StereoCalibration();

    StereoCalibration(const std::string &yamlFile_left,
                      const std::string &yamlFile_right,
                      const cv::Size &boardSize,
                      float squareSize);

    void addChessboardData(const std::vector<cv::Point2f> &corners_left,
                           const std::vector<cv::Point2f> &corners_right);

    bool calibrate(void);

    void drawResults(std::vector<cv::Mat> &images) const;

  private:
    bool calibrateHelper();

    void optimize();

    cv::Size m_boardSize;
    float m_squareSize;

    CameraPtr m_camera_left;
    CameraPtr m_camera_right;

    cv::Mat ex_rvec, ex_tvec;
    std::vector<cv::Mat> rvecs, tvecs;

    std::vector<std::vector<cv::Point2f>> m_imagePoints_left;
    std::vector<std::vector<cv::Point2f>> m_imagePoints_right;
    std::vector<std::vector<cv::Point3f>> m_scenePoints;

    bool m_verbose;
};
}

#endif
