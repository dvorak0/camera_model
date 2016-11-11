#include "camodocal/calib/StereoCalibration.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/CataCamera.h"
#include <opencv2/opencv.hpp>
namespace camodocal
{

StereoCalibration::StereoCalibration()
    : m_boardSize(cv::Size(0, 0)), m_squareSize(0.0f), m_verbose(false)
{
}

StereoCalibration::StereoCalibration(const std::string &yamlFile_left,
                                     const std::string &yamlFile_right,
                                     const cv::Size &boardSize,
                                     float squareSize)
    : m_boardSize(boardSize), m_squareSize(squareSize), m_verbose(true)
{
    m_camera_left = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(yamlFile_left);
    m_camera_right = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(yamlFile_right);
}

void StereoCalibration::addChessboardData(const std::vector<cv::Point2f> &corners_left,
                                          const std::vector<cv::Point2f> &corners_right)
{
    m_imagePoints_left.push_back(corners_left);
    m_imagePoints_right.push_back(corners_right);

    std::vector<cv::Point3f> scenePointsInView;
    for (int i = 0; i < m_boardSize.height; ++i)
    {
        for (int j = 0; j < m_boardSize.width; ++j)
        {
            scenePointsInView.push_back(cv::Point3f(i * m_squareSize, j * m_squareSize, 0.0));
        }
    }
    m_scenePoints.push_back(scenePointsInView);
}

bool StereoCalibration::calibrate(void)
{
    // compute intrinsic camera parameters and extrinsic parameters for each of the views

    bool ret = calibrateHelper();

    ex_rvec = cv::Mat::zeros(3, 1, CV_64F);
    ex_tvec = cv::Mat::zeros(3, 1, CV_64F);

    // STEP 3: optimization using ceres
    optimize();

    if (m_verbose)
    {
        puts("reproject");
        double err = m_camera_left->reprojectionError(m_scenePoints, m_imagePoints_left, rvecs, tvecs);
        std::cout << "[" << m_camera_left->cameraName() << "] "
                  << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;

        err = m_camera_right->reprojectionError(m_scenePoints, m_imagePoints_right, rvecs, tvecs);
        std::cout << "[" << m_camera_right->cameraName() << "] "
                  << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
    }

    return ret;
}

void StereoCalibration::drawResults(std::vector<cv::Mat> &images) const
{
    //    std::vector<cv::Mat> rvecs, tvecs;
    //
    //    for (size_t i = 0; i < images.size(); ++i)
    //    {
    //        cv::Mat rvec(3, 1, CV_64F);
    //        rvec.at<double>(0) = m_cameraPoses.at<double>(i, 0);
    //        rvec.at<double>(1) = m_cameraPoses.at<double>(i, 1);
    //        rvec.at<double>(2) = m_cameraPoses.at<double>(i, 2);
    //
    //        cv::Mat tvec(3, 1, CV_64F);
    //        tvec.at<double>(0) = m_cameraPoses.at<double>(i, 3);
    //        tvec.at<double>(1) = m_cameraPoses.at<double>(i, 4);
    //        tvec.at<double>(2) = m_cameraPoses.at<double>(i, 5);
    //
    //        rvecs.push_back(rvec);
    //        tvecs.push_back(tvec);
    //    }
    //
    //    int drawShiftBits = 4;
    //    int drawMultiplier = 1 << drawShiftBits;
    //
    //    cv::Scalar green(0, 255, 0);
    //    cv::Scalar red(0, 0, 255);
    //
    //    for (size_t i = 0; i < images.size(); ++i)
    //    {
    //        cv::Mat &image = images.at(i);
    //        if (image.channels() == 1)
    //        {
    //            cv::cvtColor(image, image, CV_GRAY2RGB);
    //        }
    //
    //        std::vector<cv::Point2f> estImagePoints;
    //        m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i),
    //                                estImagePoints);
    //
    //        float errorSum = 0.0f;
    //        float errorMax = std::numeric_limits<float>::min();
    //
    //        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
    //        {
    //            cv::Point2f pObs = m_imagePoints.at(i).at(j);
    //            cv::Point2f pEst = estImagePoints.at(j);
    //
    //            cv::circle(image,
    //                       cv::Point(cvRound(pObs.x * drawMultiplier),
    //                                 cvRound(pObs.y * drawMultiplier)),
    //                       5, green, 2, CV_AA, drawShiftBits);
    //
    //            cv::circle(image,
    //                       cv::Point(cvRound(pEst.x * drawMultiplier),
    //                                 cvRound(pEst.y * drawMultiplier)),
    //                       5, red, 2, CV_AA, drawShiftBits);
    //
    //            float error = cv::norm(pObs - pEst);
    //
    //            errorSum += error;
    //            if (error > errorMax)
    //            {
    //                errorMax = error;
    //            }
    //        }
    //
    //        std::ostringstream oss;
    //        oss << "Reprojection error: avg = " << errorSum / m_imagePoints.at(i).size()
    //            << "   max = " << errorMax;
    //
    //        cv::putText(image, oss.str(), cv::Point(10, image.rows - 10),
    //                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
    //                    1, CV_AA);
    //    }
}

bool StereoCalibration::calibrateHelper()
{
    rvecs.assign(m_scenePoints.size(), cv::Mat());
    tvecs.assign(m_scenePoints.size(), cv::Mat());

    //std::vector<cv::Mat> rvecs_right(m_scenePoints.size(), cv::Mat());
    //std::vector<cv::Mat> tvecs_right(m_scenePoints.size(), cv::Mat());

    // STEP 1: Estimate intrinsics
    //for (size_t i = 0; i < m_scenePoints.size(); i++)
    //    for (size_t j = 0; j < m_scenePoints.at(i).size(); j++)
    //    {
    //        printf("%f %f %f %f %f\n", m_scenePoints.at(i).at(j).x,
    //               m_scenePoints.at(i).at(j).y,
    //               m_scenePoints.at(i).at(j).z,
    //               m_imagePoints_left.at(i).at(j).x,
    //               m_imagePoints_left.at(i).at(j).y);
    //    }
    m_camera_left->estimateIntrinsics(m_boardSize, m_scenePoints, m_imagePoints_left);
    //m_camera_right->estimateIntrinsics(m_boardSize, m_scenePoints, m_imagePoints_right);
    //std::cout << m_camera_left->parametersToString() << std::endl;

    puts("STEP 2: Estimate extrinsics");
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        m_camera_left->estimateExtrinsics(m_scenePoints.at(i), m_imagePoints_left.at(i), rvecs.at(i), tvecs.at(i));
        //m_camera_right->estimateExtrinsics(m_scenePoints.at(i), m_imagePoints_right.at(i), rvecs_right.at(i), tvecs_right.at(i));
    }

    if (m_verbose)
    {
        cv::Mat perViewErrors;
        std::cout << "[" << m_camera_left->cameraName() << "] "
                  << "# INFO: "
                  << "Initial reprojection error: "
                  << std::fixed << std::setprecision(3)
                  << m_camera_left->reprojectionError(m_scenePoints, m_imagePoints_left, rvecs, tvecs, perViewErrors)
                  << " pixels" << std::endl;

        //std::cout << "[" << m_camera_right->cameraName() << "] "
        //          << "# INFO: "
        //          << "Initial reprojection error: "
        //          << std::fixed << std::setprecision(3)
        //          << m_camera_right->reprojectionError(m_scenePoints, m_imagePoints_right, rvecs_right, tvecs_right, perViewErrors)
        //          << " pixels" << std::endl;
    }

    return true;
}

void StereoCalibration::optimize()
{
    std::cout << m_camera_left->parametersToString() << std::endl;
    std::cout << m_camera_right->parametersToString() << std::endl;

    // Use ceres to do optimization
    ceres::Problem problem;
    ceres::Problem problem2;

    std::vector<Transform, Eigen::aligned_allocator<Transform>> transformVec(rvecs.size());

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::Vector3d rvec;
        cv::cv2eigen(rvecs.at(i), rvec);

        transformVec.at(i).rotation() = Eigen::AngleAxisd(rvec.norm(), rvec.normalized());
        transformVec.at(i).translation() << tvecs[i].at<double>(0),
            tvecs[i].at<double>(1),
            tvecs[i].at<double>(2);
        //transformVec.at(i).rotation().setIdentity();
        //transformVec.at(i).translation().setZero();
    }

    Eigen::Matrix3d R;
    R << -1, 0, 0,
        0, 1, 0,
        0, 0, -1;

    //transformVec.at(0).rotation().w() = 0.519409;
    //transformVec.at(0).rotation().x() = -0.493348;
    //transformVec.at(0).rotation().y() = -0.483286;
    //transformVec.at(0).rotation().z() = -0.503246;
    //transformVec.at(0).translation().x() = -14.412781;
    //transformVec.at(0).translation().y() = -111.382226;
    //transformVec.at(0).translation().z() = -14.412781;

    Transform ex_transformVec;
    ex_transformVec.rotation() = R;
    ex_transformVec.translation() = Eigen::Vector3d{0, 0, -20};

    std::vector<double> intrinsicCameraParams_left;
    m_camera_left->writeParameters(intrinsicCameraParams_left);

    std::vector<double> intrinsicCameraParams_right;
    m_camera_right->writeParameters(intrinsicCameraParams_right);

    // create residuals for each observation
    for (size_t i = 0; i < m_imagePoints_left.size(); ++i)
    {
        for (size_t j = 0; j < m_imagePoints_left.at(i).size(); ++j)
        {
            const cv::Point3f &spt = m_scenePoints.at(i).at(j);

            {
                const cv::Point2f &ipt_left = m_imagePoints_left.at(i).at(j);
                ceres::CostFunction *costFunction =
                    CostFunctionFactory::instance()->generateCostFunction(m_camera_left,
                                                                          Eigen::Vector3d(spt.x, spt.y, spt.z),
                                                                          Eigen::Vector2d(ipt_left.x, ipt_left.y),
                                                                          CAMERA_INTRINSICS | CAMERA_POSE);

                ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
                problem.AddResidualBlock(costFunction, lossFunction,
                                         intrinsicCameraParams_left.data(),
                                         transformVec.at(i).rotationData(),
                                         transformVec.at(i).translationData());
            }
        }
        ceres::LocalParameterization *quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(transformVec.at(i).rotationData(),
                                    quaternionParameterization);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.num_threads = 1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "end ceres" << std::endl;

    if (m_verbose)
    {
        std::cout << summary.FullReport() << std::endl;
    }

    for (size_t i = 0; i < m_imagePoints_left.size(); ++i)
    {
        for (size_t j = 0; j < m_imagePoints_left.at(i).size(); ++j)
        {
            const cv::Point3f &spt = m_scenePoints.at(i).at(j);
            const cv::Point2f &ipt_left = m_imagePoints_left.at(i).at(j);
            const cv::Point2f &ipt_right = m_imagePoints_right.at(i).at(j);
            ceres::CostFunction *costFunction =
                CostFunctionFactory::instance()->generateCostFunction(m_camera_left, m_camera_right,
                                                                      Eigen::Vector3d(spt.x, spt.y, spt.z),
                                                                      Eigen::Vector2d(ipt_left.x, ipt_left.y), Eigen::Vector2d(ipt_right.x, ipt_right.y));

            ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
            double *para[] = {intrinsicCameraParams_left.data(),
                              intrinsicCameraParams_right.data(),
                              transformVec.at(i).rotationData(),
                              transformVec.at(i).translationData(),
                              ex_transformVec.rotationData(),
                              ex_transformVec.translationData()};

            double residual[4];

            costFunction->Evaluate(para, residual, NULL);
            printf("%lu %lu %f %f %f %f\n", i, j, residual[0], residual[1], residual[2], residual[3]);
            problem2.AddResidualBlock(costFunction, lossFunction,
                                      intrinsicCameraParams_left.data(),
                                      intrinsicCameraParams_right.data(),
                                      transformVec.at(i).rotationData(),
                                      transformVec.at(i).translationData(),
                                      ex_transformVec.rotationData(),
                                      ex_transformVec.translationData());
        }
    }

    ceres::LocalParameterization *quaternionParameterization =
        new EigenQuaternionParameterization;
    problem2.SetParameterization(ex_transformVec.rotationData(),
                                 quaternionParameterization);

    //problem2.SetParameterBlockConstant(intrinsicCameraParams_right.data());
    //problem2.SetParameterBlockConstant(ex_transformVec.rotationData());
    //problem2.SetParameterBlockConstant(ex_transformVec.translationData());

    ceres::Solve(options, &problem2, &summary);
    std::cout << summary.FullReport() << std::endl;

    m_camera_left->readParameters(intrinsicCameraParams_left);
    m_camera_right->readParameters(intrinsicCameraParams_right);
    std::cout << m_camera_left->parametersToString() << std::endl;
    std::cout << m_camera_right->parametersToString() << std::endl;
    m_camera_left->writeParametersToYamlFile("/home/dji/up.yaml");
    m_camera_right->writeParametersToYamlFile("/home/dji/down.yaml");

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::AngleAxisd aa(transformVec.at(i).rotation());

        Eigen::Vector3d rvec = aa.angle() * aa.axis();
        cv::eigen2cv(rvec, rvecs.at(i));

        cv::Mat &tvec = tvecs.at(i);
        tvec.at<double>(0) = transformVec.at(i).translation()(0);
        tvec.at<double>(1) = transformVec.at(i).translation()(1);
        tvec.at<double>(2) = transformVec.at(i).translation()(2);

        Eigen::Quaterniond q_r = Eigen::Quaterniond(ex_transformVec.rotationData()) * Eigen::Quaterniond(transformVec.at(i).rotationData());

        Eigen::Vector3d t_r;
        t_r(0) = transformVec.at(i).translation().x();
        t_r(1) = transformVec.at(i).translation().y();
        t_r(2) = transformVec.at(i).translation().z();

        t_r = Eigen::Quaterniond(ex_transformVec.rotationData()) * t_r;
        t_r(0) += ex_transformVec.translation().x();
        t_r(1) += ex_transformVec.translation().y();
        t_r(2) += ex_transformVec.translation().z();
        Eigen::Vector2d predicted_p_r;
        cv::Mat color_image{1280, 1024, CV_8UC3};
        color_image.setTo(cv::Scalar(0, 0, 0));

        for (size_t j = 0; j < m_scenePoints.at(i).size(); j++)
        {
            printf("%lu %lu\n", i, j);
            Eigen::Vector3d P(m_scenePoints.at(i).at(j).x,
                              m_scenePoints.at(i).at(j).y,
                              m_scenePoints.at(i).at(j).z);
            Eigen::Vector2d p(m_imagePoints_right.at(i).at(j).x,
                              m_imagePoints_right.at(i).at(j).y);

            camodocal::EquidistantCamera::spaceToPlane(intrinsicCameraParams_right.data(), q_r.coeffs().data(), t_r.data(), P, predicted_p_r);
            cv::circle(color_image, cv::Point2f(p.x(), p.y()), 3, cv::Scalar(255, 0, 0), 3);
            cv::circle(color_image, cv::Point2f(predicted_p_r.x(), predicted_p_r.y()), 3, cv::Scalar(0, 255, 0), 3);
            printf("%lu %lu: %f\n", i, j, (p - predicted_p_r).norm());
        }
        cv::imshow("color", color_image);
        cv::waitKey(0);
    }
    Eigen::AngleAxisd aa(ex_transformVec.rotation());

    Eigen::Vector3d rvec = aa.angle() * aa.axis();
    cv::eigen2cv(rvec, ex_rvec);

    ex_tvec.at<double>(0) = ex_transformVec.translation()(0);
    ex_tvec.at<double>(1) = ex_transformVec.translation()(1);
    ex_tvec.at<double>(2) = ex_transformVec.translation()(2);
    std::cout << ex_transformVec.rotation().toRotationMatrix() << std::endl;
    std::cout << ex_transformVec.translation() << std::endl;
    cv::FileStorage fs("/home/dji/ex.yaml", cv::FileStorage::WRITE);
    fs << "qw" << ex_transformVec.rotation().w()
       << "qx" << ex_transformVec.rotation().x()
       << "qy" << ex_transformVec.rotation().y()
       << "qz" << ex_transformVec.rotation().z()
       << "tx" << ex_transformVec.translation().x()
       << "ty" << ex_transformVec.translation().y()
       << "tz" << ex_transformVec.translation().z();
    fs.release();
}
}
