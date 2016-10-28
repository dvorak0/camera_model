#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
} // namespace backward

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camodocal/chessboard/Chessboard.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/calib/CameraCalibration.h"
#include "camodocal/gpl/gpl.h"
int main(int argc, char **argv)
{
    cv::Size boardSize;
    float squareSize;
    std::string inputDir;
    std::string cameraModel;
    std::string cameraName;
    std::string prefix;
    std::string fileExtension;
    bool useOpenCV;
    bool viewResults;
    bool verbose;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(10), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(14), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(7.f), "Size of one square in mm")
        ("input,i", boost::program_options::value<std::string>(&inputDir)->default_value("test"), "Input directory containing chessboard images")
        ("prefix,p", boost::program_options::value<std::string>(&prefix)->default_value("frame"), "Prefix of images")
        ("file-extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".jpg"), "File extension of images")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("kannala-brandt"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-name", boost::program_options::value<std::string>(&cameraName)->default_value("camera"), "Name of camera")
        ("opencv", boost::program_options::bool_switch(&useOpenCV)->default_value(false), "Use OpenCV to detect corners")
        ("view-results", boost::program_options::bool_switch(&viewResults)->default_value(true), "View results")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(true), "Verbose output")
        ;
    // clang-format on

    boost::program_options::positional_options_description pdesc;
    pdesc.add("input", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    if (!boost::filesystem::exists(inputDir) && !boost::filesystem::is_directory(inputDir))
    {
        std::cerr << "# ERROR: Cannot find input directory " << inputDir << "." << std::endl;
        return 1;
    }

    camodocal::Camera::ModelType modelType;
    if (boost::iequals(cameraModel, "kannala-brandt"))
    {
        modelType = camodocal::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(cameraModel, "mei"))
    {
        modelType = camodocal::Camera::MEI;
    }
    else if (boost::iequals(cameraModel, "pinhole"))
    {
        modelType = camodocal::Camera::PINHOLE;
    }
    else if (boost::iequals(cameraModel, "scaramuzza"))
    {
        modelType = camodocal::Camera::SCARAMUZZA;
    }
    else
    {
        std::cerr << "# ERROR: Unknown camera model: " << cameraModel << std::endl;
        return 1;
    }

    switch (modelType)
    {
    case camodocal::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case camodocal::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case camodocal::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    case camodocal::Camera::SCARAMUZZA:
        std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
        break;
    }

    // look for images in input directory
    std::vector<std::string> imageFilenames;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();

        // check if prefix matches
        if (!prefix.empty())
        {
            if (filename.compare(0, prefix.length(), prefix) != 0)
            {
                continue;
            }
        }

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        imageFilenames.push_back(itr->path().string());

        if (verbose)
        {
            std::cerr << "# INFO: Adding " << imageFilenames.back() << std::endl;
        }
    }

    if (imageFilenames.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << imageFilenames.size() << std::endl;
    }

    cv::Mat image = cv::imread(imageFilenames.front(), -1);
    const cv::Size frameSize = image.size();
    if (true)
    {
        camodocal::CameraPtr m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cameraName + "_camera_calib.yaml");
        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            cv::Mat img = cv::imread(imageFilenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
            const int size = 200;
            const int degree = 90;
            const double f = 0.5 * size / std::tan(0.5 * degree / 180.0 * M_PI);
            Eigen::Matrix3d R[5];
            int dxy[][2] = {{0, 0},
                            {-1, 0},
                            {1, 0},
                            {0, -1},
                            {0, 1}};
            R[0].setIdentity();

            //left
            R[1] << 0, 0, -1,
                0, 1, 0,
                1, 0, 0;

            //right
            R[2] << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;

            R[3] << 1, 0, 0,
                0, 0, -1,
                0, 1, 0;

            R[4] << 1, 0, 0,
                0, 0, 1,
                0, -1, 0;

            cv::Mat mapx[5], mapy[5];
            for (int i = 0; i < 5; i++)
            {
                mapx[i] = cv::Mat{size, size, CV_32F};
                mapy[i] = cv::Mat{size, size, CV_32F};
            }
            for (int v = 0; v < size; v++)
                for (int u = 0; u < size; u++)
                {
                    Eigen::Vector3d P{(u - 0.5 * size) / f, (v - 0.5 * size) / f, 1};
                    for (int i = 0; i < 5; i++)
                    {
                        Eigen::Vector2d p;
                        m_camera->spaceToPlane(R[i] * P, p);
                        mapx[i].at<float>(v, u) = p.x();
                        mapy[i].at<float>(v, u) = p.y();
                    }
                }
            cv::imshow("transform", img);
            cv::Mat s_img{size * 3, size * 3, CV_8U};
            //{size * 3, size * 3, CV_8U};
            for (int i = 0; i < 5; i++)
            {
                cv::Mat tmp;
                cv::remap(img, tmp, mapx[i], mapy[i], cv::INTER_LINEAR);
                puts("write");
                tmp.copyTo(s_img(cv::Rect(dxy[i][0] * size + size, dxy[i][1] * size + size, size, size))); // = tmp;
                printf("%d %d\n", s_img.type(), tmp.type());
                cv::imshow("sub", tmp);
                cv::imshow("undistort", s_img);
            }
            cv::waitKey(0);
        }
        return 0;
    }

    camodocal::CameraCalibration calibration(modelType, cameraName, frameSize, boardSize, squareSize);
    calibration.setVerbose(verbose);

    std::vector<bool> chessboardFound(imageFilenames.size(), false);
    for (size_t i = 0; i < imageFilenames.size(); ++i)
    {
        image = cv::imread(imageFilenames.at(i), -1);

        camodocal::Chessboard chessboard(boardSize, image);

        chessboard.findCorners(useOpenCV);
        if (chessboard.cornersFound())
        {
            if (verbose)
            {
                std::cerr << "# INFO: Detected chessboard in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
            }

            calibration.addChessboardData(chessboard.getCorners());

            cv::Mat sketch;
            chessboard.getSketch().copyTo(sketch);

            cv::imshow("Image", sketch);
            cv::waitKey(50);
        }
        else if (verbose)
        {
            std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
        }
        chessboardFound.at(i) = chessboard.cornersFound();
    }
    cv::destroyWindow("Image");

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    double startTime = camodocal::timeInSeconds();

    calibration.calibrate();
    calibration.writeParams(cameraName + "_camera_calib.yaml");
    calibration.writeChessboardData(cameraName + "_chessboard_data.dat");

    if (verbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                  << " sec.\n";
    }

    if (verbose)
    {
        std::cerr << "# INFO: Wrote calibration file to " << cameraName + "_camera_calib.yaml" << std::endl;
    }

    if (viewResults)
    {
        std::vector<cv::Mat> cbImages;
        std::vector<std::string> cbImageFilenames;

        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            if (!chessboardFound.at(i))
            {
                continue;
            }
            cbImages.push_back(cv::imread(imageFilenames.at(i), -1));
            cbImageFilenames.push_back(imageFilenames.at(i));

            // visualize observed and reprojected points
            calibration.drawResults(cbImages);

            for (size_t i = 0; i < cbImages.size(); ++i)
            {
                cv::putText(cbImages.at(i), cbImageFilenames.at(i), cv::Point(10, 20),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                            1, CV_AA);
                cv::imshow("Image", cbImages.at(i));
                cv::waitKey(0);
            }
        }
    }

    return 0;
}
