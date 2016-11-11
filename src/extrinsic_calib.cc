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
#include "camodocal/calib/StereoCalibration.h"
#include "camodocal/gpl/gpl.h"

cv::Size boardSize;
float squareSize;
std::string inputDir_left;
std::string inputDir_right;
std::string cameraModel;
std::string cameraName_left;
std::string cameraName_right;
std::string prefix;
std::string fileExtension;
bool useOpenCV;
bool viewResults;
bool verbose;
bool rectification;

std::vector<std::string> getImageFileNames(const std::string &dir)
{
    std::vector<std::string> imageFilenames;
    for (boost::filesystem::directory_iterator itr(dir); itr != boost::filesystem::directory_iterator(); ++itr)
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

    std::sort(std::begin(imageFilenames), std::end(imageFilenames));
    return imageFilenames;
}
int main(int argc, char **argv)
{

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(3), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(4), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(13.5), "Size of one square in mm")
        ("input_left", boost::program_options::value<std::string>(&inputDir_left)->default_value("stereo"), "Input directory containing chessboard images")
        ("input_right", boost::program_options::value<std::string>(&inputDir_right)->default_value("stereo"), "Input directory containing chessboard images")
        ("prefix,p", boost::program_options::value<std::string>(&prefix)->default_value("frame"), "Prefix of images")
        ("file-extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".jpg"), "File extension of images")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("kannala-brandt"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-name-left", boost::program_options::value<std::string>(&cameraName_left)->default_value("up"), "Name of left camera")
        ("camera-name-right", boost::program_options::value<std::string>(&cameraName_right)->default_value("down"), "Name of right camera")
        ("opencv", boost::program_options::bool_switch(&useOpenCV)->default_value(false), "Use OpenCV to detect corners")
        ("view-results", boost::program_options::bool_switch(&viewResults)->default_value(true), "View results")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(true), "Verbose output")
        ("rectification,r", boost::program_options::bool_switch(&rectification)->default_value(false), "rectification output")
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

    camodocal::StereoCalibration calibration(cameraName_left + "_camera_calib.yaml", cameraName_right + "_camera_calib.yaml", boardSize, squareSize);

    // look for images in input directory
    std::vector<std::string> imageFilenames_left = getImageFileNames(inputDir_left);
    std::vector<std::string> imageFilenames_right = getImageFileNames(inputDir_right);

    if (imageFilenames_left.empty() || imageFilenames_right.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # left images: " << imageFilenames_left.size() << std::endl;
        std::cerr << "# INFO: # right images: " << imageFilenames_right.size() << std::endl;
    }

    std::vector<bool> chessboardFound(imageFilenames_left.size(), false);

    for (size_t i = 0; i < imageFilenames_left.size(); ++i)
    {
        cv::Mat image_left = cv::imread(imageFilenames_left.at(i), -1).rowRange(0, 1024);
        cv::Mat image_right = cv::imread(imageFilenames_right.at(i), -1).rowRange(1024, 2048);

        camodocal::Chessboard chessboard_left(boardSize, image_left);
        chessboard_left.findCorners(useOpenCV);
        camodocal::Chessboard chessboard_right(boardSize, image_right);
        chessboard_right.findCorners(useOpenCV);

        if (chessboard_left.cornersFound() && chessboard_right.cornersFound())
        {
            if (verbose)
            {
                std::cerr << "# INFO: Detected chessboard in left image " << i << ", " << imageFilenames_left.at(i) << std::endl;
                std::cerr << "# INFO: Detected chessboard in right image " << i << ", " << imageFilenames_right.at(i) << std::endl;
            }

            calibration.addChessboardData(chessboard_left.getCorners(), chessboard_right.getCorners());

            cv::Mat sketch_left;
            chessboard_left.getSketch().copyTo(sketch_left);
            cv::imshow("left image", sketch_left);

            cv::Mat sketch_right;
            chessboard_right.getSketch().copyTo(sketch_right);
            cv::imshow("right image", sketch_right);
            cv::waitKey(50);
            chessboardFound.at(i) = true;
        }
        else if (verbose)
        {
            chessboardFound.at(i) = false;
            std::cerr << "# INFO: Did not detect chessboard in image " << i << std::endl;
        }
    }
    cv::destroyWindow("Image");

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    double startTime = camodocal::timeInSeconds();

    calibration.calibrate();

    if (verbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                  << " sec.\n";
    }

    //if (viewResults)
    //{
    //    std::vector<cv::Mat> cbImages;
    //    std::vector<std::string> cbImageFilenames;

    //    for (size_t i = 0; i < imageFilenames.size(); ++i)
    //    {
    //        if (!chessboardFound.at(i))
    //        {
    //            continue;
    //        }
    //        cbImages.push_back(cv::imread(imageFilenames.at(i), -1));
    //        cbImageFilenames.push_back(imageFilenames.at(i));

    //        // visualize observed and reprojected points
    //        calibration.drawResults(cbImages);
    //    }

    //    for (size_t i = 0; i < cbImages.size(); ++i)
    //    {
    //        cv::putText(cbImages.at(i), cbImageFilenames.at(i), cv::Point(10, 20),
    //                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
    //                    1, CV_AA);
    //        cv::imshow("Image", cbImages.at(i));
    //        cv::waitKey(0);
    //    }
    //}
    return 0;
}
