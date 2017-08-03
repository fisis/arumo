/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <limits>
#include <unordered_map>

#include "utils/string_utils.hpp"

//socket libraries
/*#include<sys/types.h> 
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<netdb.h>
#define SERVERPORT "4950"   // the port users will be connecting to */

#define _max(x,y)  ( ((x)<(y)) ? (y) : (x) )

using namespace std;
using namespace cv;


namespace {
const char* about =
        "Calibration using a ArUco Planar Grid board\n"
        "  To capture a frame for calibration, press 'c',\n"
        "  If input comes from video, press any key for next frame\n"
        "  To finish capturing, press 'ESC' key and calibration starts.\n";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side lenght (in meters) }"
        "{s        |       | Separation between two consecutive markers in the grid (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{@outfile |<none> | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{waittime | 50   | Inter-frame wait time (in ms). }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       | false | Apply refind strategy }"
        "{zt       | false | Assume zero tangential distortion }"
        "{a        |       | Fix aspect ratio (fx/fy) to this value }"
        "{pc       | false | Fix the principal point at the center }"
        "{fmark    | 0.4   | Fraction of markers to be observed in order to capture a frame }"
        "{frdiff   | 200   | Acceptable average distance between markers between consecutive captured frames}"
        "{nfcycle  | 10    | Number of frames to capture before attempting a callibration }"
        "{nfcalib  | 30    | Max. number of frames to use during a calibration}"
        "{rethresh | 1.0   | Acceptable re-projection error threshold }";
}

/**
 */
static bool readDetectorParameters(string filename, aruco::DetectorParameters &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params.adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params.adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params.adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params.adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params.minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params.maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params.polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params.minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params.minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params.minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params.doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params.cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params.cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params.cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params.markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params.perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params.perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params.maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params.minOtsuStdDev;
    fs["errorCorrectionRate"] >> params.errorCorrectionRate;
    return true;
}


// ================================================================

/**
 */
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}



/**
 */

// =================================================

template <class T>
int find_in_vec (T val, vector<T> vec) {
    for (int a=0; a<vec.size(); ++a)
        if (vec[a] == val)  return (a);
    return (-1);
}

double marker_dist (std::vector<cv::Point2f> corner1, std::vector<cv::Point2f> corner2) {
    return (norm ( (corner1[0]+corner1[1]+corner1[2]+corner1[3]) - (corner2[0]+corner2[1]+corner2[2]+corner2[3]) ) );
}

// -------------------------------------------------

int nFramesPerCalibration, calibrationFlags = 0;
float aspectRatio = 1;
aruco::GridBoard board;
string outputFile;

// collected frames for calibration
vector< vector< vector< Point2f > > > allCorners;
vector< vector< int > > allIds;
Size imgSize;

// =================================================

bool attempt_calibration (double repErrorThresh=std::numeric_limits<double>::max()) {
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    
    Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // concatinate data for calibration
    for(unsigned int i = _max(0,((int)allCorners.size())-nFramesPerCalibration); i < allCorners.size(); ++i) {
        markerCounterPerFrame.push_back ((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }
    cout << "exiting loop" << endl;
    
    // calibrate camera
    repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                           markerCounterPerFrame, board, imgSize, cameraMatrix,
                                           distCoeffs, rvecs, tvecs, calibrationFlags);
    
    if (repError < repErrorThresh) {
        bool savedOk = saveCameraParams (outputFile, imgSize, aspectRatio, calibrationFlags, cameraMatrix, distCoeffs, repError);
        if (!savedOk) cout << "Failed to save file." <<endl;
        cout << "Reprojection error = " << repError << ". Camera params saved in " << outputFile << endl;
        return (savedOk);
    }
    else {
        cout << "repError = " << repError << ". Error threshold not met." << endl;
        return (false);
    }
}

 
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 6) {
        parser.printMessage();
        return 0;
    }
    
    int camId = parser.get<int>("ci");
    unordered_map<string,string> fname_replacements = { {"[ci]", to_string(camId)} };

    int markersX = parser.get<int>("w");
    int markersY = parser.get<int>("w");
    float markerLength = parser.get<float>("l");
    float markerSeparation = parser.get<float>("s");
    int dictionaryId = parser.get<int>("d");
    outputFile = multi_replace(parser.get<String>(0),fname_replacements);
    
    if(parser.has("a")) {
        calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = parser.get<float>("a");
    }
    if(parser.get<bool>("zt")) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    if(parser.get<bool>("pc")) calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

    aruco::DetectorParameters detectorParams;
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters (multi_replace(parser.get<string>("dp"),fname_replacements), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    bool refindStrategy = parser.get<bool>("rs");
    
    double frac_marker = parser.get<double>("fmark");
    int nFramesPerCycle = parser.get<int>("nfcycle");
    nFramesPerCalibration = parser.get<int>("nfcalib");
    double repErrorThresh = parser.get<double>("rethresh");
    double frameDiffThresh = parser.get<double>("frdiff");
    
    
    String video;

    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }
    

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = parser.get<int>("waittime");
    }

    aruco::Dictionary dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // create board object
    board = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners, ids);
        putText(imageCopy, "Show calibration board to camera.",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imshow("out", imageCopy); // SB
        char key = (char) waitKey (waitTime); // SB
        // if(key == 27) break; // SB
        
        if ( ids.size() > (int)(frac_marker*((double)markersX*markersY)) ) {  // SB
            
            // test if the frame is significantly different from last captured frame
            double avg_marker_dist = std::numeric_limits<double>::max();
            if (allIds.size() > 0) {
                int match_ct = 0;
                double total_dist = 0.0;
                for (int a=0; a<ids.size(); ++a) {
                    int lastpos = find_in_vec (ids[a], allIds.back()); // find the id in the last reading
                    if (lastpos >= 0) { // found the id in the last frame
                        total_dist += marker_dist (corners[a], allCorners.back()[lastpos]);
                        ++match_ct;
                    }
                }
                avg_marker_dist = total_dist/match_ct;
                if (match_ct!=0 && avg_marker_dist<frameDiffThresh) {
                    cout << "Frame is not sufficiently different from last: " << avg_marker_dist << endl;
                    continue;
                }
            }
            
            cout << "Frame captured" << endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            imgSize = image.size();
            
            if (allIds.size() % nFramesPerCycle == 0) {
                //break;
                cout << nFramesPerCycle << " new frames have been captured. Will now (re)attempt calibration." << endl;
                if (attempt_calibration (repErrorThresh)) { // successful
                    cout << "Successful!!" << endl;
                    return(0);
                }
                else
                    cout << "Continuing to gather frames." << endl;
            }
        }
        else
            cout << "Frame has insufficient markers: " << ids.size() << endl;
        
        
    }
    
    cout << "Video capture failure!!" << endl;
    return (1);
}
