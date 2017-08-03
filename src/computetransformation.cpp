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
#include <opencv2/aruco.hpp>
#include <opencv2/ccalib.hpp>
#include <iostream>
#include <unordered_map>
#include <stdio.h>

#include "utils/string_utils.hpp"
#include "utils/cv_data_utils.hpp"

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{@outfile |<none> | Output file with transformation parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{fframe   | 0.75  | The fraction of frames that should have the marker for it to be detected }"
        "{rmaxerr  | 1e-8  | Max. allowed determinant of covariance of rvecs of a marker }"
        "{tmaxerr  | 1e-15 | Max. allowed determinant of covariance of tvecs of a marker }"
        "{r        |       | show rejected candidates too }"
        "{grcoords | u     | Ground coordinate of markers in format 'id1:(x1,y1,z1);id2:(x2,y2,z2);...;[i|u]' (no space), where the last letter ('i' or 'u') indicates whether to (i)gnore other markers or ask for (u)ser input }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
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

unordered_map<int,Vec3d> get_ground_coords (String gCoordString, bool& ask_user_input) {
    unordered_map<int,Vec3d> ret;
    int id;
    double x, y, z;
    char ui;
    ask_user_input = true;
    size_t found, search_start=0;
    while (true) {
        found = gCoordString.find (';', search_start);
        if (found != String::npos) {
            sscanf (gCoordString.substr(search_start, found-search_start).c_str(), "%d:\(%lf,%lf,%lf\)", &id, &x, &y, &z);
            ret[id] = Vec3d(x,y,z);
        }
        else {
            sscanf (gCoordString.substr(search_start, found-search_start).c_str(), "%c", &ui);
            if (ui=='i') ask_user_input=false;
            break;
        }
        search_start = found + 1;
    }
    return (ret);
}

// ================================================================

/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }
    
    int camId = parser.get<int>("ci");
    unordered_map<string,string> fname_replacements = { {"[ci]", to_string(camId)} };

    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");
    double fracFrames = parser.get<double>("fframe");
    double rMaxErr = parser.get<double>("rmaxerr");
    double tMaxErr = parser.get<double>("tmaxerr");
    string outputFile = multi_replace(parser.get<String>(0),fname_replacements);
    String gCoordString = parser.get<String>("grcoords");
    
    //cout << "User-provided ground coordinates: " << gCoordString << endl;
    bool ask_user_input = true;
    unordered_map<int,Vec3d> user_ground_coords = get_ground_coords (gCoordString, ask_user_input);

    aruco::DetectorParameters detectorParams;
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters (multi_replace(parser.get<string>("dp"),fname_replacements), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams.doCornerRefinement = true; // do corner refinement in markers

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }
    

    aruco::Dictionary dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters (multi_replace(parser.get<string>("c"),fname_replacements), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    double totalTime = 0;
    int totalIterations = 0;
    unordered_map <int, vector<Vec3d> >  all_rvecs, all_tvecs;
    unordered_map <int,Vec3d>  ground_coords;
    unordered_map <int,int>  all_counts;
    
    Mat image, imageCopy;
    
    while (inputVideo.grab() && totalIterations<100) {
        inputVideo.retrieve(image);
        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if(estimatePose && ids.size()>0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        /*if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }*/
        
        // update history
        for(unsigned int i=0; i<ids.size(); i++) {
            all_rvecs[ids[i]].push_back (rvecs[i]);
            all_tvecs[ids[i]].push_back (tvecs[i]);
            if (all_counts.find(ids[i]) == all_counts.end()) all_counts[ids[i]] = 1;
            else ++all_counts[ids[i]];
        }
        
        // Compute and print up-to-date transformation data
        if (totalIterations % 10 == 0) {
            cout << "Markers detected: " << all_counts.size() << endl;
        }
        
        // ------------------------------------------
        // draw current markers
        image.copyTo (imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++) {
                    aruco::drawAxis (imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }
    
    // =============================================================
    // Camera transfomation computation
    vector<Vec3d>   marker_rvecs, marker_tvecs, marker_gcoords;
    
    cout << "Total markers detected: " << all_counts.size() << endl;
        for (auto it=all_counts.begin(); it!=all_counts.end(); ++it) {
            int marker_id = it->first;
            
            Mat_<double> marker_mean_rvec, marker_mean_tvec;
            Mat_<double> marker_rvec_cov(3,3), marker_tvec_cov(3,3);
            double rvec_cov_det, tvec_cov_det;
            
            
            calcCovarMatrix (vectorVec3d_to_mat (all_rvecs[marker_id]), marker_rvec_cov, marker_mean_rvec, 
                                CV_COVAR_NORMAL|CV_COVAR_ROWS);
            calcCovarMatrix (vectorVec3d_to_mat (all_tvecs[marker_id]), marker_tvec_cov, marker_mean_tvec, 
                                CV_COVAR_NORMAL|CV_COVAR_ROWS);
            rvec_cov_det = determinant (marker_rvec_cov);
            tvec_cov_det = determinant (marker_tvec_cov);
            
            image.copyTo (imageCopy);
            aruco::drawAxis (imageCopy, camMatrix, distCoeffs, marker_mean_rvec, marker_mean_tvec, markerLength * 0.5f);
            imshow("out", imageCopy);
            char key = (char)waitKey(waitTime);
            
            cout << "\nMarker " << marker_id << " (" << all_counts[marker_id] << " detections)." << endl;
            cout << "rvec mean: " << marker_mean_rvec << ". rvec error: " << rvec_cov_det << endl;
            cout << "tvec mean: " << marker_mean_tvec << ". tvec error: " << tvec_cov_det << endl;
            
            if (all_counts[marker_id] < totalIterations*fracFrames)
                cout << "Not enough detections." << endl;
            else if (rvec_cov_det > rMaxErr)
                cout << "rvec error above threshold." << endl;
            else if (tvec_cov_det > tMaxErr)
                cout << "tvec error above threshold." << endl;
            else {
                if (user_ground_coords.find (marker_id) != user_ground_coords.end()) {
                    cout << "Using ground coordinates " << user_ground_coords[marker_id] << endl;
                    // cout << ">>> Hit return key to continue."; cin.ignore();
                    ground_coords[marker_id] = user_ground_coords[marker_id];
                }
                else if (ask_user_input) {
                    ground_coords[marker_id] = Vec3d(0.0,0.0,0.0);
                    cout << ">>> Enter ground coordinates of this marker [comma separated numbers: x, y, z ]: ";
                    scanf ("%f, %f, %f", 
                            &(ground_coords[marker_id][0]), &(ground_coords[marker_id][1]), &(ground_coords[marker_id][2]) );
                }
                else {
                    cout << "Coordinate not provided. skipping." << endl;
                    continue;
                }
                marker_rvecs.push_back (Vec3d (marker_mean_rvec));
                marker_tvecs.push_back (Vec3d (marker_mean_tvec));
                marker_gcoords.push_back (ground_coords[marker_id]);
            }
                
        }
        
    // computation
    if (marker_tvecs.size() < 4) {
        cout << "Requires at least 4 points to compute transformation. " << marker_tvecs.size() << " provided." << endl;
        return(1);
    }
    
    Mat_<double> trans(3,4);
    std::vector<uchar> inliers;
    estimateAffine3D (vectorVec3d_to_mat(marker_tvecs), vectorVec3d_to_mat(marker_gcoords), trans, inliers, 3, 0.99);
    
    cout << "\nmarker_tvecs = \n" << vectorVec3d_to_mat(marker_tvecs) << endl;
    cout << "marker_gcoords = \n" << vectorVec3d_to_mat(marker_gcoords) << endl;
    cout << "Computed transfmation matrix = \n" << trans << endl;
    
    FileStorage fs (outputFile, FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "transformationMatrix" << trans;
        cout  << "Transformation matrix saved in " << outputFile << endl;
        fs.release();
    } else
        cout << "Failed to save file." <<endl;

    return 0;
}

