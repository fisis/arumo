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
#include <iostream>
#include <unordered_map>
#include <cstdlib>
#include <deque>

#include "utils/string_utils.hpp"
#include "utils/cv_data_utils.hpp"

#define PI 3.141592653589793
#define TIME_STAMP_SEC (((double)getTickCount())/getTickFrequency())

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id(s) if input doesnt come from video (-v). Can be multiple comma-separated ids. }"
        "{c        |       | Camera intrinsic parameter file pattern: /path/to/file_with_[ci].yml. [ci] will be replaced by camera id. }"
        "{t        |       | Camera transformation parameter file pattern: /path/to/file_with_[ci].yml. [ci] will be replaced by camera id. }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File pattern of marker detector parameters: /path/to/file_with_[ci].yml. [ci] will be replaced by camera id. }"
        "{mposeage | 1.0   | Threshold on the age of a marker reading to consider it for computing average}"
        "{r        |       | show rejected candidates too }";
}

// ====================================================

vector<int> get_cam_ids (string camIdstring) {
    size_t len, nxtpos=0, lastpos=0;
    vector<int> ret;
    while (nxtpos!=string::npos) {
        nxtpos = camIdstring.find (',', lastpos);
        if (nxtpos==string::npos) len = camIdstring.length() - lastpos;
        else len = nxtpos - lastpos;
        ret.push_back ( atoi (camIdstring.substr (lastpos, len).c_str()));
        lastpos = nxtpos + 1;
    }
    return (ret);
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


class PoseReading {
public:
    Vec3d tvec, rvec;
    double timestamp;
    int camid;
    
    PoseReading (Vec3d tv, Vec3d rv, double t, int c): tvec(tv), rvec(rv), timestamp(t), camid(c) { }
};


enum VecType { COL_VEC, ROW_VEC };

Mat vec_to_Mat (vector<double> v, VecType vt=ROW_VEC) {
    Mat ret;
    if (vt == ROW_VEC) ret = Mat (1, v.size(), CV_64F);
    else if (vt == COL_VEC) ret = Mat (v.size(), 1, CV_64F);
    
    for (int i=0; i<v.size(); ++i)
        ret.at<double>(i) = v[i];
    return (ret);
}


/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }
    
    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");
    int waitTime;
    
    aruco::Dictionary dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    
    string camIdstring = parser.get<string>("ci");
    cout << "camIdstring: " << camIdstring << endl;
    vector<int> camIds = get_cam_ids (camIdstring);
    
    /*String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }*/
    
    // ==========================================================
    // Initiate for each cam:
    
    unordered_map<int,aruco::DetectorParameters> detectorParams;
    unordered_map<int,Mat> camMatrix, distCoeffs, transformationMatrix, transformationMatrix3x3;
    unordered_map<int,VideoCapture> inputVideo;
    
    for (auto it=camIds.begin(); it!=camIds.end(); ++it) {
        int camId = *it;
        unordered_map<string,string> fname_replacements = { {"[ci]", to_string(camId)} };
        
        detectorParams[camId] = aruco::DetectorParameters();
        if(parser.has("dp")) {
            bool readOk = readDetectorParameters (multi_replace(parser.get<string>("dp"),fname_replacements), detectorParams[camId]);
            if(!readOk) {
                cerr << "Invalid detector parameters file for camera " << camId << endl;
                return 0;
            }
        }
        detectorParams[camId].doCornerRefinement = true; // do corner refinement in markers
        
        // ----------------------------
        
        camMatrix[camId] = Mat(); distCoeffs[camId] = Mat();
        if(estimatePose) {
            bool readOk = readCameraParameters (multi_replace(parser.get<string>("c"),fname_replacements), camMatrix[camId], distCoeffs[camId]);
            if(!readOk) {
                cerr << "Invalid camera file for camera " << camId << endl;
                return 0;
            }
        }
        
        // ----------------------------
        
        FileStorage fs (multi_replace(parser.get<string>("t"),fname_replacements), FileStorage::READ);
        fs["transformationMatrix"] >> transformationMatrix[camId];
        transformationMatrix3x3[camId] = Mat(3,3,CV_64F);
        Mat hom_row = vec_to_Mat ({0.0,0.0,0.0,1.0}, ROW_VEC);
        transformationMatrix[camId].push_back (hom_row);
        transformationMatrix[camId] (Range(0,3), Range(0,3)) .copyTo (transformationMatrix3x3[camId]);
        
        cout << "transformationMatrix[camId]: " << transformationMatrix[camId] << endl;
        cout << "transformationMatrix3x3[camId]: " << transformationMatrix3x3[camId] << endl;
        
        /*int waitTime;
        if(!video.empty()) {
            inputVideo[camId].open(video);
            waitTime = 0;
        } else { */
            inputVideo[camId].open(camId);
            waitTime = 10;
        //}
    }
    

    

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    double totalTime = 0;
    int totalIterations = 0;
    bool stop=false;
    
    unordered_map <int, deque<PoseReading> >  marker_pose;
    int max_queue_size = 100;
    double max_pose_age = parser.get<double>("mposeage"); 
    
    while (!stop) {
        
        for (auto it=camIds.begin(); it!=camIds.end(); ++it) {
            int camId = *it;
            
            inputVideo[camId].grab();
            
            Mat image, imageCopy;
            inputVideo[camId].retrieve(image);

            double tick = (double)getTickCount();

            vector< int > ids;
            vector< vector< Point2f > > corners, rejected;
            vector< Vec3d > rvecs, tvecs;

            // detect markers and estimate pose
            aruco::detectMarkers (image, dictionary, corners, ids, detectorParams[camId], rejected);
            if(estimatePose && ids.size() > 0)
                aruco::estimatePoseSingleMarkers (corners, markerLength, camMatrix[camId], distCoeffs[camId], rvecs, tvecs);

            double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if(totalIterations % 30 == 0) {
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                     << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
            }

            // draw and aggregate results
            image.copyTo (imageCopy);
            if (ids.size() > 0) {
                aruco::drawDetectedMarkers (imageCopy, corners, ids);
                if(estimatePose) {
                    for(unsigned int i = 0; i < ids.size(); i++) {
                        // draw
                        aruco::drawAxis (imageCopy, camMatrix[camId], distCoeffs[camId], rvecs[i], tvecs[i], markerLength * 0.5f);
                        // aggregate
                        marker_pose[ids[i]].push_back ( PoseReading (tvecs[i], rvecs[i], TIME_STAMP_SEC, camId) );
                    }
                }
            }

            if(showRejected && rejected.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

            imshow (string("out")+to_string(camId), imageCopy);
            char key = (char)waitKey(waitTime);
            if(key == 27) stop=true;
        }
        
        
        for (auto it=marker_pose.begin(); it!=marker_pose.end(); ++it) {
            /*int& marker_id = it->first;
            deque< PoseReading >&  stamped_poses = it->second;*/
            vector<Vec3d> headvecs, tvecs;
            Mat_<double> marker_mean_headvec, marker_mean_tvec;
            Mat_<double> marker_headvec_cov(3,3), marker_tvec_cov(3,3);
            
            // pop old marker poses
            while (it->second.size() > max_queue_size)
                it->second.pop_front();
            
            // Compute average
            // ---------------
            double now = TIME_STAMP_SEC;
            int count = 0;
            for (auto it2=it->second.begin(); it2!=it->second.end(); ++it2) 
                if (now - it2->timestamp < max_pose_age) {
                    
                    // transform tvec
                    Mat hom_tvec = vec_to_Mat ({it2->tvec[0], it2->tvec[1], it2->tvec[2], 1.0}, COL_VEC); 
                    hom_tvec = transformationMatrix[it2->camid] * hom_tvec;
                    tvecs.push_back (Vec3d (hom_tvec.at<double>(0), hom_tvec.at<double>(1), hom_tvec.at<double>(2)));
                    
                    // transform rvec
                    Mat rvec = vec_to_Mat ({it2->rvec[0], it2->rvec[1], it2->rvec[2]}, COL_VEC);
                    Mat_<double> rmat(3,3);
                    Rodrigues (rvec, rmat); // rmat * camera_coord_vec = marker_ccord_vec
                                            // => ground_coord_vec = transformationMatrix3x3 * camera_coord_vec
                                            //                    = transformationMatrix3x3 * inv(rmat) * marker_ccord_vec
                    Mat headvec = transformationMatrix3x3[it2->camid] * rmat.inv() * vec_to_Mat ({1.0,0.0,0.0}, COL_VEC);
                    headvecs.push_back (Vec3d (headvec.at<double>(0), headvec.at<double>(1), headvec.at<double>(2)));
                    
                    ++count;
                }
            
            if (count > 0) {
                calcCovarMatrix (vectorVec3d_to_mat (headvecs), marker_headvec_cov, marker_mean_headvec, 
                                CV_COVAR_NORMAL|CV_COVAR_ROWS);
                calcCovarMatrix (vectorVec3d_to_mat (tvecs), marker_tvec_cov, marker_mean_tvec, 
                                CV_COVAR_NORMAL|CV_COVAR_ROWS);
                double heading_degrees = atan2(marker_mean_headvec.at<double>(1),marker_mean_headvec.at<double>(0)) * 180.0 / PI;
                
                // print
                cout << "Marker " << it->first << " in ground coordinates:\n\ttvec = " << marker_mean_tvec << "\n\theadvec = " << marker_mean_headvec << " (heading = " << heading_degrees << " degrees)" << endl;
            }
        }
        
    }

    return 0;
}
