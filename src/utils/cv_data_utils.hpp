#ifndef CV_DATA_UTILS_HPP__
#define CV_DATA_UTILS_HPP__

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/ccalib.hpp>
#include <iostream>
#include <unordered_map>
#include <stdio.h>

template <class VV>
cv::Mat_<double> vectorVec3d_to_mat (VV m) {
    int n_rows = m.size(), n_cols = 3; //m[0].size();
    cv::Mat_<double> ret (n_rows, n_cols);
    for (int a=0; a<n_rows; ++a)
        for (int b=0; b<n_cols; ++b)
            ret(a,b) = m[a][b];
    return ret;
}

#endif
