#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include<Eigen/Dense>
#include <Eigen/Core>

// Not required for now

// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_binary_edge.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/core/robust_kernel_impl.h>

// #include "sophus/se3.hpp" 

#include <iostream>
#include <iomanip>
#include <ctype.h>
#include <algorithm> 
#include <iterator> 
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<double> getQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    Eigen::Quaterniond r(cos(M_PI/4), 0, 0, sin(M_PI/4));

    Eigen::Quaterniond new_q = r * q * r.inverse();

    std::vector<double> v(4);
    v[0] = new_q.x();
    v[1] = new_q.y();
    v[2] = new_q.z();
    v[3] = new_q.w();

    return v;
}
// void getQuaternion_rotxy(const cv::Mat &R, const Mat &T, std::ofstream &file, double timestep)
// {
//     Mat Transform_ = (Mat_<float>(4, 4) <<
//                 R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0, 0),
//                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1, 0),
//                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2, 0),
//                 0                 , 0                 , 0                  , 1);
    
//     Mat Correction = (Mat_<float>(4, 4) << 1,   0,           0,         0,
//                                            0,   cos(M_PI/2),  -sin(M_PI/2), 0,
//                                            0,   sin(M_PI/2),   cos(M_PI/2), 0,
//                                            0,   0,           0,         1);

//     Mat C_Transform = Transform_ * Correction;

//     Mat New_R = (Mat_<float>(3, 3) <<
//                 C_Transform.at<double>(0, 0), C_Transform.at<double>(0, 1), C_Transform.at<double>(0, 2),
//                 C_Transform.at<double>(1, 0), C_Transform.at<double>(1, 1), C_Transform.at<double>(1, 2),
//                 C_Transform.at<double>(2, 0), C_Transform.at<double>(2, 1), C_Transform.at<double>(2, 2));

    

//     Eigen::Matrix<double,3,3> eigMat = toMatrix3d(New_R) ;
//     Eigen::Quaterniond q(eigMat);

//     std::vector<double> v(4);
//     v[0] = q.x();
//     v[1] = q.y();
//     v[2] = q.z();
//     v[3] = q.w();

//     // file << setprecision(6) << timestep << setprecision(7) << " " << T.at<double>(0) << " " << -T.at<double>(2) << " " << T.at<double>(1)
//     //         << " " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << endl;
//     file << setprecision(6) << timestep << setprecision(7) << " " << C_Transform.at<double>(0, 3) << " " << C_Transform.at<double>(1, 3) << " " << C_Transform.at<double>(2, 3)
//             << " " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << endl;
// }

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int count = 0;
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVectorSem(vector<cv::Point2f> &v, vector<uchar> status)
{
    int count = 0;
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (!status[i])
            v[j++] = v[i];
    v.resize(j);
}

void morphOps(Mat &thresh){

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

// void filterSem(Mat &imgSem, vector<cv::Point2f> &points)
// {
//     Mat frame_HSV, frame_threshold;
    
//     vector<vector<cv::Point> > contours;
//     vector<cv::Vec4i> hierarchy;
//     vector<uchar> status(points.size());
    

//     cvtColor(imgSem, frame_HSV, COLOR_BGR2HSV);
//     inRange(frame_HSV, Scalar(1, 0, 0), Scalar(168, 255, 255), frame_threshold);

//     morphOps(frame_threshold);

//     findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

//     vector<vector<Point> > contours_poly( contours.size() );
//     vector<Rect> boundRect( contours.size() );

//     for( size_t i = 0; i < contours.size(); i++ )
//     {
//         approxPolyDP( contours[i], contours_poly[i], 3, true );
//         boundRect[i] = boundingRect( contours_poly[i]);
//     }

//     for(size_t i = 0; i < points.size(); i++)
//     {
//         for(size_t j = 1; j<= boundRect.size(); j++)
//         {            
//             if(boundRect[j].contains(points[i]))
//             {
//                 status[i] = 1;
//             }
//         }
//     }

//     reduceVectorSem(points, status);

// }


void filterSem(Mat &imgSem, vector<cv::Point2f> &points)
{
    vector<uchar> status(points.size());

    for( size_t i = 0; i < points.size(); i++ ) 
    {
        // keypoints1.push_back(cv::KeyPoint(points1[i], 1.f));
        cv::Vec3b color = imgSem.at<cv::Vec3b>(cv::Point(points[i].x,points[i].y));
        // myfile << points1[i] << "   color : " <<  (int)color[0] << " ," << (int)color[1] << " ," << (int)color[2] << " " << std::endl;
        bool cars = ((int)color[0] == 0 && (int)color[1] == 0 && (int)color[2] == 255);
        bool road = ((int)color[0] == 0 && (int)color[1] == 244 && (int)color[2] == 0);

        if(cars || road )
        {
            // myfile << points1[i] << "   color : " <<  (int)color[0] << " ," << (int)color[1] << " ," << (int)color[2] << " " << std::endl;
            // keypoints2.push_back(cv::KeyPoint(points1[i], 1.f));
            // points2.push_back(points1[i]);
            status[i] = 1;
        }
    }

    reduceVectorSem(points, status);
}

void featureTracking(Mat &img_1, Mat &img_2, vector<Point2f> &points1, vector<Point2f> &points2)
{
    vector<uchar> status;
    vector<float> err;					
    Size winSize=Size(21,21);																								
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    reduceVector(points1,status);
    reduceVector(points2,status);

    // int indexCorrection = 0;
    // for( int i=0; i<status.size(); i++)
    // {
    //     Point2f pt = points2.at(i- indexCorrection);
    //     if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))
    //     {
    //         if((pt.x<0)||(pt.y<0))
    //         {
    //             status.at(i) = 0;
    //         }
    //         points1.erase (points1.begin() + (i - indexCorrection));
    //         points2.erase (points2.begin() + (i - indexCorrection));
    //         indexCorrection++;
    //     }
    // }
}

void featureDetection(Mat &img_1, vector<Point2f> &points1)
{
    vector<KeyPoint> keypoints_1;
    // Ptr<FeatureDetector> detector = ORB::create();
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    // detector->detect(img_1,keypoints_1);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}

bool getFileContent(string fileName, vector<double> & vec)
{
    // Open the File
    ifstream in(fileName.c_str());
    // Check if object is valid
    if(!in)
    {
        cerr << "Cannot open the File : "<<fileName<<endl;
        return false;
    }
    string str;
    // Read the next line from File untill it reaches the end.
    while (getline(in, str))
    {
        // Line contains string of length > 0 then save it in vector
        if(str.size() > 0)
            vec.push_back(stod(str));
    }
    //Close The File
    in.close();
    return true;
}

void triangulation(const vector<Point2f> &prevPts, const vector<Point2f> &currPts, const Mat &currR, const Mat &currT,  vector<Point3d> &points) 
{

    Mat T1 = (Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);

    Mat T2 = (Mat_<float>(3, 4) <<
                currR.at<double>(0, 0), currR.at<double>(0, 1), currR.at<double>(0, 2), currT.at<double>(0, 0),
                currR.at<double>(1, 0), currR.at<double>(1, 1), currR.at<double>(1, 2), currT.at<double>(1, 0),
                currR.at<double>(2, 0), currR.at<double>(2, 1), currR.at<double>(2, 2), currT.at<double>(2, 0));
    
    Mat pts_4d;
    triangulatePoints(T1, T2, prevPts, currPts, pts_4d);

    for (int i = 0; i < pts_4d.cols; i++) 
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(2, 0);

        Point3d p( x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0) );
        points.push_back(p);
    }
}