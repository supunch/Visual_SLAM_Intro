#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <sstream>
#include <fstream>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Mat img, img_sem;
bool mousedown;
vector<vector<Point> > contours;
vector<Point> pts;
RNG rng(12345);

void morphOps(Mat &thresh){

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);

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

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int count = 0;
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (!status[i])
            v[j++] = v[i];
    v.resize(j);
}


void filterSem(Mat &imgSem, vector<cv::Point2f> &points)
{
    Mat frame_HSV, frame_threshold;
    
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    vector<uchar> status(points.size());
    

    cvtColor(imgSem, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(1, 0, 0), Scalar(168, 255, 255), frame_threshold);
    imshow("hsv", frame_HSV);

    morphOps(frame_threshold);

    findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = boundingRect( contours_poly[i]);
    }

    for(size_t i = 0; i < points.size(); i++)
    {
        for(size_t j = 1; j<= boundRect.size(); j++)
        {            
            if(boundRect[j].contains(points[i]))
            {
                status[i] = 1;
            }
        }
    }

    reduceVectorSem(points, status);

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





int main( int argc, const char** argv )
{
    // src = imread(argv[1]);
    // if(src.empty())
    // {
    //     return -1;
    // }
    // char filename1[200];
    // sprintf(filename1, "/home/vagrant/shared/Kitti/00/image_1/%06d.png", 0);

    // namedWindow(window_detection_name);
    // Trackbars to set thresholds for HSV values
    // createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    // createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    // createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    // createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    // createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    // createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    Mat frame, frame_HSV, frame_threshold;

    img = imread("/home/vagrant/shared/scania_images_undistorted/frame000085.png");
    img_sem = imread("/home/vagrant/shared/scania_sem_images/frame000085.png");

    frame = img.clone();

    cvtColor(img,img, COLOR_BGR2GRAY);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(img, img);

    // cvtColor(img_sem, frame_HSV, COLOR_BGR2HSV);
    // inRange(frame_HSV, Scalar(1, 0, 0), Scalar(168, 255, 255), frame_threshold);

    // morphOps(frame_threshold);


    // vector<vector<Point> > contours;
    // vector<Vec4i> hierarchy;
    // findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );


    // // Mat drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
    // vector<vector<Point> > contours_poly( contours.size() );
    // vector<Rect> boundRect( contours.size() );
    // vector<Point2f> centers( contours.size() );
    // vector<float> radius( contours.size() );
    // for( size_t i = 0; i < contours.size(); i++ )
    // {
    //     approxPolyDP( contours[i], contours_poly[i], 3, true );
    //     boundRect[i] = boundingRect( contours_poly[i] );
    //     // minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    // }

    // for( size_t i = 0; i< contours.size(); i++ )
    // {
    //     Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
    //     drawContours( frame, contours_poly, (int)i, color );
    //     rectangle( frame, boundRect[i].tl(), boundRect[i].br(), color, 2 );
    //     // circle( frame, centers[i], (int)radius[i], color, 2 );
    // }

    // // cv::bitwise_and(img,frame_threshold,frame);


    vector<cv::Point2f> points1, points2;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    ofstream myfile;
    myfile.open ("colors.txt");

    featureDetection(img, points1);
    
    for( size_t i = 0; i < points1.size(); i++ ) 
    {
        keypoints1.push_back(cv::KeyPoint(points1[i], 1.f));
    }

    for( size_t i = 0; i < points1.size(); i++ ) 
    {
        // keypoints1.push_back(cv::KeyPoint(points1[i], 1.f));
        cv::Vec3b color = img_sem.at<cv::Vec3b>(cv::Point(points1[i].x,points1[i].y));
        // myfile << points1[i] << "   color : " <<  (int)color[0] << " ," << (int)color[1] << " ," << (int)color[2] << " " << std::endl;
        bool cars = ((int)color[0] == 0 && (int)color[1] == 0 && (int)color[2] == 255);
        bool road = ((int)color[0] == 0 && (int)color[1] == 244 && (int)color[2] == 0);

        if(!cars && !road)
        {
            myfile << points1[i] << "   color : " <<  (int)color[0] << " ," << (int)color[1] << " ," << (int)color[2] << " " << std::endl;
            // keypoints2.push_back(cv::KeyPoint(points1[i], 1.f));
            points2.push_back(points1[i]);

        }

    }


    
    // filterSem(img_sem, points1);

    // for( size_t i = 0; i < points1.size(); i++ ) 
    // {
    //     keypoints2.push_back(cv::KeyPoint(points1[i], 1.f));
    // }

    
    // //ORB 
    // //−− initialization
    // // std::vector<KeyPoint> keypoints_1, keypoints_2;
    // // Mat descriptors_1, descriptors_2;
    // // Ptr<FeatureDetector> detector = ORB::create();
    // // Ptr<DescriptorExtractor> descriptor = ORB::create();
    // // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // //−− detect Oriented FAST
    // //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // // detector->detect(img,keypoints_1);
    // // cout<< points1.size() << endl;

    // // for(size_t i = 0; i < points1.size(); i++)
    // // {   
    // //     // if (points1[i].y < 1024 && points1[i].x< 640)
    // //     // {
            
    // //     // }
    // //     myfile << frame_threshold.at<cv::Vec3b>(points1[i].pt) << points1[i].pt << endl;
    // //     cv::Vec3b color = frame_threshold.at<cv::Vec3b>(points1[i].pt);

    // //     if(color.val[0] == 0 && color.val[1] == 0 && color.val[2] == 0 )
    // //     {
    // //         
    // //         // cout <<  points1[i].pt << endl;
    // //     }
    // //     // cout <<  cv::Point(points1[i].y, points1[i].x) << endl;
    // //     //frame_threshold[points1[i].x,points1[i].y,0];

   
    
    // cout<< points1.size() << endl;

    // vector<uchar> status(points1.size());
    // for(size_t i = 0; i < points1.size(); i++)
    // {
    //     for(size_t j = 1; j<= boundRect.size(); j++)
    //     {            
    //         if(boundRect[j].contains(points1[i]))
    //         {
    //             // points2.push_back(points1[i]);
    //             // break;
    //             // points1.erase(points1.begin() + i);
    //             status[i] = 1;
                
    //         }
    //     }
    // }
    // cout<< status.size() << endl;

    // reduceVector(points1,status);
    
    // cout<< points1.size() << endl;


    
    // for(size_t i = 0; i < points1.size(); i++)
    // {

    //     // for(size_t j = 1; j<= boundRect.size(); j++)
    //     // {
                        
    //     //     if(boundRect[j].contains(points1[i]))
    //     //     {
    //     //         // points2.push_back(points1[i]);
    //     //         // // break;
    //     //        points1.erase(points1.begin() + i);
    //     //         // points1.s
    //     //     }
            
    //     // }
    // }
    // // cout<< points2.size() << endl;
    
    // // Mat::zeros(Size(image.cols,image.rows),CV_8UC1);
    

    // // for(size_t i = 0; i < frame_threshold.rows ; i++)
    // // {
    // //     for(size_t j = 0; j < frame_threshold.cols ; j++)
    // //     {
    // //         cv::Vec3b color = frame_threshold.at<cv::Vec3b>(cv::Point(i, j));
    // //         if(color.val[0] == 0 && color.val[1] == 0 && color.val[2] == 0 )
    // //         {

    // //             // points2.push_back(points1[i]);
    // //             cv::circle(img, cv::Point(i, j), 2.0, cv::Scalar(0, 0, 255), 2, cv::LINE_8, 0);
    // //             cout <<  cv::Point(i, j) << endl;
    // //         }

    // //     }
    // // }
    


    
    // for( size_t i = 0; i < points1.size(); i++ ) 
    // {
    //     keypoints1.push_back(cv::KeyPoint(points1[i], 1.f));
    // }

    for( size_t i = 0; i < points2.size(); i++ ) 
    {
        keypoints2.push_back(cv::KeyPoint(points2[i], 1.f));
    }
    // Mat th1 = frame_threshold.clone();
    // Mat th2 = frame_threshold.clone();

    drawKeypoints(img, keypoints1, img, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    drawKeypoints(frame, keypoints2, frame, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    drawKeypoints(img_sem, keypoints1, img_sem, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // // drawKeypoints(frame_threshold, difference, frame_threshold, Scalar::all(-1), DrawMatchesFlags::DEFAULT);


    // // // Show the frames
    imshow("Unmasked", img);
    imshow("Unmasked seg", frame);

    imshow(" Mask", img_sem);

    // imshow(" bitwise", frame_threshold);

    // imshow(" bitwise", frame);


    // 
    // while (true) 
    // {
        

    //     char key = (char) waitKey(30);
    //     if (key == 'q' || key == 27)
    //     {
    //         break;
    //     }
        
    // }

    // namedWindow("Create Mask", WINDOW_AUTOSIZE);
    // // cloneimg = src.clone();
    // // setMouseCallback( "Create Mask", onMouse, &cloneimg );
    // imshow( "Create Mask", img_sem );

    waitKey(0);
    return 0;
}
