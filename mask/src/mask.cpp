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

// void onMouse( int event, int x, int y, int flags, void* userdata )
// {
//     Mat img = *((Mat *)userdata);

//     if( event == EVENT_LBUTTONDOWN )
//     {
//         mousedown = true;
//         contours.clear();
//         pts.clear();
//     }

//     if( event == EVENT_LBUTTONUP )
//     {
//         mousedown = false;
//         if(pts.size() > 2 )
//         {
//             Mat mask(img.size(),CV_8UC1);
//             mask = 0;
//             contours.push_back(pts);
//             drawContours(mask,contours,0,Scalar(255),-1);
//             Mat masked(img.size(),CV_8UC3,Scalar(255,255,255));
//             src.copyTo(masked,mask);
//             src.copyTo(cloneimg);
//             imshow( "masked", masked );
//         }
//     }

//     if(mousedown)
//     {
//         if(pts.size() > 2 )
//             line(img,Point(x,y),pts[pts.size()-1],Scalar(0,255,0));

//         pts.push_back(Point(x,y));

//         imshow( "Create Mask", img );
//     }
// }

// const int max_value_H = 360/2;
// const int max_value = 255;
// const String window_capture_name = "Video Capture";
// const String window_detection_name = "Object Detection";
// int low_H = 0, low_S = 0, low_V = 0;
// int high_H = max_value_H, high_S = max_value, high_V = max_value;

// static void on_low_H_thresh_trackbar(int, void *)
// {
//     low_H = min(high_H-1, low_H);
//     setTrackbarPos("Low H", window_detection_name, low_H);
// }
// static void on_high_H_thresh_trackbar(int, void *)
// {
//     high_H = max(high_H, low_H+1);
//     setTrackbarPos("High H", window_detection_name, high_H);
// }
// static void on_low_S_thresh_trackbar(int, void *)
// {
//     low_S = min(high_S-1, low_S);
//     setTrackbarPos("Low S", window_detection_name, low_S);
// }
// static void on_high_S_thresh_trackbar(int, void *)
// {
//     high_S = max(high_S, low_S+1);
//     setTrackbarPos("High S", window_detection_name, high_S);
// }
// static void on_low_V_thresh_trackbar(int, void *)
// {
//     low_V = min(high_V-1, low_V);
//     setTrackbarPos("Low V", window_detection_name, low_V);
// }
// static void on_high_V_thresh_trackbar(int, void *)
// {
//     high_V = max(high_V, low_V+1);
//     setTrackbarPos("High V", window_detection_name, high_V);
// }

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

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
    // int erosion_size = 3;
    // int dilation_size = 1;


	// Mat erodeElement = getStructuringElement(MORPH_RECT,Size(2*erosion_size + 1, 2*erosion_size+1), Point( erosion_size, erosion_size ));
    // Mat erodeElement = getStructuringElement(MORPH_RECT,Size(1, 1));
	//dilate with larger element so make sure object is nicely visible
	// Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(2*dilation_size + 1, 2*dilation_size+1), Point( dilation_size, dilation_size ));
    // Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

	// erode(thresh, thresh, erodeElement);
	// erode(thresh, thresh, dilateElement);

	// dilate(thresh, thresh, dilateElement);
	// dilate(thresh, thresh, dilateElement);
    // dilate(thresh, thresh, dilateElement);
    // dilate(thresh, thresh, dilateElement);
    // dilate(thresh, thresh, dilateElement);

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);

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

    img = imread("/home/vagrant/shared/scania_images_undistorted/frame000704.png");
    img_sem = imread("/home/vagrant/shared/scania_sem_images/frame000704.png");

    frame = img.clone();

    cvtColor(img,img, COLOR_BGR2GRAY);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(img, img);

    cvtColor(img_sem, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(1, 0, 0), Scalar(168, 255, 255), frame_threshold);

    morphOps(frame_threshold);


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );


    // Mat drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f> centers( contours.size() );
    vector<float> radius( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = boundingRect( contours_poly[i] );
        // minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }

    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( frame, contours_poly, (int)i, color );
        rectangle( frame, boundRect[i].tl(), boundRect[i].br(), color, 2 );
        // circle( frame, centers[i], (int)radius[i], color, 2 );
    }

    // cv::bitwise_and(img,frame_threshold,frame);


    vector<cv::Point2f> points1, points2; 

    featureDetection(img, points1);
    // featureDetection(frame, points2);
    

    ofstream myfile;
    myfile.open ("colors.txt");
    //ORB 
    //−− initialization
    // std::vector<KeyPoint> keypoints_1, keypoints_2;
    // Mat descriptors_1, descriptors_2;
    // Ptr<FeatureDetector> detector = ORB::create();
    // Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    //−− detect Oriented FAST
    //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // detector->detect(img,keypoints_1);
    // cout<< points1.size() << endl;

    // for(size_t i = 0; i < points1.size(); i++)
    // {   
    //     // if (points1[i].y < 1024 && points1[i].x< 640)
    //     // {
            
    //     // }
    //     myfile << frame_threshold.at<cv::Vec3b>(points1[i].pt) << points1[i].pt << endl;
    //     cv::Vec3b color = frame_threshold.at<cv::Vec3b>(points1[i].pt);

    //     if(color.val[0] == 0 && color.val[1] == 0 && color.val[2] == 0 )
    //     {
    //         
    //         // cout <<  points1[i].pt << endl;
    //     }
    //     // cout <<  cv::Point(points1[i].y, points1[i].x) << endl;
    //     //frame_threshold[points1[i].x,points1[i].y,0];

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    for( size_t i = 0; i < points1.size(); i++ ) 
    {
        keypoints1.push_back(cv::KeyPoint(points1[i], 1.f));
    }
    
    cout<< points1.size() << endl;

    vector<uchar> status(points1.size());
    for(size_t i = 0; i < points1.size(); i++)
    {
        for(size_t j = 1; j<= boundRect.size(); j++)
        {            
            if(boundRect[j].contains(points1[i]))
            {
                // points2.push_back(points1[i]);
                // break;
                // points1.erase(points1.begin() + i);
                status[i] = 1;
                
            }
        }
    }
    cout<< status.size() << endl;

    reduceVector(points1,status);
    
    cout<< points1.size() << endl;


    
    for(size_t i = 0; i < points1.size(); i++)
    {

        // for(size_t j = 1; j<= boundRect.size(); j++)
        // {
                        
        //     if(boundRect[j].contains(points1[i]))
        //     {
        //         // points2.push_back(points1[i]);
        //         // // break;
        //        points1.erase(points1.begin() + i);
        //         // points1.s
        //     }
            
        // }
    }
    // cout<< points2.size() << endl;
    
    // Mat::zeros(Size(image.cols,image.rows),CV_8UC1);
    

    // for(size_t i = 0; i < frame_threshold.rows ; i++)
    // {
    //     for(size_t j = 0; j < frame_threshold.cols ; j++)
    //     {
    //         cv::Vec3b color = frame_threshold.at<cv::Vec3b>(cv::Point(i, j));
    //         if(color.val[0] == 0 && color.val[1] == 0 && color.val[2] == 0 )
    //         {

    //             // points2.push_back(points1[i]);
    //             cv::circle(img, cv::Point(i, j), 2.0, cv::Scalar(0, 0, 255), 2, cv::LINE_8, 0);
    //             cout <<  cv::Point(i, j) << endl;
    //         }

    //     }
    // }
    


    
    for( size_t i = 0; i < points1.size(); i++ ) 
    {
        keypoints2.push_back(cv::KeyPoint(points1[i], 1.f));
    }

    // for( size_t i = 0; i < points2.size(); i++ ) 
    // {
    //     keypoints2.push_back(cv::KeyPoint(points2[i], 1.f));
    // }
    Mat th1 = frame_threshold.clone();
    Mat th2 = frame_threshold.clone();

    drawKeypoints(th1, keypoints1, th1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    drawKeypoints(th2, keypoints2, th2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // drawKeypoints(frame_threshold, difference, frame_threshold, Scalar::all(-1), DrawMatchesFlags::DEFAULT);


    // // Show the frames
    imshow("Unmasked", th1);
    imshow(" Mask", th2);

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
