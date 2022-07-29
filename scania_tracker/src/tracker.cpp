#include "tracker.h"

#define MAX_FRAME 3200
#define MIN_NUM_FEAT 2000

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    Mat img_1, img_2, img_1_c, img_2_c, img_1_sem, img_2_sem, img_1_c_d, img_2_c_d;
    Mat R_f, t_f;

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    //  Get timesteps from KITTI to save trajectory in TUM format
    string time_file = "/home/vagrant/shared/Scania_GT/scania_timesteps.txt";
    vector<double> timesteps;
    getFileContent(time_file,timesteps);
    
    ofstream myfile;
    ofstream debugfile;
    myfile.open ("/home/vagrant/comp/traj_rl.txt");
    debugfile.open("debugfile.txt");

    myfile << setprecision(6) << timesteps[0] << setprecision(7) << " "<< 0 << " " << 0 << " " << 0
            << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;

    double scale = 1.00;
    double prevScale = 1.00;
    int count = 0;
    char filename1[200];
    char filename2[200];
    char filename1_sem[200];
    char filename2_sem[200];
    // sprintf(filename1, "/home/vagrant/shared/scania_raw_undistorted/%06d.png", 1);
    // sprintf(filename2, "/home/vagrant/shared/scania_raw_undistorted/%06d.png", 2);
    sprintf(filename1, "/home/vagrant/shared/scania_raw_undistorted_2/%06d.png", 85);
    sprintf(filename2, "/home/vagrant/shared/scania_raw_undistorted_2/%06d.png", 86);

    // sprintf(filename1_sem, "/home/vagrant/shared/scania_sem_images/frame%06d.png", 100);
    // sprintf(filename2_sem, "/home/vagrant/shared/scania_sem_images/frame%06d.png", 101);

    char text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;  
    cv::Point textOrg(10, 50);

    img_1_c = imread(filename1);
    img_2_c = imread(filename2);

    img_1_sem = imread(filename1_sem);
    img_2_sem = imread(filename2_sem);

    double fx = 445.84586038;
    double fy = 446.87871792;
    double cx = 520.48531042;
    double cy = 356.04878655;

    double k1 = -0.055161;
    double k2 =  0.028367;
    double k3 = -0.047311;
    double k4 =  0.026752;
    
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortionMatrix = cv::Mat::zeros(1, 4, CV_64F);
    cv::Size imageSize = cv::Size(1024,640);
    cv::Mat rectificationMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat newCameraMatrix  = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat map1;
    cv::Mat map2;

    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;

    distortionMatrix.at<double>(0, 0) = k1;
    distortionMatrix.at<double>(0, 1) = k2;
    distortionMatrix.at<double>(0, 2) = k3;
    distortionMatrix.at<double>(0, 3) = k4;

    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distortionMatrix, imageSize, rectificationMatrix, newCameraMatrix, 0, imageSize, 0);
    // cv::fisheye::initUndistortRectifyMap(cameraMatrix, distortionMatrix, rectificationMatrix, newCameraMatrix, imageSize, CV_16SC2 , map1, map2);

    // std::cout << distortionMatrix << '\n' << std::endl;
    // std::cout << newCameraMatrix << '\n' << std::endl;
    // std::cout << cameraMatrix << '\n' << std::endl;

    // cv::remap(img_1_c_d, img_1_c, map1, map2, 1, CV_HAL_BORDER_CONSTANT, 0);
    // cv::remap(img_2_c_d, img_2_c, map1, map2, 1, CV_HAL_BORDER_CONSTANT, 0);

    cvtColor(img_1_c,img_1, COLOR_BGR2GRAY);
    cvtColor(img_2_c,img_2, COLOR_BGR2GRAY);

    clahe->apply(img_1, img_1);
    clahe->apply(img_2, img_2);

    // feature tracking
    vector<Point2f> points1, points2; 

    featureDetection(img_1, points1);
    // filterSem(img_1_sem, points1);

    featureTracking(img_1,img_2,points1,points2);
    // filterSem(img_2_sem, points2);

    std::cout << points1.size() << '\n' << std::endl;
    std::cout << points2.size() << '\n' << std::endl;
    
    Mat E, currR, currT, mask;
    E = findEssentialMat(points2, points1, cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1,cameraMatrix, currR, currT, mask);

    vector<Point3d> currPoints3d;
    Mat prevR = Mat::eye(3, 3, CV_64F);
    Mat prevT = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    
    triangulation(points1, points2, currR, currT, currPoints3d);

    Mat prevImage = img_2;
    Mat prevImage_sem = img_2_sem;
    Mat currImage_c, currImage, currImage_d;
    vector<Point2f> currFeatures;
    vector<Point2f> prevFeatures = points2;
    vector<Point3d> prevpoints3d = currPoints3d;
    prevR = currR;
    prevT = currT;

    
    char filename[100];
    char filename_sem[100];

    R_f = currR.clone();
    t_f = currT.clone();

    vector<double> Quat(4);
    Quat = getQuaternion(R_f);
    myfile << setprecision(6) << timesteps[1] << setprecision(7) << " " << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2)
            << " " << Quat[0] << " " << Quat[1] << " " << Quat[2] << " " << Quat[3] << endl;

    namedWindow( "Road facing camera", WINDOW_AUTOSIZE );
    namedWindow( "Trajectory", WINDOW_AUTOSIZE );

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    for(int numFrame=87; numFrame < MAX_FRAME; numFrame++)	
    {
        cout << " loopin " << numFrame << endl;
        // sprintf(filename, "/home/vagrant/shared/scania_raw_undistorted/%06d.png", numFrame);
        sprintf(filename, "/home/vagrant/shared/scania_raw_undistorted_2/%06d.png", numFrame);
        // sprintf(filename_sem, "/home/vagrant/shared/scania_sem_images/frame%06d.png", numFrame);


        currImage_c = imread(filename);
        // cv::remap(currImage_d, currImage_c, map1, map2, 1, CV_HAL_BORDER_CONSTANT, 0);
        // Mat currImage_sem = imread(filename_sem);


        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
        clahe->apply(currImage, currImage);

        if (prevFeatures.size() < MIN_NUM_FEAT)
        {
            featureDetection(prevImage, prevFeatures);
            // filterSem(prevImage_sem, prevFeatures); 
            featureTracking(prevImage,currImage,prevFeatures,currFeatures);
            // filterSem(currImage_sem, currFeatures); 
        }
        else
        {
            featureTracking(prevImage, currImage, prevFeatures, currFeatures);
            // cout << " curr points" << " " << currFeatures.size() << endl;
            // filterSem(currImage_sem, currFeatures);
            // cout << " curr points" << " " << currFeatures.size() << endl;
        }
        debugfile << "debug angles" << std::endl;

        cv::Size size(prevImage.cols + currImage.cols, MAX(prevImage.rows, currImage.rows));
        cv::Mat outImg = cv::Mat::zeros(size, CV_MAKETYPE(prevImage.depth(), prevImage.channels()));
        cv::hconcat(prevImage, currImage, outImg);

        std::vector<uchar> angle_status(currFeatures.size(), 1);
        size_t N = MIN(currFeatures.size(), prevFeatures.size());
        for (size_t i = 0; i < N; i++)
        {
            cv::Point2f pt1 = prevFeatures[i];
            cv::Point2f pt2 = cv::Point2f(std::min(currFeatures[i].x + prevImage.cols, float(outImg.cols - 1)), currFeatures[i].y);
            double angle = std::atan2(pt2.y - pt1.y,pt2.x - pt1.x);
            debugfile << "angle : "<< N << " : " << angle << std::endl;
            if(abs(angle) > 0.01)
            {
                angle_status[i] = 0;
            }
            // cv::line(outImg, pt1, pt2, cv::Scalar::all(255), 1, cv::LINE_AA);
        }
        
        reduceVector(prevFeatures, angle_status);
        reduceVector(currFeatures, angle_status);
        
        E = findEssentialMat(currFeatures, prevFeatures, cameraMatrix, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, cameraMatrix, currR, currT, mask);

        vector<Point3d> currPoints3d;
        triangulation(prevFeatures, currFeatures, currR, currT, currPoints3d);


        scale = 0;
        count = 0;
        
        for (size_t j=0; j < 1000-1; j++) 
        {
            for (size_t k=j+1; k< 1000; k++) 
            {
                double s = norm(prevpoints3d[j] - prevpoints3d[k]) / norm(currPoints3d[j] - currPoints3d[k]);
                if(0<s<10)
                {
                    scale += s;
                    count++;
                }
                
            }
        }

        assert(count > 0);
        scale /= count;
        cout << scale << endl;

        assert(0<scale<3);

        if(!isnan(scale))
        {
            if( (0<scale) && (scale<3))
            {
                t_f = t_f + (1/scale)*(R_f*currT);
                R_f = currR*R_f;

            }
            else
            {
                t_f = t_f + 1*(R_f*currT);
                R_f = currR*R_f;
            }
        }

        N = MIN(currFeatures.size(), currFeatures.size());
        for (size_t i = 0; i < N; i++)
        {
            cv::Point2f pt1 = prevFeatures[i];
            cv::Point2f pt2 = cv::Point2f(std::min(currFeatures[i].x + prevImage.cols, float(outImg.cols - 1)), currFeatures[i].y);
            cv::line(outImg, pt1, pt2, cv::Scalar::all(255), 1, cv::LINE_AA);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;
        prevpoints3d = currPoints3d;
        prevR = currR;
        prevT = currT;
        prevScale = scale;

        vector<double> Quat(4);
        Quat = getQuaternion(R_f);
        myfile << setprecision(6) << timesteps[numFrame] << setprecision(7) << " " << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2)
            << " " << Quat[0] << " " << Quat[1] << " " << Quat[2] << " " << Quat[3] << endl;

        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(1)) + 100;
        circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

        rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), -1 );
        // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        // putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

        // std::vector<cv::KeyPoint> keypoints;
        // for( size_t i = 0; i < currFeatures.size(); i++ ) 
        // {
        //     keypoints.push_back(cv::KeyPoint(currFeatures[i], 1.f));
        // }

        // drawKeypoints(currImage, keypoints, currImage, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

        imshow( "Road facing camera", outImg );
        imshow( "Trajectory", traj );

        waitKey(10);


    }
    waitKey(10);
    return 0;
}