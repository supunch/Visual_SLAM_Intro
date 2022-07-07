#include "tracker.h"

#define MAX_FRAME 3200
#define MIN_NUM_FEAT 2000

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    Mat img_1, img_2, img_1_c, img_2_c, img_1_sem, img_2_sem;
    Mat R_f, t_f;

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    //  Get timesteps from KITTI to save trajectory in TUM format
    string time_file = "/home/vagrant/shared/Scania_GT/scania_timesteps.txt";
    vector<double> timesteps;
    getFileContent(time_file,timesteps);
    
    ofstream myfile;
    myfile.open ("traj.txt");

    ofstream debugfile;
    debugfile.open ("scale.txt");

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
    sprintf(filename1, "/home/vagrant/shared/scania_images_undistorted/frame%06d.png", 100);
    sprintf(filename2, "/home/vagrant/shared/scania_images_undistorted/frame%06d.png", 101);

    sprintf(filename1_sem, "/home/vagrant/shared/scania_sem_images/frame%06d.png", 100);
    sprintf(filename2_sem, "/home/vagrant/shared/scania_sem_images/frame%06d.png", 101);

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
    // double fx = 981.376478;
    // double fy = 983.649959;
    // double cx = 966.939307;
    // double cy = 651.942589;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;

    // cv::Mat D = (cv::Mat_<float>(4,1) << -0.055161, 0.028367, -0.047311, 0.026752);
    // cv::Mat R = cv::Mat::eye(3,3,CV_32F);

    
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
    
    Mat E, currR, currT, mask;
    E = findEssentialMat(points2, points1, cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1,cameraMatrix, currR, currT, mask);

    cout << " curr R " << currR << endl;
    cout << " curr t " << currT << endl;

    vector<Point3d> currPoints3d;
    Mat prevR = Mat::eye(3, 3, CV_64F);
    Mat prevT = cv::Mat::zeros(cv::Size(1, 3), CV_64F);

    debugfile << " Prev Frame 0 - R " << prevR << endl;
    debugfile << " Prev Frame 0 - t " << prevT << endl;

    debugfile << "" << endl;
    debugfile << " Curr Frame 1 - R " << currR << endl;
    debugfile << " Curr Frame 1 - t " << currT << endl;
    
    triangulation(points1, points2, currR, currT, currPoints3d);

    Mat prevImage = img_2;
    Mat prevImage_sem = img_2_sem;
    Mat currImage;
    vector<Point2f> currFeatures;
    vector<Point2f> prevFeatures = points2;
    vector<Point3d> prevpoints3d = currPoints3d;
    prevR = currR;
    prevT = currT;

    cout << " curr points" << " "  << currPoints3d.size() << endl;

    // for (size_t j=0; j < currPoints3d.size()-1; j++) 
    // {
    //     cout << " curr points" << " " << currPoints3d[j] << endl;
    // }

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

    for(int numFrame=102; numFrame < MAX_FRAME; numFrame++)	
    {
        cout << " loopin " << numFrame << endl;
        // debugfile << "" << endl;
        // debugfile << " Prev frame "<<  numFrame-1 << "- R " << prevR << endl;
        // debugfile << " Prev frame "<<  numFrame-1 << "- t " << prevT << endl;

        // sprintf(filename, "/home/vagrant/shared/scania_raw_undistorted/%06d.png", numFrame);
        sprintf(filename, "/home/vagrant/shared/scania_images_undistorted/frame%06d.png", numFrame);
        sprintf(filename_sem, "/home/vagrant/shared/scania_sem_images/frame%06d.png", numFrame);

        // ofstream triFile;
        // char triname[100];
        // sprintf(triname, "triangulate%06d.txt", numFrame);
        // triFile.open (triname);

        Mat currImage_c = imread(filename);
        Mat currImage_sem = imread(filename_sem);


        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
        clahe->apply(currImage, currImage);
        // cv::fisheye::undistortImage(currImage_d,currImage,cameraMatrix,D,R);
        // vector<uchar> status;

        // if (prevFeatures.size() < MIN_NUM_FEAT)
        // {
        //     featureDetection(prevImage, prevFeatures);
        //     filterSem(prevImage_sem, prevFeatures); 
        //     featureTracking(prevImage,currImage,prevFeatures,currFeatures);
        //     // filterSem(currImage_sem, currFeatures); 
        // }
        // else
        // {
        //     featureTracking(prevImage, currImage, prevFeatures, currFeatures);
        //     cout << " curr points" << " " << currFeatures.size() << endl;
        //     filterSem(currImage_sem, currFeatures);
        //     cout << " curr points" << " " << currFeatures.size() << endl;
        // }
        featureDetection(prevImage, prevFeatures);
        cout << " curr points" << " " << prevFeatures.size() << endl;

        filterSem(prevImage_sem, prevFeatures); 
        cout << " curr points" << " " << prevFeatures.size() << endl;

        featureTracking(prevImage,currImage,prevFeatures,currFeatures);
        cout << " curr points" << " " << currFeatures.size() << endl;
        
         

        // cout<< prevFeatures.size()<<endl;
        // cout<< currFeatures.size()<<endl;
        
        E = findEssentialMat(currFeatures, prevFeatures, cameraMatrix, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, cameraMatrix, currR, currT, mask);

        // debugfile << "" << endl;
        // debugfile << " Curr frame "<<  numFrame << "- R " << currR << endl;
        // debugfile << " Curr frame "<<  numFrame << "- t " << currT << endl;

        // debugfile << "" << endl;
        // debugfile << "" << endl;

        // debugfile << " frame "<<  numFrame << "- R " << currR << endl;
        // debugfile << " frame "<<  numFrame << "- t " << currT <<endl;

        vector<Point3d> currPoints3d;
        triangulation(prevFeatures, currFeatures, currR, currT, currPoints3d);

        // triFile << "" << endl;
        // for (size_t j=0; j < currPoints3d.size()-1; j++) 
        // {
        //     triFile << " curr points" << " " << currPoints3d[j] << endl;
        // }

        scale = 0;
        count = 0;
        
        for (size_t j=0; j < currPoints3d.size()-1; j++) 
        {
            for (size_t k=j+1; k< currPoints3d.size(); k++) 
            {
                double s = norm(prevpoints3d[j] - prevpoints3d[k]) / norm(currPoints3d[j] - currPoints3d[k]);
                // cout << s << endl;
                // debugfile << "index j" << j << "index k" << k << endl;
                // debugfile << "" << currPoints3d[j] << "" << currPoints3d[k] << endl;
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

        // debugfile << "" << endl;
        // debugfile << "" << endl;

        // debugfile << " frame "<<  numFrame << "- Propagated R " << R_f << endl;
        // debugfile << " frame "<<  numFrame << "- Propagated t " << t_f << endl;

        prevImage = currImage.clone();
        prevImage_sem = currImage_sem.clone();
        prevFeatures = currFeatures;
        prevpoints3d = currPoints3d;
        prevR = currR;
        prevT = currT;
        prevScale = scale;

        

        vector<double> Quat(4);
        Quat = getQuaternion(R_f);
        myfile << setprecision(6) << timesteps[numFrame] << setprecision(7) << " " << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2)
            << " " << Quat[0] << " " << Quat[1] << " " << Quat[2] << " " << Quat[3] << endl;


        prevImage = currImage.clone();
        prevFeatures = currFeatures;
        prevpoints3d = currPoints3d;
        prevR = currR;
        prevT = currT;

        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(1)) + 100;
        circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

        rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

        std::vector<cv::KeyPoint> keypoints;
        for( size_t i = 0; i < currFeatures.size(); i++ ) 
        {
            keypoints.push_back(cv::KeyPoint(currFeatures[i], 1.f));
        }

        drawKeypoints(currImage, keypoints, currImage, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

        imshow( "Road facing camera", currImage );
        imshow( "Trajectory", traj );

        waitKey(10);


    }
    waitKey(10);
    return 0;
}