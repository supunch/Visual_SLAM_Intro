#include "tracker.h"

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    Mat img_1, img_2;
    Mat R_f, t_f;

    //  Get timesteps from KITTI to save trajectory in TUM format
    string time_file = "/home/vagrant/shared/Kitti/00/times.txt";
    vector<double> timesteps;
    getFileContent(time_file,timesteps);
    
    ofstream myfile;
    myfile.open ("traj.txt");

    myfile << setprecision(6) << timesteps[0] << setprecision(7) << " "<< 0 << " " << 0 << " " << 0
            << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;

    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprintf(filename1, "/home/vagrant/shared/Kitti/00/image_1/%06d.png", 0);
    sprintf(filename2, "/home/vagrant/shared/Kitti/00/image_1/%06d.png", 1);

    char text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;  
    cv::Point textOrg(10, 50);

    Mat img_1_c = imread(filename1);
    Mat img_2_c = imread(filename2);

    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

    // feature tracking
    vector<Point2f> points1, points2;        
    featureDetection(img_1, points1);       
    vector<uchar> status;
    featureTracking(img_1,img_2,points1,points2, status); 

    //KITTI  
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    
    Mat E, R, t, mask;
    E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1, R, t, focal, pp, mask);

    Mat prevImage = img_2;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeatures;

    char filename[100];

    R_f = R.clone();
    t_f = t.clone();

    vector<double> Quat(4);
    Quat = getQuaternion(R_f);
    myfile << setprecision(6) << timesteps[1] << setprecision(7) << " " << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2)
            << " " << Quat[0] << " " << Quat[1] << " " << Quat[2] << " " << Quat[3] << endl;

    namedWindow( "Road facing camera", WINDOW_AUTOSIZE );
    namedWindow( "Trajectory", WINDOW_AUTOSIZE );

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)	
    {
        sprintf(filename, "/home/vagrant/shared/Kitti/00/image_1/%06d.png", numFrame);
        
        Mat currImage_c = imread(filename);
        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
        vector<uchar> status;
        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
        // scale = getAbsoluteScale(numFrame); // Cheat to get scale from groundtruth file.

        
        t_f = t_f + (R_f*t);
        R_f = R*R_f;
        
        vector<double> Quat(4);
        Quat = getQuaternion(R_f);
        myfile << setprecision(6) << timesteps[numFrame] << setprecision(7) << " " << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2)
            << " " << Quat[0] << " " << Quat[1] << " " << Quat[2] << " " << Quat[3] << endl;

        if (prevFeatures.size() < MIN_NUM_FEAT)
        {
            featureDetection(prevImage, prevFeatures);
            featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(2)) + 100;
        circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

        rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

        imshow( "Road facing camera", currImage_c );
        imshow( "Trajectory", traj );

        waitKey(10);

    }

    return 0;
}