#include "tracker.h"

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

using namespace cv;
using namespace std;




int main( int argc, char** argv )
{
    Mat img_1, img_2;
    Mat R_f, t_f;
    string time_file = "/home/vagrant/shared/Kitti/00/times.txt";

    vector<double> timesteps;
    getFileContent(time_file,timesteps);

    ofstream myfile;
    myfile.open ("traj.txt");
    
    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprintf(filename1, "/home/vagrant/shared/Kitti/00/image_1/%06d.png", 0);
    sprintf(filename2, "/home/vagrant/shared/Kitti/00/image_1/%06d.png", 1);

    img_1 = imread(filename1);
    img_2 = imread(filename2);

    cout << timesteps[4540] << endl;



    return 0;
}