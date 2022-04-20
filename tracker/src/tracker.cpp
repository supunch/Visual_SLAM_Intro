#include "tracker.h"

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

using namespace cv;
using namespace std;

bool getFileContent(string fileName, vector<double> & vecOfStrs)
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
            vecOfStrs.push_back(stod(str));
    }
    //Close The File
    in.close();
    return true;
}


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

    cout << timesteps[4500] << endl;



    return 0;
}