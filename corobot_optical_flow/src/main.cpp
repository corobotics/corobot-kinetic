#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

#define MAX_FRAME 10

#define MAX_X 1200
#define MAX_Z 1200


/*********************************************************************************************
 * Kanade-Lucas-Tomasi feature tracker is used for finding sparse pixel wise correspondences. 
 * The KLT algorithm assumes that a point in the nearby space, and uses image gradients to 
 * find the best possible motion of the feature point.
 *********************************************************************************************/
void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)
{
    // this function track points from points1 in img1 tracks
    // img2 and stores points in point2
    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
}


void featureDetection(Mat img_1, vector<Point2f>& points1)
{
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    // FAST (Features from Accelerated Segment Test) corner detector
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}


int main( int argc, char** argv )
{
    Mat img;
    
    bool init = false;
    
    string line;
    double x =0, y=0, z = 0, input;
    double x_prev = 0, y_prev = 0, z_prev = 0;
    
    double scale = 1;
    char filename1[200];
    
    Mat E, R, t, mask;
    Mat R_f, t_f; //the final rotation and tranlation vectors
    
    sprintf(filename1, "/home/corobot/Downloads/input_files/%06d.png", 0);
    
    ifstream groundTruthfile ("/home/corobot/Downloads/input_files/00.txt");
    
    if(!groundTruthfile.is_open())
    {
        cout << "Unable to open ground truth file" << std::endl;
        return 0;
    }
    
    // getting ground truth coordinates of first frame
    getline (groundTruthfile,line);
    
    std::istringstream in(line);
    //cout << line << '\n';
    for (int j=0; j<12; j++)
    {
        in >> input;
        if (j==11) z = input;
        if (j==7) y = input;
        if (j==3)  x = input;
    }
    
    //read the first two frames from the dataset
    Mat img_c = imread(filename1);
    
    if ( !img_c.data)
    {
        std::cout<< "Error reading image file " << std::endl;
        return -1;
    }
    
    // we work with grayscale images
    cvtColor(img_c, img, COLOR_BGR2GRAY);
   
    //vectors to store the coordinates of the feature points
    vector<Point2f> prevFeatures;
    
    //detect features in img
    featureDetection(img, prevFeatures);
    
    // focal length of the camera
    double focal = 718.8560;
    // principle point of the camera
    cv::Point2d pp(607.1928, 185.2157);
    
    Mat prevImage = img;
    Mat currImage;
    vector<Point2f> currFeatures;
    
    char filename[100];
    
    namedWindow( "Camera", WINDOW_AUTOSIZE );// Create a window for display.
    namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
    
    Mat traj = Mat::zeros(MAX_X, MAX_Z, CV_8UC3);
    
    for(int numFrame=1; numFrame < MAX_FRAME; numFrame++)
    {
        sprintf(filename, "/home/corobot/Downloads/input_files/%06d.png", numFrame);
        
        // ground truth coordinates
        z_prev = z;
        x_prev = x;
        y_prev = y;

        getline (groundTruthfile,line);
        std::istringstream in(line);

        //cout << line << '\n';
    
        for (int j=0; j<12; j++)
        {
            in >> input;
            if (j==11) z = input;
            if (j==7) y = input;
            if (j==3)  x = input;
        }
        
        // find scale from ground truth coordinates
        scale = sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev));
        
        Mat currImage_c = imread(filename);
        
        if (!currImage_c.data)
        {
            std::cout<< "Error reading image file " << std::endl;
            return -1;
        }
        
        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
        vector<uchar> status;
        
        featureDetection(prevImage, prevFeatures);
        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
        
        // RANSAC Random sample consensus
        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
        
        //cout << "Scale is " << scale << endl;
        
        if(!init)
        {
            R_f = R.clone();
            t_f = t.clone();
            init = true;
        }
        else
        {
            t_f = t_f + scale*(R_f*t);
            R_f = R*R_f;
        }
        
        //cout << R << std::endl;
        
        prevImage = currImage.clone();
        
        int myx = int(t_f.at<double>(0));
        int myz = int(t_f.at<double>(2));
        
        circle(traj, Point(myx+ MAX_X/2, myz + MAX_Z/5) ,1, CV_RGB(255,0,0), 2);
        circle(traj, Point(x + MAX_X/2, z + MAX_Z/5) ,1, CV_RGB(0,0,255), 2);
        
        imshow( "Camera", currImage_c );
        imshow( "Trajectory", traj );
        
        waitKey(1);
        
    }

    waitKey(100);
    return 0;
}
