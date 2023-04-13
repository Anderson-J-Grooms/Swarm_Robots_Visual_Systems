
// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help(const char* programName)
{
    cout <<
    "\nA program using pyramid scaling, Canny, contours and contour simplification\n"
    "to find squares in a list of images (pic1-6.png)\n"
    "Returns sequence of squares detected on the image.\n"
    "Call:\n"
    "./" << programName << " [file_name (optional)]\n"
    "Using OpenCV version " << CV_VERSION << "\n" << endl;
}


int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray, gray0;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;
        cvtColor(image, gray0, COLOR_BGR2HSV);
        inRange(gray0, Scalar(0,0,0), Scalar(180,255,50), gray);
            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(approx)) > 1000 &&
                    isContourConvex(approx) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle 
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 ) {
                        squares.push_back(approx);
                        break;
                    }
                    
                }
            }
        }



int main(int argc, char** argv)
{
    vector<Point3f> objectPoints;
    vector<Point2f> imagePoints;
    Mat reversedImagePoints;


    objectPoints.push_back(Point3f(-4.9,-4.9,0));
    objectPoints.push_back(Point3f(-4.9,4.9,0));
    objectPoints.push_back(Point3f(4.9,-4.9,0));
    objectPoints.push_back(Point3f(4.9,4.9,0));
    
    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    
    cameraMatrix.at<double>(0,0) = 730;
    cameraMatrix.at<double>(1,1) = 730;
    cameraMatrix.at<double>(0,2) = 320;
    cameraMatrix.at<double>(1,2) = 240;
    
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = 0;//33.6;
    distCoeffs.at<double>(0, 1) = 0;//-2.9;
    distCoeffs.at<double>(0, 2) = 0;
    distCoeffs.at<double>(0, 3) = 0;
    distCoeffs.at<double>(0, 4) = 0;//21.5;
    
    Mat rvec, tvec;
	Mat image;
    vector<vector<Point> > squares;
    
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    int c = 0;
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
    {
        c++;
        if (c % 100) { 
            c = 0;
        // wait for a new frame from camera and store it into 'frame'
        cap.read(image);
        // check if we succeeded
        if (image.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
    findSquares(image, squares);
    if (squares.size() != 0) {
    for( int i = 0; i < 1; i++)
    {
        imagePoints.clear();
        polylines(image, squares[i], true, Scalar(0, 255, 0), 3, LINE_AA);
        for( int j = 0; j < 4; j++)
        {
            circle(image, squares[i][j], 1, Scalar(255,0,0), -1);
            imagePoints.push_back(squares[i][j]);
        }
        sort(imagePoints.begin(), imagePoints.end(), [](const Point2f &a,const Point2f &b) {
            return (a.x < b.x );
            });
            
        sort(imagePoints.begin(), imagePoints.begin() + 2, [](const Point2f &a,const Point2f &b) {
            return (a.y < b.y );
            });
        sort(imagePoints.begin() + 2 , imagePoints.end(), [](const Point2f &a,const Point2f &b) {
            return (a.y < b.y );
            });


        solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        double distance = norm(tvec);
        projectPoints(objectPoints, rvec, tvec, cameraMatrix,distCoeffs,reversedImagePoints);
        double theta0 = atan(tvec.at<double>(0,0) / tvec.at<double>(0,2));
      //  double theta1 = norm(rvec);
        Mat rotation_matrix, mtxR, mtxQ, Qx, Qy, Qz;
        Rodrigues(rvec, rotation_matrix);
        RQDecomp3x3(rotation_matrix, mtxR, mtxQ, Qx, Qy, Qz);


        double thetay1 = asin(Qy.at<double>(0,2));


        //double roll, pitch, yaw;
        //roll = atan2(rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
        //pitch = atan2(-rotation_matrix.at<double>(2,0), sqrt(pow(rotation_matrix.at<double>(2,1),2) + pow(rotation_matrix.at<double>(2,2),2)));
        //yaw = atan2(rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0));
            cout << "distance" << distance << endl;


        //    cout << "Roll: " << roll * 180 / CV_PI << endl;
        //    cout << "pitch: " << pitch * 180 / CV_PI << endl;
          //  cout << "Yaw: " << yaw * 180 / CV_PI << endl;
           cout << "angle 0: " << theta0  * 180 / CV_PI << endl;
            cout << "angle 1: " << thetay1 * 180 / CV_PI << endl;


    }
}
    imshow(wndname, image);
    if (waitKey(5) >= 0)
            break;
    }
}
    return 0;
}

