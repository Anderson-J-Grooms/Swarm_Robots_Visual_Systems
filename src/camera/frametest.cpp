#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int main()
{
	VideoCapture webcam_0(0);                    //open stream
	if(!webcam_0.isOpened())
	{
		cout << "error: cannot open stream between webcam_0 and application" << endl;
		waitKey(0);
		return -1;
	}

	Mat frame;                                   //Mat header for frame storing from webcam_0
	int i = 0;                                   //index, we use it for testing
	
	while ((i++ < 100) && !webcam_0.read(frame)) //skip unread frames
	{
		cout << "frame " << i << " skipped" << endl;
	}

	if (i >= 100)                                //check webcam_0 failure
	{
		cout << "cannot read frames from webcam_0, check drivers" << endl;
		waitKey(0);
		return -1;
	} else
	{
		cout << "cam is ready" << endl;
	}

	char * window_name = "webcam_0 test; press \"ESC\" to exit";
	namedWindow(window_name, WINDOW_AUTOSIZE);//make new window
	imshow(window_name, frame);                  //output 1st successfully read frame
	waitKey(10);
	
	while(1)                                     //reading frames from webcam_0
	{
		if(!webcam_0.read(frame))
		{
			cout << "cannot get frame from webcam_0, check drivers" << endl;
			waitKey(0);
			return -1;
		}
		imshow(window_name, frame);

		if (waitKey(10) == 27)                   //check if ESC is pressed
		{
			cout << "bye!" << endl;
			waitKey(0);
			break;
		}
	}
	return 0;
}
