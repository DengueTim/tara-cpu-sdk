///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, e-con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**********************************************************************
	OpenCVViewer: Defines the methods to view the stereo images from
				   the camera.

**********************************************************************/

#include "OpenCVViewer.h"
#include <ctype.h>
#include <sys/stat.h>

using namespace cv;
using namespace std;
using namespace Tara;

//Initialises all the variables and methods
int OpenCVViewer::Init()
{
	cout << endl  << "		OpenCV Viewer Application " << endl << " Displays the rectified left and right image!" << endl  << endl;

	//Init
	if(!_Disparity.InitCamera(false, false)) //Initialise the camera
	{
		if(DEBUG_ENABLED)
			cout << "Camera Initialisation Failed!\n";
		return FALSE;
	}

	//Streams the camera and process the height
	TaraViewer();

	return TRUE;
}

//Streams using OpenCV Application
//Converts the 10 bit data to 8 bit data and splits the left an d right image separately
int OpenCVViewer::TaraViewer()
{
	char WaitKeyStatus;
	Mat LeftImage, RightImage, FullImage;
	int BrightnessVal = 4;		//Default Value

	char TimeStampBuf[32];
	char FilenameBuf[240];
	char SequenceDirectoryBuf[240];

	char cwd[256];
	getcwd(cwd, sizeof(cwd));

	time_t t;
	tm tm;
	timeval tv,tv2;

	bool SaveFrames = false;
	int FrameIndex = 0;

	//Window Creation
	namedWindow("Input Image", WINDOW_AUTOSIZE);

	cout << endl << "Press q/Q/Esc on the Image Window to quit the application!" << endl;
	cout << "Press b/B on the Image Window to change the brightness of the camera" << endl;
	cout << "Press t/T on the Image Window to change to Trigger Mode" << endl;
	cout << "Press m/M on the Image Window to change to Master Mode" << endl;
	cout << "Press a/A on the Image Window to change to Auto exposure  of the camera" << endl;
	cout << "Press e/E on the Image Window to change the exposure of the camera" << endl;
	cout << "Press s/S on the Image Window to save a single frame" << endl;
	cout << "Press v/V on the Image Window to toggle saving frames" << endl << endl;

	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	string Inputline;

	//Streams the Camera using the OpenCV Video Capture Object
	while(1)
	{
		if(!_Disparity.GrabFrame(&LeftImage, &RightImage)) //Reads the frame and returns the rectified image
		{
			destroyAllWindows();
			break;
		}

		//concatenate both the image as single image
		hconcat(LeftImage, RightImage, FullImage);
		imshow("Input Image", FullImage);

		if (SaveFrames) {
			if (FrameIndex == 0) {
				gettimeofday(&tv,NULL);
			}
			gettimeofday(&tv2,NULL);
			int milliseconds = tv2.tv_sec * 1000 + tv2.tv_usec / 1000 - tv.tv_sec * 1000 - tv.tv_usec / 1000;
			sprintf(FilenameBuf, "%s/%04d_%06d_L.png", SequenceDirectoryBuf, FrameIndex, milliseconds);
			imwrite(FilenameBuf, LeftImage);
			sprintf(FilenameBuf, "%s/%04d_%06d_R.png", SequenceDirectoryBuf, FrameIndex, milliseconds);
			imwrite(FilenameBuf, RightImage);
			FrameIndex++;
		}

		//waits for the Key input
		WaitKeyStatus = waitKey(1);
		if(WaitKeyStatus == 'q' || WaitKeyStatus == 'Q' || WaitKeyStatus == 27) //Quit
		{
			destroyAllWindows();
			break;
		}

		//Sets up the mode
		else if(WaitKeyStatus == 'T' || WaitKeyStatus == 't' ) //Stream Mode 0 - Trigger Mode 1 - Master Mode
		{
			if(_Disparity.SetStreamMode(TRIGGERMODE))
			{
				cout << endl << "Switching to Trigger Mode!!" << endl;
			}
			else
			{
				cout << endl << "Selected mode and the current mode is the same!!" << endl;
			}
		}

		//Sets up the mode
		else if(WaitKeyStatus == 'M' || WaitKeyStatus == 'm' ) //Stream Mode 0 - Trigger Mode 1 - Master Mode
		{
			if(_Disparity.SetStreamMode(MASTERMODE))
			{
				cout << endl << "Switching to Master Mode!!" << endl;
			}
			else
			{
				cout << endl << "Selected mode and the current mode is the same!!" << endl;
			}
		}
		//Sets up Auto Exposure
		else if(WaitKeyStatus == 'a' || WaitKeyStatus == 'A' ) //Auto Exposure
		{
			_Disparity.SetAutoExposure();
		}
		else if(WaitKeyStatus == 'e' || WaitKeyStatus == 'E') //Set Exposure
		{
			cout << endl << "Enter the Exposure Value Range(10 to 1000000 micro seconds): " << endl;

			ManualExposure = 0;
			cin >> ws; //Ignoring whitespaces

			while(getline(std::cin, Inputline)) //To avoid floats and Alphanumeric strings
			{
				std::stringstream ss(Inputline);
				if (ss >> ManualExposure)
				{
					if (ss.eof())
					{
						if(ManualExposure >= SEE3CAM_STEREO_EXPOSURE_MIN && ManualExposure <= SEE3CAM_STEREO_EXPOSURE_MAX)
						{
							//Setting up the exposure
							_Disparity.SetExposure(ManualExposure);
						}
						else
						{
							cout << endl << " Value out of Range - Invalid!!" << endl;
						}
						break;
					}
				}
				ManualExposure = -1;
				break;
			}

			if(ManualExposure == -1)
			{
				cout << endl << " Value out of Range - Invalid!!" << endl;
			}
		}
		else if(WaitKeyStatus == 'b' || WaitKeyStatus == 'B') //Brightness
		{
			cout << endl << "Enter the Brightness Value, Range(1 to 7): " << endl;

			BrightnessVal = 0;
			cin >> ws; //Ignoring whitespaces

			while(getline(cin, Inputline)) //To avoid floats and Alphanumeric strings
			{
				std::stringstream ss(Inputline);
				if (ss >> BrightnessVal)
				{
					if (ss.eof())
					{
						//Setting up the brightness of the camera
						if (BrightnessVal >= 1  && BrightnessVal <= 7)
						{
							//Setting up the brightness
                					//In opencv-linux 3.1.0, the value needs to be normalized by max value (7)
                					_Disparity.SetBrightness((double)BrightnessVal / 7.0);
						}
						else
						{
							 cout << endl << " Value out of Range - Invalid!!" << endl;
						}
						break;
					}
				}
				BrightnessVal = -1;
				break;
			}

			if(BrightnessVal == -1)
			{
				cout << endl << " Value out of Range - Invalid!!" << endl;
			}
		}
		else if(WaitKeyStatus == 's' || WaitKeyStatus == 'S') // Save a frame
		{
			time(&t);
			localtime_r(&t, &tm);
			gettimeofday(&tv,NULL);
			int millisecs = tv.tv_usec / 1000;
			std::sprintf(TimeStampBuf, "%02d%02d%02d_%02d%02d%02d_%04d", tm.tm_mday, tm.tm_mon + 1, tm.tm_year - 100, tm.tm_hour, tm.tm_min, tm.tm_sec, millisecs);
			cout << "Writing left and right images with time stamp:" << TimeStampBuf << endl;
			std::sprintf(FilenameBuf, "%s/L_%s.png", cwd, TimeStampBuf);
			imwrite(FilenameBuf, LeftImage);
			std::sprintf(FilenameBuf, "%s/R_%s.png", cwd, TimeStampBuf);
			imwrite(FilenameBuf, RightImage);
		}
		else if(WaitKeyStatus == 'v' || WaitKeyStatus == 'V') // Toggle saving frames
		{
			SaveFrames = !SaveFrames;
			if (SaveFrames)
			{
				FrameIndex = 0;
				time(&t);
				localtime_r(&t, &tm);
				std::sprintf(SequenceDirectoryBuf, "%s/Sequence_%02d%02d%02d_%02d%02d%02d", cwd, tm.tm_mday, tm.tm_mon + 1, tm.tm_year - 100, tm.tm_hour, tm.tm_min, tm.tm_sec);
				if (mkdir(SequenceDirectoryBuf, 0755) == 0) {
					cout << "Saving frames to " << SequenceDirectoryBuf << endl;
				} else {
					cout << "Error creating directory " << SequenceDirectoryBuf << " errno:" << errno << endl;
					SaveFrames = false;
				}
			}
			else
			{
				cout << "Last frame index:" << FrameIndex << endl;
			}
		}
	}

	return TRUE;
}

//Main function
int main()
{
	if(DEBUG_ENABLED)
	{
		cout << "OpenCV Camera Viewer\n";
		cout << "--------------------\n\n";
	}

	//Object to access the application to view the stereo images
	OpenCVViewer _OpenCVViewer;
	_OpenCVViewer.Init();

	if(DEBUG_ENABLED)
		cout << "Exit : OpenCV Camera Viewer\n";

	return TRUE;
}
