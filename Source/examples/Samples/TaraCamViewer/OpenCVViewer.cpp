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
	if(!_Disparity.InitCamera(false, false)) { //Initialise the camera
		if(DEBUG_ENABLED)
			cout << "Camera Initialisation Failed!\n";
		return FALSE;
	}

	_Disparity.SetAutoExposure();

	if(!GetRevision(&g_eRev)) {
		printf("Init GetRevision Failed");
		return FALSE;
	}

	//Configuring IMU rates
	IMUCONFIG_TypeDef lIMUConfig {
		IMU_ACC_GYRO_ENABLE,
		IMU_ACC_X_Y_Z_ENABLE,
		IMU_ACC_SENS_2G,
		IMU_GYRO_X_Y_Z_ENABLE,
		IMU_GYRO_SENS_250DPS,
		IMU_ODR_208HZ
	};

//  if(g_eRev == REVISION_A) {
//		lIMUConfig.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_DPS;
//		lIMUConfig.IMU_ODR_CONFIG = IMU_ODR_HZ;
//	}
//	else if(g_eRev == REVISION_B) {
//		lIMUConfig.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_250DPS;
//		lIMUConfig.IMU_ODR_CONFIG = IMU_ODR_104HZ;
//	} else {
//		printf("Unknown camera revision:%d", g_eRev);
//		return FALSE;
//	}

	//Setting the configuration using HID command
	BOOL uStatus = SetIMUConfig(lIMUConfig);
	if(!uStatus) {
		cout << "SetIMUConfig Failed\n";
		return FALSE;
	}

	IMUCONFIG_TypeDef tmpIMUConfig;
	//Reading the configuration to verify the values are set
	uStatus = GetIMUConfig(&tmpIMUConfig);
	if(!uStatus) {
		cout << "GetIMUConfig Failed\n";
		return FALSE;
	}

//	if(		(g_eRev == REVISION_A &&
//					(lIMUConfig.GYRO_SENSITIVITY_CONFIG != IMU_GYRO_SENS_245DPS || lIMUConfig.IMU_ODR_CONFIG != IMU_ODR_238HZ))
//			||
//			(g_eRev == REVISION_B &&
//					(lIMUConfig.GYRO_SENSITIVITY_CONFIG != IMU_GYRO_SENS_250DPS || lIMUConfig.IMU_ODR_CONFIG != IMU_ODR_104HZ))) {
	if (tmpIMUConfig.GYRO_SENSITIVITY_CONFIG != lIMUConfig.GYRO_SENSITIVITY_CONFIG
			|| tmpIMUConfig.IMU_ODR_CONFIG != lIMUConfig.IMU_ODR_CONFIG) {
		cout << "GetIMUConfig Bad config\n";
		return FALSE;
	}

	//Streams the camera and process the height
	TaraViewer();

	return TRUE;
}

int OpenCVViewer::FrameWriter(char *SequenceDirectoryBuf)
{
	char FilenameBuf[240];

	int FrameIndex = 0;
	timeval timeval_start;

    while(saveFramesAndIMU)
    {
        std::unique_lock<std::mutex> lk(frameQueueMux);
        frameQueueCond.wait(lk,[&]{return !frameQueue.empty();});

        FrameQueueStruct *fqs = frameQueue.front();
        frameQueue.pop();

        lk.unlock();

        if (FrameIndex == 0) {
           	std::memcpy(&timeval_start, &(fqs->timeVal), sizeof(timeval));
        }

		int milliseconds = fqs->timeVal.tv_sec * 1000 + fqs->timeVal.tv_usec / 1000 - timeval_start.tv_sec * 1000 - timeval_start.tv_usec / 1000;
		sprintf(FilenameBuf, "%s/%04d_%06d_L.png", SequenceDirectoryBuf, FrameIndex, milliseconds);
		imwrite(FilenameBuf, fqs->left);
		*strrchr(FilenameBuf, 'L') = 'R';
		imwrite(FilenameBuf, fqs->right);
        FrameIndex++;
    }
    return FrameIndex - 1;
}

int OpenCVViewer::imuWriter(char *SequenceDirectoryBuf, pthread_mutex_t *imuDataReadyMux) {
	char FilenameBuf[240];
	sprintf(FilenameBuf, "%s/IMU.txt", SequenceDirectoryBuf);

	FILE *imuFile = fopen(FilenameBuf, "w");
	if (!imuFile) {
		cout << "Error creating " << FilenameBuf << "for writing. " << endl;
		return -1;
	}

	IMUDATAOUTPUT_TypeDef *imuSample = NULL;

	while (saveFramesAndIMU) {
		std::unique_lock<std::mutex> lk(imuQueueMux);
		imuQueueCond.wait(lk,[&]{return !imuQueue.empty();});
		imuSample = imuQueue.front();
		imuQueue.pop();
		lk.unlock();

		fprintf(imuFile, "%06d-%06d:%f\t%f\t%f\t\t%f\t%f\t%f\n",
				imuSample->imuSampleIndex,
				imuSample->millisecond,
				imuSample->accX,
				imuSample->accY,
				imuSample->accZ,
				imuSample->gyroX,
				imuSample->gyroY,
				imuSample->gyroZ);
	}
	fflush(imuFile);
	fclose(imuFile);

	return imuSample ? imuSample->imuSampleIndex : 0;
}

//Streams using OpenCV Application
//Converts the 24 bit data to 2 x 10bit data for left and right images
int OpenCVViewer::TaraViewer()
{
	const int numberOfFrameQueueStructs = 256;
	char WaitKeyStatus;
	Mat FullImage;
	int BrightnessVal = 4;		//Default Value
	bool rectifyImages = false;

	FrameQueueStruct frameQueueStructs[numberOfFrameQueueStructs];
	int frameQueueStructsIndex = 0;

	char TimeStampBuf[32];
	char FilenameBuf[240];
	char SequenceDirectoryBuf[240];

	char cwd[256];
	getcwd(cwd, sizeof(cwd));

	time_t t;
	tm tm;
	timeval tv;

	std::future<int> frameWriterFuture;
	std::future<BOOL> imuReaderFuture;
	std::future<int> imuWriterFuture;

	//Mutex initialization
	pthread_mutex_t	imuDataReadyMux;
	pthread_mutex_init(&imuDataReadyMux, NULL);
	pthread_mutex_lock(&imuDataReadyMux);

	auto stopSavingFramesAndIMU = [&]() {
		saveFramesAndIMU = false;

		cout << "Last frame index:" << frameWriterFuture.get() << endl;
		cout << "Last IMU sample index:" << imuWriterFuture.get() << endl;

		// Resetting to IMU disabled mode
		IMUDATAINPUT_TypeDef imuInput = {IMU_CONT_UPDT_DIS, IMU_AXES_VALUES_MIN};
		if(!ControlIMUCapture(&imuInput)) {
			cout << "ControlIMUCapture Failed" << endl;
		}

		if (imuReaderFuture.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout) {
			cout << "Timeout waiting for IMU reader to finish." << endl;
		} else if (!imuReaderFuture.get()) {
			cout << "IMU reader finish error!!" << endl;
		}
	};


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
	cout << "Press r/R on the Image Window to toggle image rectification." << endl << endl;

	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	string Inputline;

	//Streams the Camera using the OpenCV Video Capture Object
	while(1)
	{
		frameQueueStructsIndex++;
		frameQueueStructsIndex %= numberOfFrameQueueStructs;
		FrameQueueStruct *fqs = &(frameQueueStructs[frameQueueStructsIndex]);

		if(!_Disparity.GrabFrame(&(fqs->left), &(fqs->right), &(fqs->timeVal), rectifyImages)) //Reads and returns L/R images
		{
			destroyAllWindows();
			break;
		}

		int frameQueueSize = 0;
		if (!saveFramesAndIMU) {
			//concatenate both the image as single image
			hconcat(fqs->left, fqs->right, FullImage);
			imshow("Input Image", FullImage);
		} else {
			std::lock_guard<std::mutex> lk(frameQueueMux);
			frameQueue.push(fqs);
			frameQueueSize = frameQueue.size();
			frameQueueCond.notify_one();
		}

		if (frameQueueSize >= numberOfFrameQueueStructs) {
			cout << "frameQueue overflow!! Queue size:" << frameQueueSize << " numberOfFrameQueueStructs:" << numberOfFrameQueueStructs << endl;
			return FALSE;
		} else if (frameQueueSize >= (numberOfFrameQueueStructs * 3) / 4) {
			cout << "frameQueue 3/4 full! Queue size:" << frameQueueSize << " numberOfFrameQueueStructs:" << numberOfFrameQueueStructs << endl;
		}


		//waits for the Key input
		WaitKeyStatus = waitKey(1);
		if(WaitKeyStatus == 'q' || WaitKeyStatus == 'Q' || WaitKeyStatus == 27) //Quit
		{
			if (saveFramesAndIMU) {
				stopSavingFramesAndIMU();
			}
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
			imwrite(FilenameBuf, fqs->left);
			std::sprintf(FilenameBuf, "%s/R_%s.png", cwd, TimeStampBuf);
			imwrite(FilenameBuf, fqs->right);
		}
		else if(WaitKeyStatus == 'v' || WaitKeyStatus == 'V') // Toggle saving frames
		{
			if (!saveFramesAndIMU) {
				saveFramesAndIMU = true;
				time(&t);
				localtime_r(&t, &tm);
				std::sprintf(SequenceDirectoryBuf, "%s/Sequence_%02d%02d%02d_%02d%02d%02d", cwd, tm.tm_mday, tm.tm_mon + 1, tm.tm_year - 100, tm.tm_hour, tm.tm_min, tm.tm_sec);
				if (mkdir(SequenceDirectoryBuf, 0755) == 0) {
					cout << "Saving frames to " << SequenceDirectoryBuf << endl;
				} else {
					cout << "Error creating directory " << SequenceDirectoryBuf << " errno:" << errno << endl;
					saveFramesAndIMU = false;
				}

				sprintf(FilenameBuf, "%s/calib_cam_to_cam.txt", SequenceDirectoryBuf);

				FILE *c2cFile = fopen(FilenameBuf, "w");
				if (!c2cFile) {
					cout << "Error creating " << FilenameBuf << "for writing. " << endl;
					return FALSE;
				}

				fprintf(c2cFile, "P_rect_01:");
				MatIterator_<double> it, end;
				Mat PRect = _Disparity._TaraCamParameters.PRect2.clone();
				PRect.at<double>(0, 3) = PRect.at<double>(0, 3) * 0.001; // mm to meters.
				for( it = PRect.begin<double>(), end = PRect.end<double>(); it != end; ++it)
				{
					fprintf(c2cFile, " %.6e", *it);
				}
				fprintf(c2cFile, "\n");

				fflush(c2cFile);
				fclose(c2cFile);

				//cout << _Disparity._TaraCamParameters.P2 << endl;
				//cout << _Disparity._TaraCamParameters.PRect2 << endl;

				frameWriterFuture = std::async(std::bind(&OpenCVViewer::FrameWriter, this, SequenceDirectoryBuf));

				imuReaderFuture = std::async(&GetIMUValueBuffer, &imuQueueMux, &imuQueueCond, &imuQueue);
				//Configuring IMU update mode
				IMUDATAINPUT_TypeDef imuInput = {IMU_CONT_UPDT_EN, IMU_AXES_VALUES_MIN};
				if(!ControlIMUCapture(&imuInput)) {
					cout << "ControlIMUCapture Failed\n";
				}
				imuWriterFuture = std::async(std::bind(&OpenCVViewer::imuWriter, this, SequenceDirectoryBuf, &imuDataReadyMux));
			} else {
				stopSavingFramesAndIMU();
			}
		}
		else if (WaitKeyStatus == 'r' || WaitKeyStatus == 'r') // Toggle image rectification
		{
			if (rectifyImages) {
				rectifyImages = false;
				cout << endl << "Disabled image rectification." << endl;
			} else {
				rectifyImages = true;
				cout << endl << "Enabled image rectification." << endl;
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
