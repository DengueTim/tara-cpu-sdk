# Tara CPU SDK

This package lets you build and install the Tara SDK package on various platforms. This package also lets you build and install various sample applications using the Tara SDK.

### Contents
This package consists of the documents, tara SDK source code and sample applications source code.

##### Documents

1. **Building SDK Solutions** – Helps the user in building and installing the samples.
2. **IMU Sample App User Manual** – Describes the detailed usage of an IMU unit(LSM6DS0) integrated Tara.
3. **Linux API Manual** – Highlights the Tara API’s.
4. **Tara SDK Linux User Manual** – Highlights the SDK folder structure and applications that are included in the package.

##### Prebuilts

1. **Dependencies Script** - A script file to install the necessary dependencies in order to run the SDK properly.
2. **Install Script** - A script file to install the application binaries and libraries in the path '/usr/local/tara-sdk/'.
3. **Binaries for Ubuntu 16.04** - Contains the executable of all the applications in the SDK. The library files of the Tara and Extension unit and the dependency libraries of the OpenCV	are also placed. The executable and all the dependencies are built for 64 bit in Ubuntu-16.04.
4. **Uninstall Script** - A script file to uninstall the application binaries and libraries from the path '/usr/local/tara-sdk/'.

##### SDK Source

1. **Include** - Contains the header files that are required for the application development using the Tara SDK.
2. **Extension Unit** – Contains the source code of the extension unit library of the Tara camera.
3. **Tara SDK** – Contains the source code of the Tara SDK.

##### Examples

1. **Face Detection** – Detects the face in both the left and right images using LBP Cascade Classifier of OpenCV. The Point from the face after detection is converted to a 3D point and the depth of the person from the camera is displayed.
2. **Height Calibration** – This is used for calibrating the height of the base from the camera. The base is determined by averaging(minimum of 10 frames) the depth of a point selected by a user on the disparity map. The depth calculated is written to the file named "BaseHeight" and placed under the folder named "Height" which is used by the height estimation to determine the height.
3. **Height Estimation** –  The Height of the person standing under the camera is estimated with the reference to the "BaseHeight" file generated by the HeightCalibration project. The Height of the person is actually selected by scanning the depth in the 1/3 of the disparity map. The lowest depth is selected as the depth of the head. The Head depth is subtracted from the base depth and the height of the person is displayed.
4. **IMU** – A window with three circles denoting the three axis is created. The circles are rotated based on the rotation of the device.
5. **Point Cloud** - This is a sample to display point Cloud view of the scene.
5. **Tara Cam Viewer** – This is the sample to access the camera in the OpenCV. Rectified images of both the left and the right camera is displayed.
6. **Tara Depth Viewer** – Rectified images of both the left and the right camera is displayed. Disparity Map is displayed, depth of the point selected by the user is shown.
7. **Tara Disparity Viewer** – Disparity along with the left and right images are displayed.
8. **Volume Estimation** – The volume of a box in the scene is estimated.

### Installing Tara SDK Package

In order to install the Tara SDK package, execute the following commands

	cd <Cloned Location>/Prebuilts
	sudo ./dependencies.sh
	sudo ./install.sh
	source ~/.bashrc

### Run the Prebuilt Binaries

To run the binaries, make sure the the dependencies and SDK package are installed. Run the applications with the following command,

	sudo -i <BinaryName>
	For Example,
		sudo -i TaraCamViewer
		sudo -i FaceDetection

**List of Binaries:** FaceDetection, HeightCalibration, HeightEstimation, ImuApplication, PointCloud, TaraCamViewer, TaraDepthViewer, TaraDisparityViewer, VolumeEstimation.

### To uninstall

Execute the following command to uninstall the Tara SDK along with the samples

	cd <Cloned Location>/Prebuilts
	sudo ./uninstall.sh

## Support

If you need assistance with the TaraXL, visit at https://www.e-consystems.com/Request-form.asp?paper=See3CAM_Stereo or contact us at techsupport@e-consystems.com