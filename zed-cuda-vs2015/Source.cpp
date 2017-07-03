///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/***************************************************************************************************
** This sample demonstrates how to grab images and depth map with the ZED SDK                    **
** and apply the result in a 3D view "point cloud style" with OpenGL /freeGLUT                   **
** Some of the functions of the ZED SDK are linked with a key press event		                  **
***************************************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>
#include <time.h>

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h> 
#include <GL/gl.h>
#include <GL/glut.h>

// ZED includes
#include <sl/Camera.hpp>



//// Using std and sl namespaces
using namespace std;
using namespace sl;

//// Create ZED object (camera, callback, images)
sl::Camera zed;
sl::Mat point_cloud, depth_image, left_image, right_image;
std::thread zed_callback;
bool quit;

//// Point Cloud visualizer


//// Sample functions
void startZED();
void close();
void printHelp();

int ret_left_image;
int ret_right_image;
int ret_depth_image;

int req_display;
int req_savefile;

std::string left_format;
std::string right_format;
std:: string depth_format;

int main(int argc, char **argv) {

	cv::FileStorage storage;
	storage.open("setting.yaml", cv::FileStorage::READ);

	// Setup configuration parameters for the ZED
	InitParameters initParameters;
	if (argc == 2) initParameters.svo_input_filename = argv[1];

	int camera_resolution;
	int depth_mode;
	int coordinate_units;
	int coordinate_system;

	storage["camera_resolution"] >> camera_resolution;
	storage["depth_mode"] >> depth_mode;
	storage["coordinate_units"] >> coordinate_units;
	storage["coordinate_system"] >> coordinate_system;

	//initParameters.camera_resolution = sl::RESOLUTION_HD720;
	initParameters.camera_resolution = (RESOLUTION)camera_resolution;
	
	//initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
	initParameters.depth_mode = (DEPTH_MODE)depth_mode;
	
	//initParameters.coordinate_units = sl::UNIT_METER; // set meter as the OpenGL world will be in meters
	initParameters.coordinate_units = (UNIT)coordinate_units;

	//initParameters.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
	initParameters.coordinate_system = (COORDINATE_SYSTEM)coordinate_system;

	storage["left_image"] >> ret_left_image;
	storage["right_image"] >> ret_right_image;
	storage["depth_image"] >> ret_depth_image;

	storage["display"] >> req_display;
	storage["savefile"] >> req_savefile;

	storage["left_format"] >> left_format;
	storage["right_format"] >> right_format;
	storage["depth_format"] >> depth_format;


	storage.release();

	// Open the ZED
	ERROR_CODE err = zed.open(initParameters);
	if (err != SUCCESS) {
		cout << errorCode2str(err) << endl;
		zed.close();
		
		return 1; // Quit if an error occurred
	}

	// Print help in console
	printHelp();

	// Initialize point cloud viewer
	

	//Start ZED 
	startZED();

	/// Set GLUT callback
	//glutCloseFunc(close);
	//glutMainLoop();
	return 0;
}

/**
*  This function frees and close the ZED, its callback(thread) and the viewer
**/
void close() {
	quit = true;

	// Stop callback
	zed_callback.join();

	// Exit point cloud viewer
	

	// free buffer and close ZED
	depth_image.free(MEM_CPU);
	point_cloud.free(MEM_GPU);
	zed.close();
}

/**
*  This functions start the ZED's thread that grab images and data.
**/
void startZED() {
	quit = false;
	char key = ' ';
	int frameNum = 0;
	char filename[100];

	clock_t time1 = clock();

	while (key != 'q') {
		if (zed.grab() == SUCCESS) {
			// Get depth as a displayable image (8bits) and display it with OpenCV
			if (ret_left_image)
			{
				zed.retrieveImage(left_image, sl::VIEW_LEFT);
				cv::Mat cvLeft = cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM_CPU));
				if (req_display) cv::imshow("Left", cvLeft);
				sprintf(filename, left_format.c_str(), frameNum);
				if (req_savefile) cv::imwrite(filename, cvLeft);
			}

			if (ret_right_image)
			{
				zed.retrieveImage(right_image, sl::VIEW_RIGHT);
				cv::Mat cvRight = cv::Mat(right_image.getHeight(), right_image.getWidth(), CV_8UC4, right_image.getPtr<sl::uchar1>(sl::MEM_CPU));
				if (req_display) cv::imshow("Right", cvRight);
				sprintf(filename, right_format.c_str(), frameNum);
				if (req_savefile) cv::imwrite(filename, cvRight);
			}

			if (ret_depth_image)
			{
				zed.retrieveImage(depth_image, sl::VIEW_DEPTH); // For display purpose ONLY. To get real world depth values, use retrieveMeasure(mat, sl::MEASURE_DEPTH)
				cv::Mat cvDepth = cv::Mat(depth_image.getHeight(), depth_image.getWidth(), CV_8UC4, depth_image.getPtr<sl::uchar1>(sl::MEM_CPU));
				if (req_display) cv::imshow("Depth", cvDepth);
				sprintf(filename, depth_format.c_str(), frameNum);
				if (req_savefile) cv::imwrite(filename, cvDepth);
			}

			// Get XYZRGBA point cloud on GPU and send it to OpenGL
			zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA, sl::MEM_GPU); // Actual metrics values

			key = cv::waitKey(1);
			frameNum++;
			

			//frame rate계산
			if (frameNum % 100 == 0)
			{
				clock_t time2 = clock();
				double fps =  100 / ((time2 - time1) / (double)CLOCKS_PER_SEC);
				printf("fps : %.2lf\n", fps);
				time1 = time2;
			}
		}
		else sl::sleep_ms(1);
	}

	//왠지 버그때문에 실행하지 않음.
	//zed_callback = std::thread(run);
}

/**
* This function displays help in console
**/
void printHelp() {
	std::cout << " Press 'q' to quit" << std::endl;
}