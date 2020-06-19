#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#define PI 3.14159265
#define HEIGHTOFFSET 15
#define WIDTHOFFSET 35

using namespace std;
using namespace cv;
using namespace std;

int main(void) {

	//float FOV = 3.141592654;
	float FOV = 3.141592654 * 180 / 180;
	int x, y;
	Mat source_front = imread("fish1_copy.jpg");
	Mat source_back = imread("fish2_copy.jpg");
		
	float theta, phi, r;
	float sphX, sphY, sphZ;
	int fishX, fishY;

	float width_front = source_front.cols;
	float height_front = source_front.rows;
	float width_back = source_back.cols;
	float height_back = source_back.rows;

	Mat target_front(height_front - 15, width_front - 35, CV_8UC3);			//only for Ricoh Theta cam _ FRONT
	Mat target_back(height_back - 34, width_back - 58, CV_8UC3);			//only for Ricoh Theta cam _ BACK
	

	// Only for Ricoh Theta cam _ FRONT
	for (y = 10; y < height_front - 5; y++) {
		for (x = 30; x < width_front - 5; x++) {
			target_front.at<Vec3b>(y - 10, x - 30) = source_front.at<Vec3b>(y, x);
		}
	}
	// Only for Ricoh Theta cam _ BACK
	for (y = 14; y < height_back - 20; y++) {
		for (x = 18; x < width_back - 43; x++) {
			target_back.at<Vec3b>(y - 14, x - 18) = source_back.at<Vec3b>(y, x);
		}
	}
		
	//ofstream outFile("fisheye position.txt");
	imwrite("fish1_resize.jpg", target_front);
	imwrite("fish2_resize.jpg", target_back);

	float width = target_front.cols;
	float height = target_front.rows;
	
	Mat result_front(height, width, CV_8UC3);
	Mat result_back(height, width, CV_8UC3);
	Mat fine_front(height, width, CV_8UC3);
	Mat fine_back(height, width, CV_8UC3);

	cout << -0.5 << endl;
	cout << sin(PI * (- 0.5)) << endl;

	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			
			theta = 2.0 * PI * (x / width - 0.5);	//-pi ~ pi
			phi = PI * (y / height - 0.5);			//-pi/2 ~ pi/2

			sphX = cos(phi) * sin(theta);
			sphY = cos(phi) * cos(theta);
			sphZ = sin(phi);						//상반구 형태의 좌표계 형성

			theta = atan2(sphZ, sphX);
			phi = atan2(sqrt((sphX*sphX) + (sphZ * sphZ)), sphY);
			r = width * phi / FOV;

			fishX = 0.5 * width + r * cos(theta);
			fishY = 0.5 * width + r * sin(theta);
			/*outFile << "(" << x << "," << y << ") => fishX = " << fishX << " | fishY = " << fishY;
			outFile << "  r = " << r << ", theta = " << theta << "    sin(theta) = " << sin(theta) << endl;*/
			//outFile << theta << "\t" << phi << "\t" << sphX << "\t" << sphY << "\t" << sphZ << "\t" << endl;
			//outFile << x << "\t" << y << "\t" << fishX << "\t" << fishY << endl;

			/*if (fishX < width && fishY < height)
				result.at<Vec3b>(y, x) = source.at<Vec3b>(abs(fishY), abs(fishX));
			else
				result.at<uchar>(y, x) = 0;*/

			if ((fishX >= 0 && fishX < width) && (fishY >= 0 && fishY < height)) {
				result_front.at<Vec3b>(y, x + 0.25*width) = target_front.at<Vec3b>(fishY, fishX);
				result_back.at<Vec3b>(y, x + 0.25*width) = target_back.at<Vec3b>(fishY, fishX);
			}
			else {
				result_front.at<uchar>(y, x) = 250;
				result_back.at<uchar>(y, x) = 250;
			}
				

			
			/*if (y == 320)
				cout << "x = " << x << "is completed" << endl;*/
		}
		//cout << "y = " << y << " is completed" << endl;		//x 1, y 320부터가 문제더라
	}
	
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			fine_front.at<Vec3b>(y, x) = result_front.at<Vec3b>(y, width / 2 + x / 2);
			fine_back.at<Vec3b>(y, x) = result_back.at<Vec3b>(y, width / 2 + x / 2);
		}
	}

	//imshow("result", result);
	imwrite("fish1_result.jpg", result_front);
	imwrite("fish2_result.jpg", result_back);
	imwrite("fish1_fine.jpg", fine_front);
	imwrite("fish2_fine.jpg", fine_back);
	
	return 0;

}