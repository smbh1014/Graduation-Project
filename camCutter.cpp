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

	VideoCapture capture(1);
	Mat frame = imread("sam.jpg");
	Mat target_front(614, 590, CV_8UC3);
	Mat target_back(614, 590, CV_8UC3);
	int x, y;

	float theta, phi, r;
	float sphX, sphY, sphZ;
	int fishX, fishY;

	float FOV = 3.141592654 * 180 / 180;


	while (1) {
		capture >> frame;

		imshow("original", frame);

		imwrite("original.jpg", frame);

		for (y = 10; y < 624; y++) {
			for (x = 10; x < 600; x++) {
				target_front.at<Vec3b>(y - 10, x - 10) = frame.at<Vec3b>(y + 5, x + 5);
				target_back.at<Vec3b>(y - 10, x - 10) = frame.at<Vec3b>(y + 5, x + 665);
			}
		}

		float width = target_front.cols;
		float height = target_front.rows;

		Mat result_front(height, width, CV_8UC3);
		Mat result_back(height, width, CV_8UC3);
		Mat fine_front(height, width, CV_8UC3);
		Mat fine_back(height, width, CV_8UC3);

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

		imshow("fish1_fine", fine_front);
		imshow("fish2_fine", fine_back);

		/*imshow("cam", frame);
		imshow("front", front);
		imshow("back", back);
		imwrite("sam_orig.jpg", frame);
		imwrite("sam_front.jpg", front);
		imwrite("sam_back.jpg", back);*/
		char key = waitKey(30);
		if (key == ' ')
			break;
	}



	//float FOV = 3.141592654;

	return 0;

}