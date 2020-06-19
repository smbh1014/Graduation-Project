#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;

int main(void) {

	Mat input;
	ofstream outfile("output.txt");
	int x, y;

	input = imread("pixelsize.jpg");

	cout << (int)input.at<Vec3b>(25, 25)[1] << endl;

	for (y = 0; y < 50; y++) {
		for (x = 0; x < 50; x++) {
			outfile << "img_R[" << y * 50 + x << "] = " << (int)input.at<Vec3b>(y, x)[2] << ";" <<  endl;
		}
	}
	for (y = 0; y < 50; y++) {
		for (x = 0; x < 50; x++) {
			outfile << "img_G[" << y * 50 + x << "] = " << (int)input.at<Vec3b>(y, x)[1] << ";" << endl;
		}
	}
	for (y = 0; y < 50; y++) {
		for (x = 0; x < 50; x++) {
			outfile << "img_B[" << y * 50 + x << "] = " << (int)input.at<Vec3b>(y, x)[0] << ";" << endl;
		}
	}


	imshow("50 image", input);

	char ch = waitKey(30000);

}