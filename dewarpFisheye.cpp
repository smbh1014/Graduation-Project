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

using namespace std;
using namespace cv;
using namespace std;

int main(void) {

	float FOV = 3.141592654;
	Mat source = imread("fish1.jpg");

	float theta, phi, r;
	float sphX, sphY, sphZ;
	int fishX, fishY;
	int xx, yy;

	float width = source.cols;
	float height = source.rows;

	Mat result(height, width, CV_8UC3);

	ofstream outFile("fisheye position.txt");

	for (int y = -1 * (height / 2); y < height / 2; y++) {
		for (int x = -1 * (width / 2); x < width / 2; x++) {
			xx = x + width / 2;
			yy = y + height / 2;

			float distance = sqrt((xx + x) * (xx + x) + (yy + y) * (yy + y));

			if(distance < height/2 + 3 && distance > height/2 - 3)
				result.at<Vec3b>
		}
	}


	
	return 0;

}