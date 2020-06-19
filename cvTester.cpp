#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void main() {

	VideoCapture video(0);
	Mat frame;
	VideoWriter vout;

	Size size = Size((int)video.get(CAP_PROP_FRAME_WIDTH), (int)video.get(CAP_PROP_FRAME_HEIGHT));
	vout.open("output.avi", VideoWriter)
}