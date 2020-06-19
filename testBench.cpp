#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


using namespace std;
using namespace cv;

int main(void) {

	VideoCapture capture = VideoCapture(0);
	Mat origin;
	Mat image(648, 648, CV_8UC3);
	Mat bimage(648, 648, CV_8UC3);


	float intrinsic[] = { 412.7629646036739,0,386.1640680456128, 0, 441.9072577214341, 319.3991315133422, 0, 0, 1 };
	float distCoeffes[] = { -61.20964919779682, 3339.353658966986, 0.02892393247871838, 0.003173407409606994, -76266.4902196121 };
	Mat matIntrinsic(Size(3, 3), CV_32FC1, intrinsic);
	Mat matDistCoeffes(Size(5, 1), CV_32FC1, distCoeffes);

	Mat imageUndistorted;
	Mat bimageUndistorted;

	cout << "okay 1" << endl;

	while (1) {
		capture >> origin;
		for (int height = 0; height < 648; height++) {
			for (int width = 0; width < 648; width++) {
				image.at<Vec3b>(height, width) = origin.at<Vec3b>(height, width);
				bimage.at<Vec3b>(height, width) = origin.at<Vec3b>(height, width + 648);
			}
		}

		undistort(image, imageUndistorted, matIntrinsic, matDistCoeffes);
		undistort(bimage, bimageUndistorted, matIntrinsic, matDistCoeffes);

		imshow("win1", image);
		imshow("win2", imageUndistorted);

		int key = waitKey(1);

	}

	capture.release();

	return 0;

}