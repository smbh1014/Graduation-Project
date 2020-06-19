//#define TEST
#ifdef TEST
#include <iostream>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

#ifdef _DEBUG
#pragma comment(lib, "opencv_calib3d2413d.lib")
#pragma comment(lib, "opencv_contrib2413d.lib")
#pragma comment(lib, "opencv_features2d2413d.lib")
#pragma comment(lib, "opencv_flann2413d.lib")
#pragma comment(lib, "opencv_gpu2413d.lib")
#pragma comment(lib, "opencv_legacy2413d.lib")
#pragma comment(lib, "opencv_ml2413d.lib")
#pragma comment(lib, "opencv_nonfree2413d.lib")
#pragma comment(lib, "opencv_core2413d.lib")
#pragma comment(lib, "opencv_imgproc2413d.lib")
#pragma comment(lib, "opencv_highgui2413d.lib")
#pragma comment(lib, "opencv_objdetect2413d.lib")
#else
#pragma comment(lib, "opencv_core2413.lib")
#pragma comment(lib, "opencv_imgproc2413.lib")
#pragma comment(lib, "opencv_highgui2413.lib")
#endif
void main()
{
	VideoCapture stream1(0);

	if (!stream1.isOpened())
	{
		cout << "cannot open cammera";
	}

	namedWindow("Processing");
	namedWindow("Origin");

	while (true)
	{
		Mat cameraFrame;
		stream1.read(cameraFrame);
		imshow("Origin", cameraFrame);

		Sobel(cameraFrame, cameraFrame, CV_8U, 1, 0);
		imshow("Processing", cameraFrame);

		if (waitKey(30) >= 0)
			break;
	}

	destroyAllWindows();


}
#endif

//#define OPENGLTEST
#ifdef OPENGLTEST
#include "gl/glut.h"

void display() {



	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	glClear(GL_COLOR_BUFFER_BIT);

	glColor3f(0.0f, 0.0f, 1.0f);

	glBegin



	(GL_POLYGON);

	glVertex2f(-0.2f, -0.2f);

	glVertex2f(0.4f, -0.4f);

	glVertex2f(0.4f, 0.4f);

	glVertex2f(-0.2f, 0.2f);



	glEnd();

	glFinish();

}

int main(int argc, char** argv)

{

	glutInit(&argc, argv);

	glutCreateWindow

	("OpenGL");

	glutDisplayFunc(display);

	glutMainLoop();

	return 0;

}
#endif

#define FISHEYECONVERT
#ifdef FISHEYECONVERT

#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#define NH_ACCESS_BLUE(IMG,X,Y) ((unsigned char *)((IMG)->imageData + (IMG)->widthStep * (Y)))[(X)*3]
#define NH_ACCESS_GREEN(IMG,X,Y) ((unsigned char *)((IMG)->imageData + (IMG)->widthStep * (Y)))[(X)*3 + 1]
#define NH_ACCESS_RED(IMG,X,Y) ((unsigned char *)((IMG)->imageData + (IMG)->widthStep * (Y)))[(X)*3 + 2]

#ifdef _DEBUG
#pragma comment(lib, "opencv_calib3d2413d.lib")
#pragma comment(lib, "opencv_contrib2413d.lib")
#pragma comment(lib, "opencv_features2d2413d.lib")
#pragma comment(lib, "opencv_flann2413d.lib")
#pragma comment(lib, "opencv_gpu2413d.lib")
#pragma comment(lib, "opencv_legacy2413d.lib")
#pragma comment(lib, "opencv_ml2413d.lib")
#pragma comment(lib, "opencv_nonfree2413d.lib")
#pragma comment(lib, "opencv_core2413d.lib")
#pragma comment(lib, "opencv_imgproc2413d.lib")
#pragma comment(lib, "opencv_highgui2413d.lib")
#pragma comment(lib, "opencv_objdetect2413d.lib")
#else
#pragma comment(lib, "opencv_calib3d2413.lib")
#pragma comment(lib, "opencv_contrib2413.lib")
#pragma comment(lib, "opencv_features2d2413.lib")
#pragma comment(lib, "opencv_flann2413.lib")
#pragma comment(lib, "opencv_gpu2413.lib")
#pragma comment(lib, "opencv_legacy2413.lib")
#pragma comment(lib, "opencv_ml2413.lib")
#pragma comment(lib, "opencv_nonfree2413.lib")
#pragma comment(lib, "opencv_core2413.lib")
#pragma comment(lib, "opencv_imgproc2413.lib")
#pragma comment(lib, "opencv_highgui2413.lib")
#pragma comment(lib, "opencv_objdetect2413.lib")
#endif



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ctype.h>
#include "ts360.h"
#include "init.h"
#include "glcore.h"
#include "fish_translate.h"

extern int resolution_size = 1;
extern IplImage* pimage;


CvMat* mat_u;
CvMat* mat_v;

void fish_translate(int sz);
void panorama(IplImage* in_frame, IplImage* out_frame);

////////////////////////fish_translate.c////////////////////////////////
static void meshgrid(double lin_x[], double lin_y[], CvMat* mat_x,
	CvMat* mat_y)
{
	int row, col;


	for (col = 0; col < mat_x->cols; col++) {
		for (row = 0; row < mat_x->rows; row++) {
			cvmSet(mat_x, row, col, lin_x[col]);
		}
	}

	for (row = 0; row < mat_y->rows; row++) {
		for (col = 0; col < mat_y->cols; col++) {
			cvmSet(mat_y, row, col, lin_y[row]);
		}
	}
}

static void linspace(int x, int y, int z, double tmp[])
{
	int i;
	double ans, n;
	n = y - x;
	ans = n / (z - 1);
	tmp[0] = x;
	tmp[z - 1] = y;
	for (i = 1; i <= z - 2; i++) {
		tmp[i] = tmp[i - 1] + ans;
	}
}


static void matrix_adder(CvMat* mat, double add, CvMat* ans)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(ans, row, col, add + cvmGet(mat, row, col));
		}
	}
}


static void matrix_scalar(CvMat* mat, double add, CvMat* ans)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(ans, row, col, add * cvmGet(mat, row, col));
		}
	}
}

/* cos */
static void matrix_cos(CvMat* mat, CvMat* ans)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(ans, row, col, cos(cvmGet(mat, row, col)));
		}
	}
}

/* sin */
static void matrix_sin(CvMat* mat, CvMat* ans)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(ans, row, col, sin(cvmGet(mat, row, col)));
		}
	}
}

/* atan2 */
static void matrix_atan2(CvMat* mat_y, CvMat* mat_x, CvMat* ans)
{
	int row, col;

	for (row = 0; row < ans->rows; row++) {
		for (col = 0; col < ans->cols; col++) {
			cvmSet(ans, row, col,
				atan2(cvmGet(mat_y, row, col), (cvmGet(mat_x, row, col))));
		}
	}
}


/* acos */
static void matrix_acos(CvMat* mat, CvMat* ans)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(ans, row, col, acos(cvmGet(mat, row, col)));
		}
	}
}

static void matrix_division(CvMat* mat, double div, CvMat* ans)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(ans, row, col, cvmGet(mat, row, col) / div);
		}
	}
}

/* floor */
static void mat_floor(CvMat* mat)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			cvmSet(mat, row, col, cvFloor(cvmGet(mat, row, col)));
		}
	}
}


static void mat_sz(CvMat* mat, int sz)
{
	int row, col;

	for (row = 0; row < mat->rows; row++) {
		for (col = 0; col < mat->cols; col++) {
			if (cvmGet(mat, row, col) > sz) {
				cvmSet(mat, row, col, sz);
			}
		}
	}
}

static void mat_integer(CvMat* mat, CvMat* out_mat, int sz)
{
	matrix_adder(mat, 1, mat);
	matrix_scalar(mat, sz / 2, mat);
	mat_floor(mat);
	matrix_adder(mat, 0, out_mat);
	mat_sz(out_mat, sz - 1);
}



void fish_translate(int sz)
{

	int NX, NY;
	double PID2;
	double xx[848], yy[848];
	double tmp[848];

	NX = sz;
	NY = sz;
	PID2 = M_PI / 2;

	CvMat* cm_x = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* cm_y = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* theta = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* phi = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* p_x = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* p_y = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* p_z = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* m_tmp1 = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* m_tmp2 = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* r = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* uu = cvCreateMat(NX, NY, CV_64FC1);
	CvMat* vv = cvCreateMat(NX, NY, CV_64FC1);

	mat_u = cvCreateMat(NX, NY, CV_64FC1);
	mat_v = cvCreateMat(NX, NY, CV_64FC1);

	linspace(-1, 1, NX, tmp);
	memcpy(xx, tmp, sizeof(double) * NX);
	linspace(-1, 1, NY, tmp);
	memcpy(yy, tmp, sizeof(double) * NY);

	meshgrid(xx, yy, cm_x, cm_y);

	//theta 
	matrix_adder(cm_x, 1, m_tmp1);
	matrix_scalar(m_tmp1, PID2, theta);

	// phi
	matrix_scalar(cm_y, PID2, phi);

	// p_x
	matrix_cos(phi, m_tmp1);
	matrix_cos(theta, m_tmp2);
	cvMul(m_tmp1, m_tmp2, p_x, 1);

	// p_y
	matrix_cos(phi, m_tmp1);
	matrix_sin(theta, m_tmp2);
	cvMul(m_tmp1, m_tmp2, p_y, 1);

	// p_z
	matrix_sin(phi, p_z);

	// theta mk2
	matrix_atan2(p_z, p_x, theta);

	// phi mk2 
	matrix_acos(p_y, phi);

	// r 
	matrix_division(phi, PID2, r);

	// u 
	matrix_scalar(r, -1, m_tmp1);
	matrix_cos(theta, m_tmp2);
	cvMul(m_tmp1, m_tmp2, uu, 1);

	// v 
	matrix_sin(theta, m_tmp1);
	cvMul(r, m_tmp1, vv, 1);

	// floor 
	mat_integer(uu, mat_u, sz);
	mat_integer(vv, mat_v, sz);


	cvReleaseMat(&theta);
	cvReleaseMat(&phi);
	cvReleaseMat(&p_x);
	cvReleaseMat(&p_y);
	cvReleaseMat(&p_z);
	cvReleaseMat(&m_tmp1);
	cvReleaseMat(&m_tmp2);
	cvReleaseMat(&r);
	cvReleaseMat(&uu);
	cvReleaseMat(&vv);
	cvReleaseMat(&cm_x);
	cvReleaseMat(&cm_y);

}

///////////////////////////translate_movie.c//////////////////////////////////
void translate_movie(char* in_filename, char* out_filename)
{

	CvCapture* capture;
	CvVideoWriter* vw;

	IplImage* frame;
	IplImage* pano = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);


	if ((capture = cvCaptureFromFile(in_filename)) == NULL) {
		printf("´’´°´§´Î™¨Ã∏™ƒ™´™Í™ﬁ™ª™Û™«™∑™ø°£\n");
		return;
	}
	fish_translate(SZ * resolution_size);


	vw =
		cvCreateVideoWriter(out_filename, CV_FOURCC('X', 'V', 'I', 'D'), 30,
			cvSize(WIDTH, HEIGHT), 1);
	if (vw == NULL) {
		printf("´’´°´§´Î™ÚıÛ’Ù™«™≠™ﬁ™ª™Û\n");
		return;
	}


#ifdef DEBUG_CONVERT

	cvNamedWindow("CaptureFromFile", CV_WINDOW_AUTOSIZE);
#endif

	while (1) {

		frame = cvQueryFrame(capture);

		if (frame == NULL) {
			break;
		}


		panorama(frame, pano);

		cvWriteFrame(vw, pano);
#ifdef DEBUG_CONVERT

		cvShowImage("CaptureFromFile", pano);


		if (cvWaitKey(2) == '\x1b') {
			break;
		}
#endif

	}



	cvReleaseImage(&pano);
	cvReleaseCapture(&capture);
#ifdef DEBUG_CONVERT
	cvDestroyWindow("CaptureFromFile");
#endif
	cvReleaseVideoWriter(&vw);

}
///////////////////////// panorama.c////////////////////////////////////////

static void pix(int m, int n, int* fish_u, int* fish_v)
{
	*fish_u = cvmGet(mat_u, m, n);
	*fish_v = cvmGet(mat_v, m, n);
}



static void panoLR(IplImage* L_img, IplImage* R_img, IplImage* out_img)
{
	int sz, n, m, pano_sz, u, v;
	sz = L_img->width;
	pano_sz = out_img->width;


	for (m = 0; m < sz; m++) {
		for (n = 0; n < sz; n++) {
			pix(n, m, &u, &v);
			NH_ACCESS_RED(out_img, m, n) = NH_ACCESS_RED(L_img, u, v);
			NH_ACCESS_GREEN(out_img, m, n) = NH_ACCESS_GREEN(L_img, u, v);
			NH_ACCESS_BLUE(out_img, m, n) = NH_ACCESS_BLUE(L_img, u, v);

		}
	}

	for (m = 0; m < sz; m++) {
		for (n = 0; n < sz; n++) {
			pix(n, m, &u, &v);
			NH_ACCESS_RED(out_img, m + sz, n) =
				NH_ACCESS_RED(R_img, u, v);
			NH_ACCESS_GREEN(out_img, m + sz, n) =
				NH_ACCESS_GREEN(R_img, u, v);
			NH_ACCESS_BLUE(out_img, m + sz, n) =
				NH_ACCESS_BLUE(R_img, u, v);

		}
	}
}


static void LRimg_resize(IplImage* LR_img, IplImage* L_img, IplImage* R_img)
{

	int sz = R_img->width;
	int k = CUT * resolution_size;
	int m, n, s;
	/*L*/ for (m = 0; m < sz; m++) {
		for (n = 0; n < sz; n++) {
			NH_ACCESS_RED(L_img, n, m) = NH_ACCESS_RED(LR_img, m + k, n + k);
			NH_ACCESS_GREEN(L_img, n, m) =
				NH_ACCESS_GREEN(LR_img, m + k, n + k);
			NH_ACCESS_BLUE(L_img, n, m) = NH_ACCESS_BLUE(LR_img, m + k, n + k);

		}
	}

	cvFlip(L_img, NULL, 1);
	/*R*/ s = (k * 3) + sz;
	for (m = 0; m < sz; m++) {
		for (n = 0; n < sz; n++) {
			NH_ACCESS_RED(R_img, n, m) = NH_ACCESS_RED(LR_img, m + s, n + k);
			NH_ACCESS_GREEN(R_img, n, m) =
				NH_ACCESS_GREEN(LR_img, m + s, n + k);
			NH_ACCESS_BLUE(R_img, n, m) = NH_ACCESS_BLUE(LR_img, m + s, n + k);

		}
	}
	cvFlip(R_img, NULL, 0);

}

void panorama(IplImage* in_frame, IplImage* out_frame)
{
	int size = resolution_size * SZ;
	IplImage* fish_L_img = cvCreateImage(cvSize(size, size), IPL_DEPTH_8U, 3);
	IplImage* fish_R_img = cvCreateImage(cvSize(size, size), IPL_DEPTH_8U, 3);


	LRimg_resize(in_frame, fish_L_img, fish_R_img);
	panoLR(fish_L_img, fish_R_img, out_frame);
	cvReleaseImage(&fish_L_img);
	cvReleaseImage(&fish_R_img);
}
#define GCORE
#ifdef GCORE
//////////////////////////g.core.c///////////////////////////////
static int ww, hh;

static float pan, tilt;				// ¸ﬁ?’·
static float zoom = 80;

static GLuint mFontTextureId;
static GLUquadric* sphere;


IplImage* pimage; // ?™ﬂ™¿™∑™ø´—´Œ´È´ﬁ?ﬂ¿
CvCapture* capture;
CvVideoWriter* VideoWriter;



/* ‘—Ì¬´‚?´… */
enum playmode pmode;

static int grid_flag;
static int rec_flag;
static int disp_flag_v;
static int disp_flag_p;
static int flip_flag;

static int mouse_x, mouse_y;
static void gridline(int grid_space_x, int grid_space_y, IplImage* source_img);
static void saveImage(int imageWidth, int imageHeight);

static void Keyboard(unsigned char key, int x, int y);
static void LoadTexture(int val);

// ´≥?´Î´–´√´Ø
static void mouse_func(int button, int state, int x, int y);
static void motion_func(int x, int y);
static void skey_func(int skey, int x, int y);

// Ÿ⁄?´Î?´¡´Û
static void drawScene();

static void handleResize(int w, int h);
static void time_char(char*);

static void Rendering(IplImage* img);

// ´ﬁ´¶´π™Ú‰„™∑™ø„¡™Œ´≥?´Î´–´√´Ø
static void mouse_func(int button, int state, int x, int y)
{
	mouse_x = x;
	mouse_y = y;

	switch (button) {	// ´€´§?´Î
	case 2:
		zoom = 80;
		break;
	case 3:
		zoom--;
		break;
	case 4:
		zoom++;
		break;
	default:
		break;
	}
	glutPostRedisplay();	// Ÿ⁄?´≥?´Î´–´√´Ø™Ú˚º™”ıÛ™π  
}

// ´ﬁ´¶´π´…´È´√´∞™∑™ø„¡™Œ´≥?´Î´–´√´Ø
static void motion_func(int x, int y)
{
	pan += x - mouse_x;
	tilt += y - mouse_y;
	mouse_x = x;
	mouse_y = y;
	glutPostRedisplay();	// Ÿ⁄?´≥?´Î´–´√´Ø™Ú˚º™”ıÛ™π  
}

// ˜Â‚®´≠?√Ì¬„¡™Œ´≥?´Î´–´√´Ø
static void skey_func(int skey, int x, int y)
{
	switch (skey) {
	case GLUT_KEY_LEFT:
		pan -= 1;
		break;
	case GLUT_KEY_RIGHT:
		pan += 1;
		break;
	case GLUT_KEY_UP:
		tilt -= 1;
		break;
	case GLUT_KEY_DOWN:
		tilt += 1;
		break;
	case GLUT_KEY_PAGE_UP:
		zoom--;
		break;
	case GLUT_KEY_PAGE_DOWN:
		zoom++;
		break;
	}
	glutPostRedisplay();	// Ÿ⁄?´≥?´Î´–´√´Ø™Ú˚º™”ıÛ™π
}

// Ÿ⁄?´Î?´¡´Û
static void drawScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// ´Ø´Í´¢

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(zoom, (float)ww / (float)hh, 0.5, 100.0);

	glMatrixMode(GL_MODELVIEW);	// Ï§ÀΩ™Œ?◊‚™œ´‚´«´Î™Ú?ﬂ⁄™»™π™Î
								//////////////////////////////////////////////////////////////////////////
	glLoadIdentity();	// ?Í»˙º÷™
	gluLookAt(0, 0, 0, 0, 1, 0, 0.0, 0.0, 1.0);	// ´´´·´È£®Õ≥Ô“£©

												///////////////////////////////////////////////////////////////////////////
												// ´¡´Î´»™∑™∆™´™È´—´Û™∑™ ™§™»™¶™ﬁ™Øﬁ÷™√™ø™Ë™¶™À‘—™´™ ™§???™ ™º£ø
	glRotatef(tilt, 1, 0, 0);	// (¸ﬁ? «£¨¸ﬁ?ıÓ)
	glRotatef(pan, 0, 0, 1);	// (¸ﬁ? «£¨¸ﬁ?ıÓ)

								// œπ
	glBegin(GL_QUAD_STRIP);	// STRIP™Ú™ƒ™±™ ™§™»Ú¢œπ™Œ™Ω™≥™À˙Î™¨™¢™Ø
	glColor3f(1, 1, 1);
	gluSphere(sphere, 2, 30, 30);	// xxxx, ⁄‚?£¨ ?”¯€∞˙æ›¬˘‹?£¨Í’”¯€∞˙æ›¬˘‹?
	glEnd();

	glutSwapBuffers();	// ?∑™ŒŸ⁄?(™≥™≥™«ﬂæ‚˚™ŒÊ—ﬂ©™ÚÔ´™·™∆?„ø™π™Î)
}

static void handleResize(int w, int h)
{
	ww = w;
	hh = h;
	glViewport(0, 0, ww, hh);
}

static void saveImage(int imageWidth, int imageHeight)	//´π´Ø´Í?´Û´∑´Á´√´»
{
	IplImage* buf =
		cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
	IplImage* outImage =
		cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);


	char outputimage[256];

	time_char(outputimage);

	strcat(outputimage, ".jpg");

	printf("Snapshot:%s\n", outputimage);

	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, imageWidth, imageHeight, GL_RGB, GL_UNSIGNED_BYTE,
		buf->imageData);
	glFlush();
	cvConvertImage(buf, outImage, CV_CVTIMG_FLIP + CV_CVTIMG_SWAP_RB);


	cvSaveImage(outputimage, outImage, 0);
	cvReleaseImage(&buf);
	cvReleaseImage(&outImage);

}

static void time_char(char* name)
{
	time_t timer;
	struct tm* timeptr;
	timer = time(NULL);
	timeptr = localtime(&timer);
	strftime(name, 256, "%Y%m%d%H%M%S", timeptr);

}

static void gridline(int grid_space_x, int grid_space_y, IplImage* img)
{

	int n = 0;

	// ?‡ 
	for (n = (img->height / 2) - (grid_space_y / 2); n < img->height; n += grid_space_y) {
		cvLine(img, cvPoint(0, n), cvPoint(img->width, n), cvScalar(0, 204, 0, 0), GRID_LINE_WIDTH, CV_AA, 0);
	}
	for (n = (img->height / 2) + (grid_space_y / 2); n > 0; n -= grid_space_y) {
		cvLine(img, cvPoint(0, n), cvPoint(img->width, n), cvScalar(0, 204, 0, 0), GRID_LINE_WIDTH, CV_AA, 0);
	}

	cvLine(img, cvPoint(0, img->height / 2), cvPoint(img->width, img->height / 2), cvScalar(204, 0, 0, 0), GRID_LINE_WIDTH * 2, CV_AA, 0); // ÓÂ‘≥

																																		   // Í’‡ 
	for (n = 0; n < img->width; n += grid_space_x) {
		cvLine(img, cvPoint(n, 0), cvPoint(n, img->height), cvScalar(0, 204, 0, 0), GRID_LINE_WIDTH, CV_AA, 0);
	}


}

static void grid_reset(IplImage* img, int g_flg)//JPG™À´∞´Í´√´…‡ ™ÚÏ⁄™Ø??
{
	IplImage* img_tmp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);//´∞´Í´√´…Èƒ?ﬂ¿´«?´ø
	if (g_flg == 1) {
		cvCopy(img, img_tmp, NULL);
		gridline(GRID_X, GRID_Y, img_tmp);	//´∞´Í´√´…‡ ™ÚÏ⁄™Ø
	}
	else {
		cvCopy(img, img_tmp, NULL);//Í™™Œ?ﬂ¿™ÀÛ¨™∑Ù™®™Î
	}
	Rendering(img_tmp);
	cvReleaseImage(&img_tmp);
}

/* œπ™À„Ê™∑™§´∆´Ø´π´¡´„™ÚÙ‰™Î */
static void Rendering(IplImage* img)
{


	if (grid_flag == 1 && pmode != JPG_PANO) {
		gridline(GRID_X, GRID_Y, img);	//´∞´Í´√´…‡ ™ÚÏ⁄™Ø
	}

	if (flip_flag == 1) {
		cvFlip(img, 0, NULL);
	}


	//glActiveTexture(GL_TEXTURE);

	glDeleteTextures(1, &mFontTextureId);	// ÕØ™§´∆´Ø´π´¡´„™Ú·º™π
	glGenTextures(1, &mFontTextureId);

	glBindTexture(GL_TEXTURE_2D, mFontTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// GL_NEAREST
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);	// GL_NEAREST
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img->width, img->height, 0, GL_RGB,
		GL_UNSIGNED_BYTE, img->imageData);

	// œπ™ŒﬂÊ‡˜
	if (sphere != NULL) {	// ÕØ™§´™´÷´∏´ß´Ø´»™Ú·º™π       
		gluDeleteQuadric(sphere);
	}
	sphere = gluNewQuadric();	// Õÿÿ¸´™´÷´∏´ß´Ø´»™ŒﬂÊ‡˜

	gluQuadricTexture(sphere, GL_TRUE);	// ´∆´Ø´π´¡´„Ò®¯ˆ™ÚﬂÊ‡˜™ÚÍÛ?˚˘£®ÒÏÈ©£©

	glutPostRedisplay(); // Ó¢¯˙„∆
}


static int num;
static char** str;

void MainLoop(enum playmode pm)
{
	// ?¸µ´ﬁ´√´◊™ŒﬂÊ‡˜
	fish_translate(SZ * resolution_size);

	pmode = pm;

	glutInit(&num, str);	//´¿´ﬂ?
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(DISP_WIDTH, DISP_HEIGHT);
	glutCreateWindow("PANORAMTIC IMAGE VIEWER (TAKAGO LAB.)");
	glClearColor(0.3, 0.3, 0.3, 1.0);	/* €ŒÃÿﬂ‰ */

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_TEXTURE_2D);	// 2D´∆´Ø´π´¡´„ÍÛ?˚˘
	LoadTexture(0);	// ´∆´Ø´π´¡´„™ÚÙ‰™Î

	switch (pmode) { // ?Ú≠?Ï§Ë‚™Œ™»™≠™œÔ“—¢Ó‹™À?ﬂ¿™ÚÛ¨™∑Ù™®™Î
	case JPG_PANO:
		break;
	default:
		glutTimerFunc(UPDATE_PRERIOD_MS, LoadTexture, 0); //?ﬂ¿™ŒÛ¨™∑Ù™®
		break;
	}

	glutDisplayFunc(drawScene);
	glutReshapeFunc(handleResize);


	// ´´´Í´Û´∞™«Õ‘·‹˚˘
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);	// ¯˙ÿ¸∂À€(œπ??™´™ÈÃ∏™Î™Œ™«)
	glutKeyboardFunc(Keyboard); // ´≠?™¨‰„™µ™Ï™ø™È
	glutSpecialFunc(skey_func);	// ˜Â‚®´≠?™¨‰„™µ™Ï™ø™È
	glutMouseFunc(mouse_func);	// ´ﬁ´¶´π™¨‰„™µ™Ï™ø™È
	glutMotionFunc(motion_func);	// ´ﬁ´¶´π´…´È´√´∞™Ú?Ú±™∑™ø™È

									// GL´Î?´◊
	glutMainLoop();

	// ?¸µ´ﬁ´√´◊™Œﬁ˚∂
	cvReleaseMat(&mat_u);
	cvReleaseMat(&mat_v);
}

static void Keyboard(unsigned char key, int x, int y)
{
	char outputVideo[256];


	switch (key) {
	case 'q': //Q´≠?:˚÷ı
		exit(1);
	case 'g': //G´≠?:´∞´Í´√´…‡ ON/OFF
		grid_flag = 1 - grid_flag;
		if (grid_flag)
			printf("´∞´Í´√´…¯˙„∆ ON\n");
		else
			printf("´∞´Í´√´…¯˙„∆ OFF\n");
		switch (pmode) {
		case JPG_PANO:
			grid_reset(pimage, grid_flag);
			break;
		default:
			break;
		}
		break;
	case 's': //S´≠?:´π´Ø´Í?´Û´∑´Á´√´»
		saveImage(DISP_WIDTH, DISP_HEIGHT);
		break;
	case 'r': //R´≠?:??ON/OFF
		rec_flag = 1 - rec_flag;
		switch (pmode) {
		case USBCAM_FEYES:
		case VFILE_FEYES:
		case VFILE_PANO:
			if (rec_flag) {
				time_char(outputVideo);
				strcat(outputVideo, ".avi");
				VideoWriter =
					cvCreateVideoWriter(outputVideo, CV_FOURCC('X', 'V', 'I', 'D'),
						FPS, cvSize(WIDTH * resolution_size, HEIGHT * resolution_size), 1);
				printf("REC start:%s\n", outputVideo);
			}
			else {
				printf("REC stop\n");
				cvReleaseVideoWriter(&VideoWriter);
			}
			break;
		}
		break;
	case 'v': //V´≠?:´—´Œ´È´ﬁ‘—?¯˙„∆ON/OFF
		disp_flag_v = 1 - disp_flag_v;
		switch (pmode) {
		case USBCAM_FEYES:
		case VFILE_FEYES:
		case VFILE_PANO:
			if (disp_flag_v) {
				cvNamedWindow("Stereo_Fisheye", CV_WINDOW_NORMAL);
				cvMoveWindow("Stereo_Fisheye", 0, 0);
				cvResizeWindow("Stereo_Fisheye", DISP_WIDTH, DISP_HEIGHT);
			}
			else {
				cvDestroyWindow("Stereo_Fisheye");
			}
			break;
		}
		break;
	case 'p'://P´≠?:´«´Â´¢´ÎÂ‡‰—‘—?¯˙„∆ON/OFF
		disp_flag_p = 1 - disp_flag_p;
		switch (pmode) {
		case USBCAM_FEYES:
		case VFILE_FEYES:
		case VFILE_PANO:
			if (disp_flag_p) {
				cvNamedWindow("Panorama", CV_WINDOW_NORMAL);
				cvMoveWindow("Panorama", 0, 720);
				cvResizeWindow("Panorama", DISP_WIDTH, DISP_HEIGHT);
			}
			else {
				cvDestroyWindow("Panorama");
			}
			break;
		default:
			break;
		}
	case 'f':
		switch (pmode) {
		case USBCAM_FEYES:
		case VFILE_FEYES:
		case VFILE_PANO:
			if (flip_flag) {
				flip_flag = 1 - flip_flag;
			}
			break;
		default:
			break;
		}
	}
}



/* œπ?™ÀÙ‰™Í‹ı™±™Î™ø™·™Œ´∆´Ø´π´¡´„™Œ´Ì?´… */
static void LoadTexture(int val)
{
	IplImage* frame = NULL; // Â‡‰—
	IplImage* pano = NULL; // ´—´Œ´È´ﬁ

	switch (pmode)
	{
	case JPG_PANO:
		cvCvtColor(pimage, pimage, CV_BGR2RGB);
		cvFlip(pimage, NULL, -1);
		//cvFlip(pimage, NULL, 0);
		//cvFlip(pimage, NULL, 1);
		Rendering(pimage); // ´∆´Ø´π´¡´„™ŒÛ¨™∑Ù™®
		break;
	case VFILE_PANO:
		pimage = cvQueryFrame(capture);
		cvCvtColor(pimage, pimage, CV_BGR2RGB);
		cvFlip(pimage, NULL, 0);
		cvFlip(pimage, NULL, 1);
		Rendering(pimage); // ´∆´Ø´π´¡´„™ŒÛ¨™∑Ù™®
		glutTimerFunc(10, LoadTexture, 0);  // ´ø´§´ﬁ?™ÚÓ¢‘Ù?
		break;
	case VFILE_FEYES:
	case USBCAM_FEYES:
		//‘—?™Œ?ﬂ¿´«?´ø;
		frame = cvCreateImage(cvSize(BEFORE_WIDTH * resolution_size, BEFORE_HEIGHT * resolution_size), IPL_DEPTH_8U, 3);

		//´—´Œ´È´ﬁ?ﬂ¿	
		pano = cvCreateImage(cvSize(WIDTH * resolution_size, HEIGHT * resolution_size), IPL_DEPTH_8U, 3);

		cvResize(cvQueryFrame(capture), frame, CV_INTER_CUBIC);	//´≠´„´◊´¡´„™∑™øÂ‡‰—?ﬂ¿™Ú´Í´µ´§´∫		
		panorama(frame, pano); // Â‡‰—™Ú´—´Œ´È´ﬁ?ﬂ¿™À?¸µ

		if (rec_flag == 1)
			cvWriteFrame(VideoWriter, pano);

		if (disp_flag_v == 1) {
			cvShowImage("Stereo_Fisheye", frame);
			cvWaitKey(1); // Ÿ⁄??˙º			
		}
		if (disp_flag_p == 1) {
			cvShowImage("Panorama", pano);
			cvWaitKey(1); // Ÿ⁄??˙º			
		}

		cvCvtColor(pano, pano, CV_BGR2RGB);
		cvFlip(pano, NULL, 0);
		cvFlip(pano, NULL, 1);
		Rendering(pano); // ´∆´Ø´π´¡´„™ŒÛ¨™∑Ù™®
		cvReleaseImage(&frame);
		cvReleaseImage(&pano);
		glutTimerFunc(UPDATE_PRERIOD_MS, LoadTexture, 0);  // ´ø´§´ﬁ?™ÚÓ¢‘Ù?
		break;
	}
}

////////////////////////////////////////////////////////
#endif

//////////////////////////////////////////////////////
int theta_movie(char* filename)
{

	printf("%s\n", filename);
	//‘—??™ﬂ?™ﬂ
	if ((capture = cvCaptureFromFile(filename)) == NULL) {
		//´’´°´§´Î™¨Ã∏™ƒ™´™È™ ™§„¡
		printf("´’´°´§´Î™¨Ã∏™ƒ™´™Í™ﬁ™ª™Û™«™∑™ø°£\n");
		exit(1);
	}
	MainLoop(VFILE_PANO);

	cvReleaseCapture(&capture);
	cvDestroyWindow("CaptureFromFile");

}

int theta_live(int num, char** str)
{

	//´´´·´È?™ﬂ?™ﬂ
	if ((capture = cvCreateCameraCapture(num)) == NULL) {
		printf("´´´·´È™¨Ã∏™ƒ™´™Í™ﬁ™ª™Û™«™∑™ø\n");
		exit(1);
	}

	MainLoop(USBCAM_FEYES);

	cvReleaseCapture(&capture);
	cvDestroyWindow("CaptureFromFile");

}
//////////////////////////////////////////////////////
int main()
{
	/*
	const char* filename = "fisheyeimg.jpg";
	pimage = cvLoadImage(filename, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	if (pimage == NULL) {
	printf("error!\n");
	exit(1);
	}

	cvNamedWindow("Origin", 0);
	cvResizeWindow("Origin", 1280, 720);
	cvShowImage("Origin", pimage);
	if (cvWaitKey(100) >= 0)
	{
	cvDestroyWindow("Origin");
	return -1;
	}

	MainLoop(JPG_PANO);

	cvReleaseImage(&pimage);
	*/


	char *in_filename = "dualfishmoive.avi";
	const char* out_filename = "convertmoive.avi";
	//ranslate_movie(in_filename, out_filename);
	theta_live(1, NULL);
}
#endif

