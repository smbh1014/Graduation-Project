#ifndef GLCORE_H_
#   define GLCORE_H_
#   include <opencv/cv.h>
#   include <opencv/highgui.h>
#   include <GL/freeglut.h>

#   define UPDATE_PRERIOD_MS 10 
extern IplImage * pimage;
extern CvCapture* capture;
extern CvVideoWriter* VideoWriter;

enum playmode {
	JPG_PANO,
	VFILE_PANO,
	VFILE_FEYES,
	USBCAM_FEYES,
};




void MainLoop(enum playmode pm);

#endif
