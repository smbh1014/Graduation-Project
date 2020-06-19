
#ifndef INIT_H
#define INIT_H

#   include <opencv/cv.h>
#   include <opencv/highgui.h>

extern IplImage* pimage;
extern CvCapture* capture;
extern CvVideoWriter* VideoWriter;

int theta_convert_movie(char* filename);
int theta_image(char* filename);
int theta_live(int num, char** str);
int theta_movie(char* filename);
int theta_stream(char* ip_port);

#endif
