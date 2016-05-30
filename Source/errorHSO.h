#ifndef __ERRORHSO_H__
#define __ERRORHSO_H__

#include "cv.h"      
#include "highgui.h" 
#include "cxcore.h" 

/** This is the data structure passed to the Levenberg-Marquardt procedure */
struct data_struct
{
	cv::Mat &LSS;		// Line segments vectors (3xN)	
	cv::Mat &Lengths;	// Length of line segments (NxN) (diagonal)
	cv::Mat &midPoints;	// Mid points (c=(a+b)/2) (3xN)	

	cv::Mat &K;		// Camera calibration matrix (3x3)

	data_struct (cv::Mat &_LSS, cv::Mat &_Lengths, cv::Mat &_midPoints, cv::Mat &_K): 
		LSS(_LSS), Lengths(_Lengths), midPoints(_midPoints), K(_K)
	{
	}
};

float distanceHSO( cv::Mat &vanishingPoint, cv::Mat &lineSegment, float lengthLineSegment, cv::Mat &midPoint );

void evaluateHSO( const double *param, int m_dat, const void *data, double *fvec, int *info);


#endif // __ERRORHSO_H__
