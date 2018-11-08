#ifndef __CVALGORITHMINCLUDE_H_
#define __CVALGORITHMINCLUDE_H_

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include "../stdafx.h"
#include <iostream>
using namespace cv;

class _CVAlgorithm{
private:
     
public:
	_CVAlgorithm();
	~_CVAlgorithm();
	uchar ShowImg(Mat imgSrc);
    void MaskOperation(Mat imgSrc,Mat imgOut);
    Mat MaskOperation(Mat imgSrc);
	Mat MergedImages(Mat imgSrcA, Mat imgSrcB, double Weight);
	void MergedImages(Mat imgSrcA, Mat imgSrcB, Mat imgOut, double Weight);
};

#endif