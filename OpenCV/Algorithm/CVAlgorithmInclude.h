#ifndef __CVALGORITHMINCLUDE_H_
#define __CVALGORITHMINCLUDE_H_

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include "../stdafx.h"
#include <iostream>
using namespace cv;


#define MAPTYPE_ZOOMOUT 1
#define MAPTYPE_RLMIRROR 2
#define MAPTYPE_UDMIRROR 3

class _CVAlgorithm{
private:
     
public:
	_CVAlgorithm();
	~_CVAlgorithm();
	uchar ShowImg(Mat imgSrc);
	/************************************
	*掩膜操作，两张图片权重合并
	*************************************/
    void MaskOperation(Mat imgSrc,Mat imgOut);
    Mat MaskOperation(Mat imgSrc);
	Mat MergedImages(Mat imgSrcA, Mat imgSrcB, double Weight);
	void MergedImages(Mat imgSrcA, Mat imgSrcB, Mat imgOut, double Weight);
	/************************************
	*霍夫直线变化 Hough line transform 和 霍夫园变换 
	*************************************/
	void HoughLineTransform(Mat imgSrc, Mat imgOut);
	void HoughCirclesTransform(Mat imgSrc, Mat imgOut);
	/************************************
	* 像素重映射
	**************************************/
	void ImgRemapping(Mat imgSrc, Mat imgOut,unsigned char MapType);
	void ImgRemapping(Mat imgSrc, Mat imgOut, unsigned char MapType, double Scale);
};

#endif