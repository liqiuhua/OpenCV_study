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
	*��Ĥ����������ͼƬȨ�غϲ�
	*************************************/
    void MaskOperation(Mat imgSrc,Mat imgOut);
    Mat MaskOperation(Mat imgSrc);
	Mat MergedImages(Mat imgSrcA, Mat imgSrcB, double Weight);
	void MergedImages(Mat imgSrcA, Mat imgSrcB, Mat imgOut, double Weight);
	/************************************
	*����ֱ�߱仯 Hough line transform �� ����԰�任 
	*************************************/
	void HoughLineTransform(Mat imgSrc, Mat imgOut);
	void HoughCirclesTransform(Mat imgSrc, Mat imgOut);
	/************************************
	* ������ӳ��
	**************************************/
	void ImgRemapping(Mat imgSrc, Mat imgOut,unsigned char MapType);
	void ImgRemapping(Mat imgSrc, Mat imgOut, unsigned char MapType, double Scale);
};

#endif