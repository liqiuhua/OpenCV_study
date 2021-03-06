// Main.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <math.h>
#include "./Algorithm/CVAlgorithmInclude.h"
using namespace std;
int main()
{
/*	Mat imgSrc = imread("Picture\\Line.bmp");
	_CVAlgorithm CVAlg;
	Mat imgOut;
	imshow("image source", imgSrc);*///到像素重映射使用此图片
	/******************************************
	以下是掩膜操作测试用例
	*******************************************/
	//imshow("output Mask image",CVAlg.MaskOperation(imgSrc));

	/******************************************
	以下是像素权重合并测试用例
	*******************************************/
	//Mat imgWinlogo = imread("G:\\leo\\VS\\winlogo.jpg");
	//Mat imgLinuxlogo = imread("G:\\leo\\VS\\linuxlogo.jpg");
	//imshow("output addweight image",CVAlg.MergedImages(imgWinlogo,imgLinuxlogo,0.5));

	/******************************************
	以下是霍夫线变化测试用例
	*******************************************/
	//CVAlg.HoughLineTransform(imgSrc, imgOut);
	//imshow("Hough Line output image", imgOut);

	/******************************************
	以下是霍夫圆变化测试用例
	*******************************************/
	//CVAlg.HoughCirclesTransform(imgSrc, imgOut);
	//imshow("Hough Circles output image", imgOut);

	/******************************************
	以下是像素重映射测试用例
	*******************************************/
	//float Scale = 0.3;
	//imgOut.create(imgSrc.rows*Scale, imgSrc.cols*Scale, imgSrc.type());
	//CVAlg.ImgRemapping(imgSrc,imgOut, MAPTYPE_ZOOMOUT, Scale);
	//imshow("Image Remap MAPTYPE_ZOOMOUT", imgOut);

	//Mat imgOut1;
	//imgOut1.create(imgSrc.size(), imgSrc.type());
	//CVAlg.ImgRemapping(imgSrc, imgOut1, MAPTYPE_RLMIRROR);
	//imshow("Image Remap MAPTYPE_RLMIRROR", imgOut1);

	//Mat imgOut2;
	//imgOut2.create(imgSrc.size(), imgSrc.type());
	//CVAlg.ImgRemapping(imgSrc, imgOut2, MAPTYPE_UDMIRROR);
	//imshow("Image Remap MAPTYPE_UDMIRROR", imgOut2);







	/******************************************
	以下是模板匹配测试用例
	*******************************************/
	Mat imgSrc = imread("Picture\\flower1.jpg");
	_CVAlgorithm CVAlg;
	Mat imgOut;
	Mat imgMatch=imread("Picture\\flower1_match.png");
	if (imgSrc.empty() || imgMatch.empty())
	{
		printf("could not image...\n");
			waitKey(0);
			return -1;
	}
	imshow("image source", imgSrc);
	imshow("image match", imgMatch);

	CVAlg.ImgTemplateMatch(imgSrc, imgMatch, imgOut,1);
	if (imgOut.empty() )
	{
		printf("could not out image...\n");
		waitKey(0);
		return -1;
	}
	imshow("Image TempLate matches", imgOut);
	// 等待6000 ms后窗口自动关闭                            
	waitKey(0);
	return 0;

}

