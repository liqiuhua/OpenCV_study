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
	Mat imgSrc = imread("Picture\\Line.bmp");
	_CVAlgorithm CVAlg;
	Mat imgOut;
	imgOut.create(imgSrc.size(), imgSrc.type());
	//imshow("output Mask image",CVAlg.MaskOperation(imgSrc));
	//
	//Mat imgWinlogo = imread("G:\\leo\\VS\\winlogo.jpg");
	//Mat imgLinuxlogo = imread("G:\\leo\\VS\\linuxlogo.jpg");
	//imshow("output addweight image",CVAlg.MergedImages(imgWinlogo,imgLinuxlogo,0.5));

	//CVAlg.HoughLineTransform(imgSrc, imgOut);
	//imshow("Hough Line output image", imgOut);

	//CVAlg.HoughCirclesTransform(imgSrc, imgOut);
	//imshow("Hough Circles output image", imgOut);
	int Key;
	while (true)
	{
		Key = waitKey(500);
		cout << Key << endl;
		if ((char)Key == 27)break;
		CVAlg.ImgRemapping(imgSrc, imgOut, (Key%4), 0.5);
		imshow("Image Remap", imgOut);
	}

	// 等待6000 ms后窗口自动关闭                            
	//waitKey(0);
	return 0;

}

