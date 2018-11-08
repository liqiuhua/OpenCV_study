// Main.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <math.h>
#include "./Algorithm/CVAlgorithmInclude.h"
int main()
{  
	Mat imgSrc = imread("Picture\\Line.bmp");
	_CVAlgorithm CVAlg;
	imshow("output Mask image",CVAlg.MaskOperation(imgSrc));
	
	Mat imgWinlogo = imread("G:\\leo\\VS\\winlogo.jpg");
	Mat imgLinuxlogo = imread("G:\\leo\\VS\\linuxlogo.jpg");
	imshow("output addweight image",CVAlg.MergedImages(imgWinlogo,imgLinuxlogo,0.5));
	// 等待6000 ms后窗口自动关闭                            
	waitKey(0);
	return 0;

}

