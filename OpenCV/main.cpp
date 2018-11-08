// CV_1.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace cv;
int main()
{  
	Mat img = imread("Picture\\Line.bmp");
	if (img.empty())
	{
		printf("Can't load the img");
		waitKey(0);
		return 1;
	}

	namedWindow("line",CV_WINDOW_AUTOSIZE); 
	imshow("line",img);


	// 等待6000 ms后窗口自动关闭                            
	waitKey(0);
	return 0;

}

