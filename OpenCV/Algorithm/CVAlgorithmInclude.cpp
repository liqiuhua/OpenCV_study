#include "CVAlgorithmInclude.h"

using namespace std;
_CVAlgorithm::_CVAlgorithm(void)
{

}
_CVAlgorithm::~_CVAlgorithm(void)
{

}
uchar _CVAlgorithm::ShowImg(Mat imgSrc)
{
	if (imgSrc.empty())
	{
		printf("Can't load the img");
		return false;
	}
	else
	{
		namedWindow("lena", CV_WINDOW_AUTOSIZE);
		imshow("lena", imgSrc);
		return true;
	}
}
/** @brief Mask operation
@param imgSrc :input the image 
@param return  image
 */
Mat _CVAlgorithm::MaskOperation(Mat imgSrc)
{
	Mat imgOut;
	if (!imgSrc.data)
	{
		cout << "Could not load the image 1" << endl;
		return imgOut;// NULL;
	}
	imgOut=Mat::zeros(imgSrc.size(), imgSrc.type());
	int row, col;
	for ( row=1; row < (imgSrc.rows - 1); ++row)
	{
		const uchar* previous = imgSrc.ptr<uchar>(row - 1);
		const uchar* current = imgSrc.ptr<uchar>(row);
		const uchar* next = imgSrc.ptr<uchar>(row + 1);
		for ( col = imgSrc.channels(); col < (imgSrc.cols * imgSrc.channels()); col++)
		{
			//	imgOut
			imgOut.ptr<uchar>(row)[col] =saturate_cast<uchar>(5 * current[col] - (current[col - imgSrc.channels()] + current[col + imgSrc.channels()] + previous[col] + next[col]))	;
		}
	}
	return imgOut;
}
/** @brief Mask operation
@param imgSrc :input the image 
@param imgOut :output image
 */
void _CVAlgorithm::MaskOperation(Mat imgSrc,Mat imgOut)
{
	if (!imgSrc.data)
	{
		cout << "Could not load the image 1" << endl;
		return;
	}
	Mat kernel = (Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	imgOut = Mat::zeros(imgSrc.size(), imgSrc.type());
	filter2D(imgSrc, imgOut, imgOut.depth(), kernel);
}
/** @brief add weight operation
@param imgSrcA :input the A image 
@param imgSrcB :input the B image 
@param Weight : 
@param return A+B = output image
 */
Mat _CVAlgorithm::MergedImages(Mat imgSrcA, Mat imgSrcB,double Weight =0.5)
{
	Mat imgDst;
	if (!imgSrcA.data)
	{
		cout << "Could not load the image A" << endl;
		return imgDst;
	}
	if (!imgSrcB.data)
	{
		cout << "Could not load the image B" << endl;
		return imgDst;
	}
	imgDst.create(imgSrcA.size(), imgSrcA.type());
	if ((imgSrcA.rows == imgSrcB.rows) && (imgSrcA.cols == imgSrcB.cols) && (imgSrcA.type() == imgSrcB.type()))
	{
		addWeighted(imgSrcA, Weight, imgSrcB, (1.0 - Weight), 0.0, imgDst);
		return imgDst;
	}
	else
	{
		cout << "the two image don't merge" << endl;
	}
}
/** @brief add weight operation
@param imgSrcA :input the A image 
@param imgSrcB :input the B image 
@param Weight : 
@param imgOut: A+B = output image
 */
void _CVAlgorithm::MergedImages(Mat imgSrcA, Mat imgSrcB, Mat imgOut,double Weight =0.5)
{
	if (!imgSrcA.data)
	{
		cout << "Could not load the image A" << endl;
		return ;
	}
	if (!imgSrcB.data)
	{
		cout << "Could not load the image B" << endl;
		return ;
	}
		imgOut.create(imgSrcA.size(), imgSrcA.type());
	if ((imgSrcA.rows == imgSrcB.rows) && (imgSrcA.cols == imgSrcB.cols) && (imgSrcA.type() == imgSrcB.type()))
	{
		addWeighted(imgSrcA, Weight, imgSrcB, (1.0 - Weight), 0.0, imgOut);
	}
	else
	{
		cout << "the two image don't merge" << endl;
	}
}
/** @brief Hough line operation
@param imgSrcA :input imagee
@param imgOut: output image
@ step1:将图像变成8bit灰度图像，进行边缘检测(但还是我觉得使用threshold得到的结果比我想象中的要好，不是很清楚Canndy为什么没有这样的效果)
@ step2:
@ cv::HoughLinesP(
InputArray src, // 输入图像，必须8-bit的灰度图像
OutputArray lines, // 输出的极坐标来表示直线
double rho, // 生成极坐标时候的像素扫描步长
double theta, //生成极坐标时候的角度步长，一般取值CV_PI/180
int threshold, // 阈值，只有获得足够交点的极坐标点才被看成是直线
double minLineLength=0;// 最小直线长度
double maxLineGap=0;// 最大间隔
)

 */
void _CVAlgorithm::HoughLineTransform(Mat imgSrc, Mat imgOut)
{
	if (!imgSrc.data)
	{
		cout << "Could not load the image " << endl;
		return;
	}
	Mat imgGray;
	cvtColor(imgSrc, imgGray, CV_BGR2GRAY);
	threshold(~imgGray, imgGray, 10, 255, THRESH_BINARY);
	//Canny(imgSrc, imgGray,60,200);


	//cvtColor(imgGray, imgOut, CV_GRAY2BGR);

	vector<Vec4f> plines;

	HoughLinesP(imgGray, plines, 1, CV_PI / 180.0,150, 0, 0);

	Scalar color = Scalar(0, 0, 255);
	for (size_t i = 0; i < plines.size(); i++)
	{
		Vec4f hline = plines[i];
		line(imgOut, Point(hline[0], hline[1]), Point(hline[2], hline[3]), color, 3, LINE_AA);
	}

}

/** @brief Hough Circles operation
@param imgSrcA :input  imagee
@param imgOut: output image
@ step1:将图像变成8bit灰度图像，进行边缘检测(但还是我觉得使用threshold得到的结果比我想象中的要好，不是很清楚Canndy为什么没有这样的效果)
@ step2: 因为圆的检测对噪声比较敏感，所以一般要先对图像进行滤波，这里使用中值滤波
@ step3： Opencv中使用的霍夫圆变换检测是基于图像梯度的，检测边缘，发现可能的圆心，计算最佳圆心半径大小
@HoughCircles(
InputArray image, // 输入图像 ,必须是8位的单通道灰度图像
OutputArray circles, // 输出结果，发现的圆信息
Int method, // 方法 - HOUGH_GRADIENT
Double dp, // dp = 1; 
Double mindist, // 10 最短距离-可以分辨是两个圆的，否则认为是同心圆- src_gray.rows/8
Double param1, // canny edge detection low threshold
Double param2, // 中心点累加器阈值 – 候选圆心
Int minradius, // 最小半径
Int maxradius//最大半径 
)
 */
void _CVAlgorithm::HoughCirclesTransform(Mat imgSrc, Mat imgOut)
{
	if (!imgSrc.data)
	{
		cout << "Could not load the image " << endl;
		return;
	}
	Mat imgGray;
	Mat imgMedianBlur;
	medianBlur(imgSrc, imgMedianBlur, 5);//中值滤波
	cvtColor(imgMedianBlur, imgGray, CV_BGR2GRAY);//转换为灰度图像
	imshow("imgGray", imgGray);
	vector<Vec3f> pcircles;


	HoughCircles(imgGray, pcircles, HOUGH_GRADIENT,1,100,50.0,30,5,70 );
	imgSrc.copyTo(imgOut);

	Scalar color = Scalar(0, 255, 0);
	for (size_t i = 0; i < pcircles.size(); i++)
	{
		Vec3f pcircle = pcircles[i];
		circle(imgOut, Point(pcircle[0], pcircle[1]),pcircle[2], color, 2, LINE_AA);
		circle(imgOut, Point(pcircle[0], pcircle[1]), 2, color, 2, LINE_AA);
	}

}

/** @brief  pixel of image remapping
@param imgSrc :input the A image
@param MapType : type of Transform
			type value:  MAPTYPE_ZOOMOUT  :Zoom out
						 MAPTYPE_RLMIRROR :Right and Left the mirror
						 MAPTYPE_UDMIRROR :Up and Down the mirror
@param Scale : scale of Zomm out
@param imgOut: output image after Transform
 */
void _CVAlgorithm::ImgRemapping(Mat imgSrc, Mat imgOut,unsigned char MapType)
{
	Mat Map_X, Map_Y;
	Map_X.create(imgSrc.size(), CV_32FC1);
	Map_Y.create(imgSrc.size(), CV_32FC1);
	for (unsigned int row = 0; row < imgSrc.rows; row++)
	{
		for (unsigned int col = 0; col < imgSrc.cols; col++)
		{
			switch (MapType)
			{
			case MAPTYPE_UDMIRROR:
			{
				Map_X.at<float>(row, col) = col;
				Map_Y.at<float>(row, col) = imgSrc.rows - row - 1;
				break;
			}
			case MAPTYPE_RLMIRROR:
			{
				Map_X.at<float>(row, col) = imgSrc.cols - col - 1;
				Map_Y.at<float>(row, col) = row;
				break;
			}
			default:
				break;
			}
		}
	}
	remap(imgSrc, imgOut, Map_X, Map_Y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 255, 0));
}
void _CVAlgorithm::ImgRemapping(Mat imgSrc, Mat imgOut, unsigned char MapType,double Scale)
{
	Mat Map_X, Map_Y;
	//imgOut.create(imgSrc.rows*Scale,imgSrc.cols*Scale , CV_32FC1);
	Map_X.create(imgOut.size(), CV_32FC1);
	Map_Y.create(imgOut.size(), CV_32FC1);
	
	for (unsigned int row = 0; row < imgOut.rows; row++)
	{
		for (unsigned int col = 0; col < imgOut.cols; col++)
		{
			switch (MapType)
			{
			case MAPTYPE_ZOOMOUT:
				Map_X.at<float>(row, col) = 1.0 / Scale * col;
				Map_Y.at<float>(row, col) = 1.0 / Scale * row;
			//		cout << 1.0 / Scale * col << endl;

			default:
				break;

			}
		}
	}
	remap(imgSrc, imgOut, Map_X, Map_Y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 255, 0));
//	imshow("re", imgOut);
}