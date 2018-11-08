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