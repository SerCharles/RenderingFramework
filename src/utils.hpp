//The utility functions used in the project
#pragma once
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <float.h>
#include <Eigen\Dense>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>  
using namespace std;
using namespace Eigen;
using namespace cv;

//util functions
/*
Load an matrix file
Returns:
	matrix [double matrix], [4 * 4]: [the loaded extrinsic]
*/
Matrix4d LoadMatrix(const char* filename)
{
	Matrix4d matrix;
	ifstream infile;
	infile.open(filename, ios::in);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			double x;
			infile >> x;
			matrix(i, j) = x;
		}
	}
	infile.close();
	return matrix;
}

/*
Switch a string to int
Args:
	str [string]: [the string to be switched]
Returns:
	sum [int]: [the result int]
*/
int StringToInt(string str)
{
	int length = str.size();
	int int_base = 1;
	int sum = 0;
	for (int i = length - 1; i >= 0; i--)
	{
		sum += (str[i] - '0') * int_base;
		int_base *= 10;
	}
	return sum;
}

/*
Use opencv to save the result picture
Args:
	data [array of Vector3d], [H * W]: [the result data]
	save_place [const char*]: [the full saving place]
	width [int]: [the width of the picture]
	height [int]: [the height of the picture]
*/
void SavePicture(Vector3d* data, const char* save_place, int width, int height)
{
	Mat image = Mat::zeros(Size(width, height), CV_8UC3);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				int color = int(data[i * width + j](k) * 256);
				if (color >= 256)
				{
					color = 255;
				}
				else if (color < 0)
				{
					color = 0;
				}
				uchar ucolor = uchar(color);
				image.at<Vec3b>(i, j)[2 - k] = ucolor;
			}
		}
	}
	imwrite(save_place, image);
}
