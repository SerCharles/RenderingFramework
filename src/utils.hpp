//The utility functions used in the project
#pragma once
#include <string>
#include <map>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <numbers>
#include <assert.h>
#include <Eigen\Dense>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>  
#include <time.h>
using namespace std;
using namespace Eigen;
using namespace cv;

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
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			for (int k = 0; k < 3; k++)
			{
				int color = int(data[j * width + i](k) * 256);
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

/*
Use the Win32 API to show the picture
Args:
	data [array of Vector3d], [H * W]: [the result data]
	width [int]: [the width of the picture]
	height [int]: [the height of the picture]
	hdc [HDC]: [the HWND used in painting]
*/
void ShowPicture(Vector3d* data, int width, int height, HDC& hdc)
{
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			Vector3d color = data[j * width + i];
			int r = int(color(0) * 256);
			if (r > 255)
			{
				r = 255;
			}
			if (r < 0)
			{
				r = 0;
			}

			int g = int(color(1) * 256);
			if (g > 255)
			{
				g = 255;
			}
			if (g < 0)
			{
				g = 0;
			}

			int b = int(color(2) * 256);
			if (b > 255)
			{
				b = 255;
			}
			if (b < 0)
			{
				b = 0;
			}

			COLORREF rgb = RGB(r, g, b);
			SetPixel(hdc, i, j, rgb);
		}
	}
}