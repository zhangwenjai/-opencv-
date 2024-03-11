#include "serialport.h"
#include<string>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
#include "Fileapi.h"
#include <windows.h>
#include <tchar.h>
#include <cstdlib>
using namespace std;
using namespace cv;

void GetROI(Mat src, Mat& ROI)
{
	int width = src.cols;
	int height = src.rows;

	//获取车道ROI区域，只对该部分进行处理
	vector<Point>pts;
	Point ptA((width / 10) * 1, height);
	Point ptB((width / 4) * 1, (height / 3) * 1);
	Point ptC((width / 4) * 3, (height / 3) * 1);
	Point ptD((width / 10) * 9, height);
	pts = { ptA ,ptB,ptC,ptD };

	//vector<vector<Point>>
	vector<vector<Point>>ppts;
	ppts.push_back(pts);

	Mat mask = Mat::zeros(src.size(), src.type());
	fillPoly(mask, ppts, Scalar::all(255));

	src.copyTo(ROI, mask);
}
void rs232send(string control, SerialInterface com)
{
	
	//测试发送
	if (com.writeStr(control))
	{
		cout << "send success" << endl;
	}
	else
	{
		cout << "send fail" << endl;
	}

}
void DetectRoadLine(Mat src, Mat& ROI, SerialInterface com)
{
	Mat gray;
	cvtColor(ROI, gray, COLOR_BGR2GRAY);
	imshow("gray", gray);

	Mat thresh;
	threshold(gray, thresh, 150, 255, THRESH_BINARY);
	imshow("thresh", thresh);


	Mat cl;
	Mat element;
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(thresh, cl, MORPH_CLOSE, element);
	imshow("close", cl);


	vector<Point>left_line;
	vector<Point>right_line;

	//左车道线
	for (int i = (cl.cols / 1280) * 351; i >= 0; i--)
	{
		for (int j = 0; j < cl.rows; j++)
		{
			if (cl.at<uchar>(j, i) == 255)
			{
				left_line.push_back(Point(i, j));
			}
		}

	}
	//右车道线
	for (int i = (cl.cols / 20) * 13; i < cl.cols; i++)
	{
		for (int j = 0; j < cl.rows; j++)
		{
			if (cl.at<uchar>(j, i) == 255)
			{
				right_line.push_back(Point(i, j));
			}
		}
	}

	//车道绘制
	if (left_line.size() > 0 && right_line.size() > 0)
	{
		Point B_L = (left_line[left_line.size() - 1]);
		Point T_L = (left_line[0]);
		Point T_R = (right_line[0]);
		Point B_R = (right_line[right_line.size() - 1]);

		circle(src, B_L, 10, Scalar(0, 0, 255), -1);
		circle(src, T_L, 10, Scalar(0, 255, 0), -1);
		circle(src, T_R, 10, Scalar(255, 0, 0), -1);
		circle(src, B_R, 10, Scalar(0, 255, 255), -1);

		line(src, Point(B_L), Point(T_L), Scalar(0, 255, 0), 10);
		line(src, Point(T_R), Point(B_R), Scalar(0, 255, 0), 10);

		vector<Point>pts;
		pts = { B_L ,T_L ,T_R ,B_R };
		vector<vector<Point>>ppts;
		ppts.push_back(pts);
		fillPoly(src, ppts, Scalar(133, 230, 238));

		//以下为补充，计算方向代码/////
		//按照rs232协议进行输出
		
		double midtop_x = (T_L.x + T_R.x) / 2;
		double midbottom_x = (B_L.x + B_R.x) / 2;
		double midtop_y = (T_L.y + T_R.y) / 2;
		double midbottom_y = (B_L.y + B_R.y) / 2;
		//cout << "top:" << midtop_x << "," << midtop_y << ";" << "bootom:" << midbottom_x << "," << midbottom_y << endl;
		double tan_yaw_angle = (midtop_x - midbottom_x) / (midbottom_y - midtop_y);
		//cout << tan_yaw_angle << endl;
		if (tan_yaw_angle > 0)
		{
			//右转
			double yaw_angle = atan(tan_yaw_angle) * 180 / 3.1415926;
			string temp = "turn right:";
			temp.append(to_string(yaw_angle));
			cout << temp << endl;
			rs232send(temp,com);
		}
		else if (tan_yaw_angle == 0)
		{
			//直行
			string temp = "go straight!";
			cout << temp << endl;
			rs232send(temp,com);
		}
		else
		{
			//左转
			double yaw_angle = atan(tan_yaw_angle)*180/3.1415926;
			string temp = "turn left:";
			temp.append(to_string(abs(yaw_angle)));
			cout << temp << endl;
			rs232send(temp,com);
		}
	}
}

int main()
{
	SerialInterface com;
	//波特率14400，数据位8位，无校验位，停止位1位
	if (!com.openSyn("COM3", CBR_115200, NOPARITY, 8, ONESTOPBIT))
	{
		cout << com.getSerialLastError() << endl;

	}
	VideoCapture capture;
	capture.open("3.mp4");

	if (!capture.isOpened())
	{
		cout << "Can not open video file!" << endl;
		system("pause");
		return -1;
	}

	Mat frame, image;
	while (capture.read(frame))
	{
		char key = waitKey(10);
		if (key == 27)
		{
			break;
		}
		GetROI(frame, image);

		DetectRoadLine(frame, image, com);


		imshow("frame", frame);
	}

	capture.release();
	destroyAllWindows();
	com.closeComm();
	system("pause");
	return 0;
}







