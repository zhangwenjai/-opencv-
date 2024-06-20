#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2\imgproc\types_c.h>
#include <iostream>
using namespace std;
using namespace cv;

vector<vector<Point2f> >corners_l_array, corners_r_array;
int array_index = 0;
//�������ͼ����Ұ���Ƿ�ƽ��
bool ChessboardStable(vector<Point2f>corners_l, vector<Point2f>corners_r) {
	if (corners_l_array.size() < 10) {
		corners_l_array.push_back(corners_l);
		corners_r_array.push_back(corners_r);
		return false;
	}
	else {
		corners_l_array[array_index % 10] = corners_l;
		corners_r_array[array_index % 10] = corners_r;
		array_index++;
		double error = 0.0;
		for (int i = 0; i < corners_l_array.size(); i++) {
			for (int j = 0; j < corners_l_array[i].size(); j++) {
				error += abs(corners_l[j].x - corners_l_array[i][j].x) + abs(corners_l[j].y - corners_l_array[i][j].y);
				error += abs(corners_r[j].x - corners_r_array[i][j].x) + abs(corners_r[j].y - corners_r_array[i][j].y);
			}
		}
		if (error < 1000)
		{
			corners_l_array.clear();
			corners_r_array.clear();
			array_index = 0;
			return true;
		}
		else
			return false;
	}
}

int  main()
{
	Mat input_image;
	Mat image_left, image_right,frame_l,frame_r;
	VideoCapture cam;

	cam.open(0, CAP_DSHOW);

	Rect left_rect(0, 0, 319, 240);  //����һ��Rect������cv�е��࣬�ĸ���������x,y,width,height  
	Rect right_rect(320, 0, 319, 240);

	if (!cam.isOpened())
		exit(0);
	cam.set(CAP_PROP_FRAME_WIDTH, 640);
	cam.set(CAP_PROP_FRAME_HEIGHT, 240);

	Size boardSize(8, 5);
	const float squareSize = 29.f;  //��������ͼ�Ĵ�С�;��롣�����õ���11*7�ģ���˻���10*6��ʮ�ֽ���㣬��ÿ��Сɫ����24mm

	vector<vector<Point2f> > imagePoints_l;
	vector<vector<Point2f> > imagePoints_r;

	int nimages = 0;

	while (true)
	{
		cam >> input_image;
		image_left = Mat(input_image, left_rect).clone();
		image_right = Mat(input_image, right_rect).clone();//�����������Ұ


		bool found_l = false, found_r = false;
		vector<Point2f> corners_l, corners_r;

		found_l = findChessboardCorners(image_left, boardSize, corners_l, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		found_r = findChessboardCorners(image_right, boardSize, corners_r, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

		if (found_l && found_r && ChessboardStable(corners_l, corners_r)) 
		{

			Mat viewGray;
			cvtColor(image_left, viewGray, COLOR_BGR2GRAY);
			cornerSubPix(viewGray, corners_l, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			cvtColor(image_right, viewGray, COLOR_BGR2GRAY);
			cornerSubPix(viewGray, corners_r, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

			imagePoints_l.push_back(corners_l);
			imagePoints_r.push_back(corners_r);
			++nimages;
			image_left += 100;
			image_right += 100;

			drawChessboardCorners(image_left, boardSize, corners_l, found_l);
			drawChessboardCorners(image_right, boardSize, corners_r, found_r);

			putText(image_left, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
			putText(image_right, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
			imshow("Left Camera", image_left);
			imshow("Right Camera", image_right);

			char c = (char)waitKey(500);
			if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
				exit(-1);

			if (nimages >= 30)
				break;
		}
		else 
		{
			drawChessboardCorners(image_left, boardSize, corners_l, found_l);
			drawChessboardCorners(image_right, boardSize, corners_r, found_r);

			putText(image_left, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
			putText(image_right, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
			imshow("Left Camera", image_left);
			imshow("Right Camera", image_right);

			char key = waitKey(1);
			if (key == 27 || key == 'q' || key == 'Q') //Allow ESC to quit
				break;
		}
	}
	if (nimages < 20) { cout << "Not enough" << endl; return -1; }

	vector<vector<Point2f> > imagePoints[2] = { imagePoints_l, imagePoints_r };
	vector<vector<Point3f> > objectPoints;
	objectPoints.resize(nimages);

	for (int i = 0; i < nimages; i++)
	{
		for (int j = 0; j < boardSize.height; j++)
			for (int k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}
	cout << "Running stereo calibration ..." << endl;

	Size imageSize(320, 240);  //�޸ķֱ��ʼǵ��޸Ĵ˴�
	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints_l, imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints_r, imageSize, 0);

	Mat R, T, E, F;

	

	double rms = stereoCalibrate(objectPoints, imagePoints_l, imagePoints_r,
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F, //ͼ��ߴ� ��ת���� ƽ�ƾ��� ��֡���� ��������
		CALIB_FIX_ASPECT_RATIO + //��������˸ñ�־λ����ô�ڵ��ñ궨����ʱ���Ż�����ֻͬʱ�ı�fx��fy�����̶�intrinsic_matrix������ֵ����� CV_CALIB_USE_INTRINSIC_GUESSҲû�б����ã���intrinsic_matrix�е�fx��fy����Ϊ�κ�ֵ����������أ�
		CALIB_ZERO_TANGENT_DIST + //�ñ�־�ڱ궨�߼��������ʱ��Ƚ���Ҫ����Ϊ��ȷ���������º�С�ľ�����䡣��ͼ���������0�ᵼ���������ź���ֵ���ȶ���ͨ�����øñ�־���Թر�����������p1��p2����ϣ���������������Ϊ0//��ֵ�����ؽǵ�
		CALIB_USE_INTRINSIC_GUESS + // cvCalibrateCamera2()�����ڲ��������ʱ��ͨ������Ҫ�������Ϣ��������˵������cx��cy��ͼ�����ģ��ĳ�ʼֵ����ֱ�Ӵӱ���image_size�еõ�����(H-1)/2,(W-1)/2)������������˸ñ�����ôinstrinsic_matrix���������ȷ��ֵ������������ʼ�²⣬ΪcvCalibrateCamera2()���Ż�ʱ����
		CALIB_SAME_FOCAL_LENGTH + //�ñ�־λ���Ż���ʱ��ֱ��ʹ��intrinsic_matrix���ݹ�����fx��fy��
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//�̶��������k1,k2,k3����������������ͨ�������Щ��־����Ϊ����ֵ��һ������һ������Ӧ����Ϊ0����ʼʹ������͸����
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));//��ֵ�����ؽǵ�

	cout << "done with RMS error=" << rms << endl;

	double err = 0;
	int npoints = 0;
	//���㼫������
	vector<Vec3f> lines[2]; //����
	for (int i = 0; i < nimages; i++)
	{
		//��ĳͼ���нǵ�����
		int npt = (int)imagePoints_l[i].size();
		Mat imgpt[2];
		imgpt[0] = Mat(imagePoints_l[i]);
		undistortPoints(imgpt[0], imgpt[0], cameraMatrix[0], distCoeffs[0], Mat(), cameraMatrix[0]);
		computeCorrespondEpilines(imgpt[0], 0 + 1, F, lines[0]);

		imgpt[1] = Mat(imagePoints_r[i]); //ĳͼ�Ľǵ���������
		undistortPoints(imgpt[1], imgpt[1], cameraMatrix[1], distCoeffs[1], Mat(), cameraMatrix[1]); //����У����Ľǵ�����
		computeCorrespondEpilines(imgpt[1], 1 + 1, F, lines[1]); //���㼫��

		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";


	Mat R1, R2, P1, P2, Q; //���������
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1], //���ڲ������� //��������
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]); //ͼ��ߴ� ��ת���� ƽ�ƾ��� ����ת�������� ����ת�������� ��ƽ�ƽ������� ��ƽ�ƽ������� ��Ƚ�������

	fs.open("extrinsics.yml", FileStorage::WRITE); //���������
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";


	
	return 0;

}