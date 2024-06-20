
#include "nimservosdk.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

#ifdef _WIN32
#include <Windows.h>
#define MySleepMS(msecs) Sleep(msecs)
#else
#include <unistd.h>
#include <string.h>
#define MySleepMS(msecs) usleep(msecs*1000)
#endif

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    printf("********************begin init motor*********************\n");
    if (argc < 3)
    {
        printf("Usage: NimServoSDK_Test2 commType commParam [unitFactor]\r\n");
        return 0;
    }
    Nim_setLogFlags(1);
    int nVal = Nim_getLogFlags();
    printf("exec Nim_getLogFlags :%d\n", nVal);
    if (0 != Nim_init("/home/zwj/bin"))
    {
        printf("exec Nim_init faild\n");
        //return -1;
    }
    unsigned int hMaster;
    if (0 != Nim_create_master(atoi(argv[1]), &hMaster))
    {
        printf("exec Nim_create_master faild\n");
        goto EXIT_APP1;
    }
    if (atoi(argv[1]) < 0 || atoi(argv[1]) > 2)
        goto EXIT_APP2;

    char conn_str[200] = {0};
    
    
        
    sprintf(conn_str, "{\"SerialPort\": \"%s\", \"Baudrate\": 115200, \"Parity\": \"N\", \"DataBits\": 8, \"StopBits\": 1,"\
                          " \"PDOIntervalMS\": 20, \"SyncIntervalMS\": 0}", argv[2]);

    printf("%s\r\n", conn_str);

    if (0 != Nim_master_run(hMaster, conn_str))
    {
        printf("exec Nim_master_run faild\n");
        goto EXIT_APP2;
    }
    Nim_master_changeToPreOP(hMaster);
    MySleepMS(50);              // 必要延时(Nim_master_changeToPreOP后)

    int nAddrs[10] = {0};
    int i = 1, nSlaveCount = 0;
    Nim_scan_nodes(hMaster, 1, 10);
    for (i=1; i<10; i++)
    {
        if (0 != Nim_is_online(hMaster, i))
        {
            nAddrs[nSlaveCount] = i;
            printf("motor %d is online\n", i);
            nSlaveCount++;
        }
    }
    if (nSlaveCount == 0)
    {
        printf("There is no motor online\n");
        goto EXIT_APP3;
    }

    double fUnitFactor = 10000.0;
    if (argc > 3)
    {
        fUnitFactor = atof(argv[3]);
    }
    printf("UnitFactor = %f\r\n", fUnitFactor);

    int nRe = 0;
    for (int nIndex = 0; nIndex < nSlaveCount; nIndex++)
    {
        if (atoi(argv[1]) == 2)
            nRe = Nim_load_params(hMaster, nAddrs[nIndex], "Modbus.db");
        else if (atoi(argv[1]) == 1)
            nRe = Nim_load_params(hMaster, nAddrs[nIndex], "EtherCAT.db");
        else
            nRe = Nim_load_params(hMaster, nAddrs[nIndex], "CANopen.db");

        if (0 != nRe)
        {
            printf("exec Nim_load_params failed\r\n");
            goto EXIT_APP3;
        }

        nRe = Nim_read_PDOConfig(hMaster, nAddrs[nIndex]);
    }
    nRe = Nim_master_changeToOP(hMaster);
    MySleepMS(200);          // 必要延时(Nim_master_changeToOP后)
    for (int nIndex = 0; nIndex < nSlaveCount; nIndex++)
    {
        nRe = Nim_set_unitsFactor(hMaster, nAddrs[nIndex], fUnitFactor);
        nRe = Nim_clearError(hMaster, nAddrs[nIndex], 1);
        nRe = Nim_power_off(hMaster, nAddrs[nIndex], 1);
        MySleepMS(50);      // 必要延时(Nim_power_off后)
        nRe = Nim_set_workMode(hMaster, nAddrs[nIndex], SERVO_VM_MODE, 1);
        MySleepMS(50);      // 必要延时(Nim_set_workMode后)
        nRe = Nim_set_vmTargetSpeed(hMaster, nAddrs[nIndex], 0, 1);
        nRe = Nim_set_vmSpeedLimit(hMaster, nAddrs[nIndex], 0, 500); 
        nRe = Nim_set_vmAccel(hMaster, nAddrs[nIndex], 200, 1);
        nRe = Nim_set_vmDecel(hMaster, nAddrs[nIndex], 200, 1);
        nRe = Nim_power_on(hMaster, nAddrs[nIndex], 1);
    }
    MySleepMS(200);          // 必要延时(Nim_power_on后)
    // 控制字发0x7F控制电机在VM模式下运行
    nRe = Nim_set_controlWord(hMaster, nAddrs[0], 0x7F, 1);
    
    printf("********************finish motor,init camera*********************\n");
    vector<vector<Point2f> >corners_l_array, corners_r_array;
    int array_index = 0;
    Mat input_image;
	Mat image_left, image_right;
	Mat frame_l, frame_r;
	VideoCapture cam(0);
	//cam.open(0, CAP_DSHOW);

	Rect left_rect(0, 0, 319, 240);  //创建一个Rect框，属于cv中的类，四个参数代表x,y,width,height  
	Rect right_rect(320, 0, 319, 240);

	if (!cam.isOpened())
		exit(0);
	cam.set(CAP_PROP_FRAME_WIDTH, 640);
	cam.set(CAP_PROP_FRAME_HEIGHT, 240);

	//读取摄像头校准文件
	Mat cameraMatrix[2], distCoeffs[2];
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R, T, E, F;
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];
	Size imageSize(320, 240);   //修改分辨率记得修改

	fs.open("extrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY,-1, imageSize, &validRoi[0], &validRoi[1]);


	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	destroyAllWindows();

	Mat imgLeft, imgRight;

	int ndisparities = 16 * 5;   /**< Range of disparity */
	int SADWindowSize = 31; /**< Size of the block window. Must be odd */
	Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
	sbm->setMinDisparity(0);
	//sbm->setNumDisparities(64);
	sbm->setTextureThreshold(10);
	sbm->setDisp12MaxDiff(-1);
	sbm->setPreFilterCap(31);
	sbm->setUniquenessRatio(25);
	sbm->setSpeckleRange(32);
	sbm->setSpeckleWindowSize(100);


	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 64, 7,
		10 * 7 * 7,
		40 * 7 * 7,
		1, 63, 10, 100, 32, StereoSGBM::MODE_SGBM);


	Mat rimg, cimg;
	Mat Mask;
    printf("********************finish camera,begin main task*********************\n");
    while (1)
	{
		cam >> input_image;
		frame_l = Mat(input_image, left_rect).clone();
		frame_r = Mat(input_image, right_rect).clone();//分离左右视野

		if (frame_l.empty() || frame_r.empty())
			continue;

		remap(frame_l, rimg, rmap[0][0], rmap[0][1], INTER_LINEAR);
		rimg.copyTo(cimg);
		Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(w * 0, 0, w, h)) : canvas(Rect(0, h * 0, w, h));
		resize(cimg, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);
		Rect vroi1(cvRound(validRoi[0].x * sf), cvRound(validRoi[0].y * sf),
			cvRound(validRoi[0].width * sf), cvRound(validRoi[0].height * sf));

		remap(frame_r, rimg, rmap[1][0], rmap[1][1], INTER_LINEAR);
		rimg.copyTo(cimg);
		Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w * 1, 0, w, h)) : canvas(Rect(0, h * 1, w, h));
		resize(cimg, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);
		Rect vroi2 = Rect(cvRound(validRoi[1].x * sf), cvRound(validRoi[1].y * sf),
			cvRound(validRoi[1].width * sf), cvRound(validRoi[1].height * sf));

		Rect vroi = vroi1 & vroi2;

		imgLeft = canvasPart1(vroi).clone();
		imgRight = canvasPart2(vroi).clone();

		rectangle(canvasPart1, vroi1, Scalar(0, 0, 255), 3, 8);
		rectangle(canvasPart2, vroi2, Scalar(0, 0, 255), 3, 8);

		if (!isVerticalStereo)
			for (int j = 0; j < canvas.rows; j += 32)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (int j = 0; j < canvas.cols; j += 32)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);


		cvtColor(imgLeft, imgLeft, COLOR_BGR2GRAY);
		cvtColor(imgRight, imgRight, COLOR_BGR2GRAY);


		//-- And create the image in which we will save our disparities
		Mat imgDisparity16S = Mat(imgLeft.rows, imgLeft.cols, CV_16S);
		Mat imgDisparity8U = Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);
		Mat sgbmDisp16S = Mat(imgLeft.rows, imgLeft.cols, CV_16S);
		Mat sgbmDisp8U = Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);

		if (imgLeft.empty() || imgRight.empty())
		{
			std::cout << " --(!) Error reading images " << std::endl; return -1;
		}

		sbm->compute(imgLeft, imgRight, imgDisparity16S);

		imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255.0 / 1000.0);
		cv::compare(imgDisparity16S, 0, Mask, CMP_GE);
		applyColorMap(imgDisparity8U, imgDisparity8U, COLORMAP_HSV);
		Mat disparityShow;
		imgDisparity8U.copyTo(disparityShow, Mask);




		sgbm->compute(imgLeft, imgRight, sgbmDisp16S);

		sgbmDisp16S.convertTo(sgbmDisp8U, CV_8UC1, 255.0 / 1000.0);
		cv::compare(sgbmDisp16S, 0, Mask, CMP_GE);
		applyColorMap(sgbmDisp8U, sgbmDisp8U, COLORMAP_COOL);
		Mat  sgbmDisparityShow, sgnm[3];
		sgbmDisp8U.copyTo(sgbmDisparityShow, Mask);



		int pix = sgbmDisparityShow.at<Vec3b>(170, 150)[1];//分离出视野中心点的灰度值（170，150）是视野中心点坐标【1】表示是第二个通道
		double alpha = 0.35;
		double intercept = 24.75; //计算距离需要使用的两个参数
		int distance = pix * alpha + intercept;//实际距离
		cout << distance << "cm" << endl;

		split(sgbmDisparityShow, sgnm);
		imshow("bmDisparity", disparityShow);//原始左右摄像头图像显示
		imshow("sgbmDisparity", sgbmDisparityShow);//热能图显示
		imshow("sgbm", sgnm[1]);//灰度显示
		imshow("rectified", canvas);//经过处理的热能图
		char c = (char)waitKey(1);
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
    
    
    
    nRe = Nim_set_vmTargetSpeed(hMaster, nAddrs[0], 200, 1);
    
    MySleepMS(4000);

    

    for (int nIndex = 0; nIndex < nSlaveCount; nIndex++)
    {
        nRe = Nim_power_off(hMaster, nAddrs[nIndex], 1);
    }
    MySleepMS(50);          // 必要延时(Nim_power_off后)
    nRe = Nim_master_changeToPreOP(hMaster);
    MySleepMS(50);          // 必要延时(Nim_master_changeToPreOP后)
EXIT_APP3:
    nRe = Nim_master_stop(hMaster);
EXIT_APP2:
    Nim_destroy_master(hMaster);
EXIT_APP1:
    Nim_clean();
    return 0;
}
