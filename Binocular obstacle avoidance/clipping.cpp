//#include <opencv2/opencv.hpp>
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//int main() {
//    // 打开默认的摄像头
//    VideoCapture cap;
//
//    cap.open(0, CAP_DSHOW);
//
//    // 检查视频流是否打开
//    if (!cap.isOpened()) {
//        cerr << "ERROR: Unable to open the camera" << endl;
//        return -1;
//    }
//
//    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
//    // 定义一个计数器
//    int count = 0;
//
//    Mat image_left, image_right;
//    // 循环读取摄像头图像
//    while (true) {
//        // 获取一帧图像
//        Mat frame;
//        cap >> frame;
//        imshow("Frame", frame);
//
//        Rect left_rect(0, 0, 319, 240);  //创建一个Rect框，属于cv中的类，四个参数代表x,y,width,height  
//        Rect right_rect(320, 0, 319, 240);
//
//        image_left = Mat(frame, left_rect).clone();
//        image_right = Mat(frame, right_rect).clone();
//
//        // 按下“s”键保存图像
//        char c = (char)waitKey(25);
//        if (c == 's') {
//            string imname1;
//            imname1 = "G:/tianchan/lujinshibie/left/" + to_string(count) + "left.jpg";
//            imwrite(imname1, image_left);
//            cout << "Saved " << count<<"left" << endl;
//            
//            string imname2;
//            imname2 = "G:/tianchan/lujinshibie/right/" + to_string(count) + "right.jpg";
//            imwrite(imname2, image_right);
//            cout << "Saved " << count << "tight" << endl;
//            count++;
//        }
//
//        // 按下ESC键退出
//        if (c == 27) {
//            break;
//        }
//    }
//
//    // 释放视频流
//    cap.release();
//
//    // 关闭所有窗口
//    //destroyAllWindows();
//
//    return 0;
//}
