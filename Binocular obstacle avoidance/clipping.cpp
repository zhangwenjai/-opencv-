//#include <opencv2/opencv.hpp>
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//int main() {
//    // ��Ĭ�ϵ�����ͷ
//    VideoCapture cap;
//
//    cap.open(0, CAP_DSHOW);
//
//    // �����Ƶ���Ƿ��
//    if (!cap.isOpened()) {
//        cerr << "ERROR: Unable to open the camera" << endl;
//        return -1;
//    }
//
//    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
//    // ����һ��������
//    int count = 0;
//
//    Mat image_left, image_right;
//    // ѭ����ȡ����ͷͼ��
//    while (true) {
//        // ��ȡһ֡ͼ��
//        Mat frame;
//        cap >> frame;
//        imshow("Frame", frame);
//
//        Rect left_rect(0, 0, 319, 240);  //����һ��Rect������cv�е��࣬�ĸ���������x,y,width,height  
//        Rect right_rect(320, 0, 319, 240);
//
//        image_left = Mat(frame, left_rect).clone();
//        image_right = Mat(frame, right_rect).clone();
//
//        // ���¡�s��������ͼ��
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
//        // ����ESC���˳�
//        if (c == 27) {
//            break;
//        }
//    }
//
//    // �ͷ���Ƶ��
//    cap.release();
//
//    // �ر����д���
//    //destroyAllWindows();
//
//    return 0;
//}
