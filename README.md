# 基于opencv的视觉导航+双目避障算法
# 关键技术点
opencv库的使用、串口通讯技术、ROI区域提取、车道线/边缘线检测、开闭运算、最小二乘法拟合、双目标定、立体匹配、SGBM算法等。
# 项目背景
1.智能移动机器人的视觉导航方案，对移动机器人检测到的边缘线、可行区域进行标记，并且通过边缘信息点通过最小二乘法推算出控制系统应该进行的操作。

2.根据双目相机获得的视差图计算出当前场景中的深度值，对于距离过近的识别为障碍物并进行避让。
# 功能实现设计
calibandtest实现双目标定获得双目相机的内外参数。

clipping用于剪裁出左右相机图像，可用于双目标定。

extrinsics.yml和intrinsics.yml为标定后获得的相机内外惨。

motorandcamera为最终控制代码，实时处理图像并识别障碍物进行避障。

serialport.h和serialport.c实现串口通信的所有功能，所有功能的设计都有详细的注释。

bendtest.c中实现全部功能，主要介绍一下视觉方面的设计：

GetROI（）函数获得感兴趣区域，大幅降低图像处理时的数据量，提高实时性。

DetectRoadLine（）在ROI区域中进行边缘检测、最小二乘法拟合、车道线绘制以及控制信息的传输。


# 开发平台及对应版本
opencv库版本众多，功能强大，本次设计中所使用的都为一些基础方法，不存太多版本问题。

电机的控制采用了电机商家立迈胜提供的SDK，项目代码在Ubuntu18及22通过测试。

IDE：VS2019 OPCV：opencv410 vritual serial port driver 
# 相关环境搭建
在vs下要想完成串口的通讯需要使用到虚拟串口软件以及串口调试助手来帮助我们设置以及观察串口的收发效果，虚拟串口推荐使用vritual serial port driver，串口调试助手推荐uartassist，安装方式如下

>https://blog.csdn.net/qq_40296728/article/details/132159837?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522171014153716800185899391%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=171014153716800185899391&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-132159837-null-null.142^v99^pc_search_result_base9&utm_term=%E4%B8%B2%E5%8F%A3%E8%B0%83%E8%AF%95%E5%8A%A9%E6%89%8B&spm=1018.2226.3001.4187

>https://blog.csdn.net/qq_17351161/article/details/89607458?ops_request_misc=&request_id=&biz_id=102&utm_term=%E8%99%9A%E6%8B%9F%E4%B8%B2%E5%8F%A3vspd&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-89607458.142^v99^pc_search_result_base9&spm=1018.2226.3001.4187

# 可能会遇到问题
第一：COM10以上的端口连接失败

设置端口信息时com10一下可以直接使用名称，10以上要写成////.COM10//。

第二：createfileA失败

查看串口设置的波特率是否相同，查看createfileA的参数设置是否匹配，不匹配也会造成错误。

第三：运行代码时立体匹配完的图像提示为空

大概率相机内外参数标定出现问题，检查两个yml文件中的内容是否有异常，可以进行多次标定检查解决问题。
