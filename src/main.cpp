#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;


int main()
{
    //read the image
    Mat img =imread("/home/guojinxuan/my_cmake_project/opencv_project/resources/test_image_2.png");
    if(img.empty())
    {
        std::cout << "can't read image!" <<std::endl;
        return -1;
    }

    //display image
    imshow("original image",img);
    waitKey(0);

    //image grayscale conversion
    Mat gray;
    cvtColor(img,gray,COLOR_BGR2GRAY);
    imshow("grey_scale map",gray);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/grey_scale_image.png", gray);


    //translate into HSV
    Mat hsvImage;
    cvtColor(img, hsvImage, COLOR_BGR2HSV);
    imshow("HSV image", hsvImage);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/hsv_image.png", hsvImage);


    //average filtering
    Mat blurImg;
    blur(img,blurImg,Size(5,5));
    imshow("average filtering",blurImg);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/average_filter_image.png", blurImg);


    //Gaussian filter
    Mat gaussianImg;
    GaussianBlur(img,gaussianImg,Size(5,5),1.5);
    imshow("Gaussian filter",gaussianImg);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/Gaussian filter_image.png", gaussianImg);

    //median filter
    Mat medianImg;
    medianBlur(img,medianImg,5);
    imshow("median filter",medianImg);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/median filter_image.png", medianImg);
    
    //red area
    Mat hsv, mask;
    cvtColor(img, hsv, COLOR_BGR2HSV);
    inRange(hsv,Scalar(0,100,100),Scalar(10,255,255),mask);
    Mat mask2;
    inRange(hsv, Scalar(170, 100, 100), Scalar(180, 255, 255), mask2);
    mask = mask | mask2;
    imshow("red area",mask);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/red area_image.png", mask);

    //red area outline
    Mat adaptive;
    adaptiveThreshold(gray,adaptive,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);
    imshow("adaptive binarization",adaptive);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/red area outline_image.png", adaptive);

    //gradient-based edge detection
    Mat edges;
    Canny(gray,edges,100,200);
    imshow("canny",edges);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/gradient-based edge detection_image.png", edges);


    //color edge detection
    Mat hsvChannels[3];
    cvtColor(img, hsv, COLOR_BGR2HSV);
    split(hsv,hsvChannels);
    Mat colorEdges;
    Canny(hsvChannels[0],colorEdges,100,200);
    imshow("color edge H",colorEdges);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/color edge detection_image.png", colorEdges);

     //morphological operation
     Mat morph;
     Mat kernel=getStructuringElement(MORPH_RECT,Size(3,3));
     morphologyEx(edges,morph,MORPH_CLOSE,kernel);
     imshow("morphological operation",morph);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/morphological operation_image.png", morph);
/*
    //contour extraction
    vector<vector<Point>> contours;
    vector<Vec4i>hierarchy;
    findContours(morph,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

    Mat contourImg = Mat::zeros(img.size(),CV_8UC3);
    for(size_t i=0;i<contours.size();i++)
    {
        drawContours(contourImg,contours,(int)i,Scalar(0,255,0),2);
    }
    imshow("contour extraction",contourImg);
    waitKey(0);

    //contour select
    for(size_t i=0;i<contours.size();i++)
    {
        double area=contourArea(contours[i]);
        if (area<500)continue;
        Rect bbox=boundingRect(contours[i]);
        rectangle(img,bbox,Scalar(0,0,255),2);
    }
    imshow("contour select",img);
    waitKey(0);
*/
    //light bar
    double threshold_value = 252;  // 阈值可以根据实际情况调整，范围是 [0, 255]
    Mat highBrightnessMask;
    threshold(gray, highBrightnessMask, threshold_value, 255, THRESH_BINARY);

    // 查找高亮区域的轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(highBrightnessMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 遍历每个轮廓
    for (size_t i = 0; i < contours.size(); i++)
    {
        // 计算轮廓的面积
        double area = contourArea(contours[i]);
        
        // 如果面积小于一定阈值（例如 500），则忽略这个轮廓
        if (area < 550) continue;

        // 计算最小外接矩形
        Rect bbox = boundingRect(contours[i]);
        
        // 绘制矩形框，框选高亮区域
        rectangle(img, bbox, Scalar(0, 0, 255), 2);
    }

    // 显示图像
    imshow("High Brightness Areas", img);
    waitKey(0);  // 等待键盘输入
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/light bar_image.png", img);

    //paint circle,square and some words
    Point center(250,250);
    int radius=100;
    Scalar circleColor(0,0,255);
    int thickness=2;
    circle(img,center,radius,circleColor,thickness);
    Point topLeft(50, 50); // 左上角
    Point bottomRight(150, 150); // 右下角
    Scalar rectangleColor(255, 0, 0); // 蓝色
    rectangle(img, topLeft, bottomRight, rectangleColor, thickness);
    string text = "Hello DXRMV!";
    Point textPosition(200, 400); // 文字位置
    Scalar textColor(0, 255, 0); // 绿色
    int fontFace = FONT_HERSHEY_SIMPLEX; // 字体
    double fontScale = 1.0; // 字体大小
    int thicknessText = 2; // 字体粗细
    putText(img, text, textPosition, fontFace, fontScale, textColor, thicknessText);
    imshow("geometric figure and words",img);
    waitKey(0); 
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/geometric figure and words_image.png", img);

    //rotate the image
    Point2f center1(img.cols / 2.0f, img.rows / 2.0f);
    double angle = 35.0;
    Mat rotationMatrix = getRotationMatrix2D(center1, angle, 1.0); // 旋转矩阵 (中心, 角度, 缩放因子)
    Rect boundingBox = RotatedRect(center, img.size(), angle).boundingRect();
    Mat rotatedImage;
    warpAffine(img, rotatedImage, rotationMatrix, boundingBox.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
    imshow("Rotated Image", rotatedImage);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/rotate_image.png", rotatedImage);

    //crop
    int croppedWidth = img.cols / 2;
    int croppedHeight = img.rows / 2;
    Rect cropRegion(0, 0, croppedWidth, croppedHeight);
    Mat croppedImage = img(cropRegion);  // 提取左上角四分之一
    imshow("Cropped Image", croppedImage);
    waitKey(0);
    imwrite("/home/guojinxuan/my_cmake_project/opencv_project/output/Cropped Image.png", croppedImage);

    return 0;
}