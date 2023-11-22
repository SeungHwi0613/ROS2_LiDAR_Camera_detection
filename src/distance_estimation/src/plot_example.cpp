#include <opencv4/opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  Mat img(300, 300, CV_8UC3, Scalar(255, 0, 0)); // 도형을 넣을 규격이 정해진 빈 화면을 그리자. 크기 row=512, column=512, 8bit_unsigned, channel=3개, Scalar=BGR 
//   Mat imgResize;
//   resize(img, imgResize, Size(600, 500));
  circle(img, Point(150, 150), 20, Scalar(0, 255, 0), 5); // 이미지 삽입할 객체, 원의 중심점, 원의 반지름, 원의 색, 원의 꽉참 유무
//   rectangle(imgResize, Point(20, 20), Point(280, 280), Scalar(0, 0, 255), 5);
//   line(img, Point(280, 20), Point(20, 280), Scalar(0,0,0), 5);
  putText(img,"korea dream", Point(50, 40),FONT_HERSHEY_SIMPLEX ,1, Scalar(0,0,255), 0.5);
//   Rect roi(50, 50, 200, 200);
//   Mat ImgCrop = img(roi);
  imshow("basic_image", img);
//   imshow("roi_image", ImgCrop);
  waitKey(0); // 윈도우 끄려면 아무거나 누르시오
  return 0;
}
