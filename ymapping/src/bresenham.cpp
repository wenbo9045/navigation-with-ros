#include  <cv.h>
#include  <highgui.h>
#include  <cxcore.h>
#include  <iostream>
#include  <cmath>

using namespace std;
IplImage *imgA;
CvSize window = { 300, 300 };    //窗口大小
int xx1, yy1, xx2, yy2;//直线的两个端点
int dx, dy, a, a1, e;//定义差量，误差等变量

void bresenham(int xx1, int yy1, int xx2, int yy2)
{
 int x, y;
  /*设置直线的两个端点（20，20），（250，100） */
 // xx1 = 0, yy1 = 0;
 // xx2 = 250, yy2 = 100;
  /*计算横纵坐标差量 */
  dx = abs (xx2 - xx1);
  dy = abs (yy2 - yy1);
    /* 设定起始点 */    
  y = yy1;
  x = xx1;
  int sx, sy;//表明方向的变量

  if (xx1 > xx2) sx = -1;//x递减方向
  else sx = 1;//x递增方向

  if (yy1 > yy2) sy = -1;//y递减方向
  else sy = 1;//y递增方向


  if (dx >= dy)
    {
     e=0;
      for (x = xx1; (sx >= 0 ? x <= xx2 : x >= xx2); x += sx)
        {
          imgA->imageData[y * imgA->widthStep + x ] = (signed char) 255;//在对应的栅格上绘制
          e+=dy;
          if ((e<<1) >= dx)
            {
              y += sy;
              e -= dx;
            }
            
        }
    }
  else
  {
      e=0;
      for (y = yy1; (sy >= 0 ? y <= yy2 : y >= yy2); y += sy)
        {
          imgA->imageData[y * imgA->widthStep + x ] = (signed char) 255;//在对应的栅格上绘制
          e+=dx;
          if ((e<<1) >= dy)
            {
              x += sx;
              e -= dy;
            }
            
        }
  }
    
}


int main (int argc, char *argv[])
{
  imgA = cvCreateImage (window, IPL_DEPTH_8U, 1);//建立图像
  cvSet (imgA, cvScalarAll (0), 0);//填充为0

 // xx1 = 0, yy1 = 0;
 // xx2 = 250, yy2 = 100;
 bresenham(0,0,100,100);
 

  cvNamedWindow ("window", CV_WINDOW_AUTOSIZE);
  cvShowImage ("window", imgA);

  cvWaitKey (0);

  cvReleaseImage (&imgA);
  cvDestroyWindow ("window");

  return 0;
}
