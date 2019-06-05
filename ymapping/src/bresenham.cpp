#include  <cv.h>
#include  <highgui.h>
#include  <cxcore.h>
#include  <iostream>
#include  <cmath>

using namespace std;
IplImage *imgA;
CvSize window = { 300, 300 };    //���ڴ�С
int xx1, yy1, xx2, yy2;//ֱ�ߵ������˵�
int dx, dy, a, a1, e;//������������ȱ���

void bresenham(int xx1, int yy1, int xx2, int yy2)
{
 int x, y;
  /*����ֱ�ߵ������˵㣨20��20������250��100�� */
 // xx1 = 0, yy1 = 0;
 // xx2 = 250, yy2 = 100;
  /*�������������� */
  dx = abs (xx2 - xx1);
  dy = abs (yy2 - yy1);
    /* �趨��ʼ�� */    
  y = yy1;
  x = xx1;
  int sx, sy;//��������ı���

  if (xx1 > xx2) sx = -1;//x�ݼ�����
  else sx = 1;//x��������

  if (yy1 > yy2) sy = -1;//y�ݼ�����
  else sy = 1;//y��������


  if (dx >= dy)
    {
     e=0;
      for (x = xx1; (sx >= 0 ? x <= xx2 : x >= xx2); x += sx)
        {
          imgA->imageData[y * imgA->widthStep + x ] = (signed char) 255;//�ڶ�Ӧ��դ���ϻ���
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
          imgA->imageData[y * imgA->widthStep + x ] = (signed char) 255;//�ڶ�Ӧ��դ���ϻ���
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
  imgA = cvCreateImage (window, IPL_DEPTH_8U, 1);//����ͼ��
  cvSet (imgA, cvScalarAll (0), 0);//���Ϊ0

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
