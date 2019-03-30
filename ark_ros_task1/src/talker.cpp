#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<ros/ros.h>
#include<ark_ros_task1/board_pose.h>
#include<ark_ros_task1/danger_region.h>
#include<sstream>

using namespace std;
using namespace cv;

Mat img;
int rows,cols;
int rm=36,rM=80,bm=24,bM=71,gm=27,gM=68;
//int rm=31,rM=77,bm=39,bM=74,gm=33,gM=75;

bool thr(int i,int j)
{
	if((bm<img.at<Vec3b>(i,j)[0] && img.at<Vec3b>(i,j)[0]<bM) && (gm<img.at<Vec3b>(i,j)[1] && img.at<Vec3b>(i,j)[1]<gM) && (rm<img.at<Vec3b>(i,j)[2] && img.at<Vec3b>(i,j)[2]<rM))
			return 1;
	return 0;
}

bool isValid(int i,int j)
{
	if(i>=0&&j>=0&&i<rows&&j<cols)
		return 1;
	return 0;
}

void binary(Mat a)
{
	for(int i=0;i<a.rows;i++)
	{
		for(int j=0;j<a.cols;j++)\
		{
			if(a.at<uchar>(i,j)!=255)
				a.at<uchar>(i,j)=0;
		}
	}
}

void erode(Mat a)
{
	Mat c=a.clone();
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			if(c.at<uchar>(i,j)==0)
			{
				bool t=1;
				for(int x=-1;x<2;x++)
				{
					if(!t)
						break;
					for(int y=-1;y<2;y++)
					{
						if(isValid(i+x,j+y)&&c.at<uchar>(i+x,j+y)==255)
						{
							a.at<uchar>(i,j)=255;
							t=0;
							break;
						}
					}
				}
			}
		}
	}
}

void dilate(Mat a)
{
	Mat c=a.clone();
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			if(c.at<uchar>(i,j)==255)
			{
				bool t=1;
				for(int x=-1;x<2;x++)
				{
					if(!t)
						break;
					for(int y=-1;y<2;y++)
					{
						if(isValid(i+x,j+y)&&c.at<uchar>(i+x,j+y)==0)
						{
							a.at<uchar>(i,j)=0;
							t=0;
							break;
						}
					}
				}
			}
		}
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"talker");
	ros::NodeHandle n;
	ros::Publisher pub=n.advertise<ark_ros_task1::board_pose>("check_pose",1);
	ros::ServiceClient client = n.serviceClient<ark_ros_task1::danger_region>("check_pose");
	ros::Rate loop_rate(30);
	ark_ros_task1::danger_region srv;
	VideoCapture cap("/home/yashsoni501/Desktop/ARK/new.avi");
	double fps=cap.get(CV_CAP_PROP_FPS);
	//cout<<fps<<endl;
	bool inDanger=0;

	while(ros::ok())
	{
		cap>>img;
		if(img.empty())
			break;
		Mat d=img.clone();
		GaussianBlur(img,img,Size(7,7),2);
	
		rows=img.rows;
		cols=img.cols;
		Mat a;
		cvtColor(img,a,COLOR_RGB2GRAY);
		for(int i=0;i<img.rows;i++)
		{
			for(int j=0;j<img.cols;j++)
			{
				if(!thr(i,j))
				{
					a.at<uchar>(i,j)=255;
				}
			}
		}
		
		binary(a);
		/*
		for(int i=0;i<5;i++)
		{
			erode(a);
		}
		for(int i=0;i<7;i++)
		{
			dilate(a);
		}
		*/
		erode(a);
		int x,y,cnt;
		x=y=cnt=0;
		for(int i=0;i<rows;i++)
		{
			for(int j=0;j<cols;j++)
			{
				if(a.at<uchar>(i,j)==0)
				{
					x+=i;
					y+=j;
					cnt++;
				}
			}
		}
		
		x/=cnt;
		y/=cnt;
		
		int x2,y2,cnt2;
		x2=y2=cnt2=0;
		for(int i=0;i<rows;i++)
		{
			for(int j=0;j<cols;j++)
			{

				if(i>x-250&&i<x+250 && j>y-150&&j<y+150)
					if(a.at<uchar>(i,j)==0)
					{
						x2+=i;
						y2+=j;
						cnt2++;
					}
			}
		}

		ark_ros_task1::board_pose pt;
		if(cnt2<5000)
		{
			//cout<<"Board absent"<<endl;
			pt.x=0;
			pt.y=0;
			pub.publish(pt);
			continue;
		}
		else
		{
			x2/=cnt2;
			y2/=cnt2;
			//cout<<x<<" "<<y<<endl;
			//cout<<"x coordinate "<<x2<<" y coordinate "<<y2<<endl;
			pt.x=x2;
			pt.y=y2;
			pub.publish(pt);
		}

		for(int i=-2;i<3;i++)
		{
			for(int j=-2;j<3;j++)
			{
				if(isValid(x2+i,y2+j))
					d.at<Vec3b>(x2+i,y2+j)={0,0,255};
			}
		}

		if(y2<420&&y2>380 && x2<370&&x2>330)
		{
			if(!inDanger)
			{
				inDanger=1;		
				srv.request.status=true;
				client.call(srv);
				ROS_WARN("Received from danger_region: %s",srv.response.out);
			}		
		}
		else
		{
			inDanger=0;
		}
		
		namedWindow("window",WINDOW_NORMAL);

		imshow("window",d);
		waitKey(1);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
