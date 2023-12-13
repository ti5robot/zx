#include<cstdlib>
#include<cstring>
#include<sstream>
#include<unistd.h>
#include<opencv2/opencv.hpp>
#include <move/robotmove.h>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
using namespace cv;
Mat draw_curve(int p2d[][2],int p,int rows,int cols){
	//绘制
	///*
	int i;Mat e(Size(cols,rows),CV_8UC1,Scalar(255));//int color=0;
	//float cz=118,cy=0,cx=300;
	float cz=133,cy=-TH.rball/sqrt(2)/2,cx=-TH.rball/sqrt(2)/2,h=50;//cz---lowest level
	float slpt=110000;
	float nmax=240;float nm[6]={nmax,nmax,nmax,nmax,nmax,nmax};setn(nm);
	TH.pos[3]=0,TH.pos[4]=pi,TH.pos[5]=0;
	TH.pos[0]=cx-rows/2+p2d[0][0];TH.pos[1]=cy-cols/2+p2d[0][1];TH.pos[2]=cz+h;move_to_pos(2);sleep(4);//移动到新起点
	TH.pos[2]=cz;move_to_pos(2);sleep(3);//下笔
	e.at<uchar>(p2d[0][0],p2d[0][1])=saturate_cast<uchar>(0);
	for(i=1;i<p-1;i++){
		if(p2d[i][0]==-1){
			i++;
			sleep(1);
			TH.pos[2]=cz+h;move_to_pos(2);sleep(3);
		    	TH.pos[0]=cx-rows/2+p2d[i][0];TH.pos[1]=cy-cols/2+p2d[i][1];move_to_pos(2);sleep(2);
		    	TH.pos[2]=cz;move_to_pos(2);sleep(3);
		    	//color++;
		}
		TH.pos[0]=cx-rows/2+p2d[i][0];
		TH.pos[1]=cy-cols/2+p2d[i][1];;
		move_to_pos(0);usleep(slpt);
		e.at<uchar>(p2d[i][0],p2d[i][1])=saturate_cast<uchar>(0);
	}
	TH.pos[2]=cz+h;move_to_pos(2);sleep(3);//抬笔
	imshow("endimg",e);waitKey(0);
	return e;
}

int get_outline(int p2d[][2],Mat d,int area){
	int drt;
	bool startpoint=false;
	float lm;
	int stp=3,drct,d1,d2,p=0,p0=0;
	int temp;//i,j
	int i,j,i1,j1,x,y,t;
	for(x=0;x<0.8*d.rows;x++){
		for(y=0;y<d.cols;y++){
			if((int)d.at<uchar>(x,y)==0){
				i=x-1,j=y+1;
				//cout<<"first p"<<i<<" "<<j<<endl;
				d.at<uchar>(i,j)=saturate_cast<uchar>(255);
				p0=p;p2d[p][0]=i,p2d[p++][1]=j;drt=9;
				//计划曲线轨迹
				while(drt){
					//检索向量
					lm=0;drt=0;
					//向左
					for(drct=j-1;drct>=0;drct--) if((int)d.at<uchar>(i,drct)==255) break;
					if(j-drct-1>lm) {
						drt=1;lm=j-drct-1;i1=i,j1=min(j-1,drct+stp);
					}
					//向左上
					for(d1=i-1,d2=j-1;d1>=0 and d2>=0;d1--,d2--) if((int)d.at<uchar>(d1,d2)==255) break;
					if(1.414*(j-d2-1)>lm) {
						drt=2;lm=1.414*(j-d2-1);i1=min(i-1,d1+stp),j1=min(j-1,d2+stp);
					}
					//向上
					for(drct=i-1;drct>=0;drct--) if((int)d.at<uchar>(drct,j)==255) break;
					if(i-drct-1>lm) {
						drt=3;lm=i-drct-1;i1=min(i-1,drct+stp),j1=j;
					}
					//向右上
					for(d1=i-1,d2=j+1;d2<d.cols and d1>=0;d1--,d2++) if((int)d.at<uchar>(d1,d2)==255) break;
					if(1.414*(d2-j-1)>lm) {
						drt=4;lm=1.414*(d2-j-1);i1=min(i-1,d1+stp),j1=max(j+1,d2-stp);
					}
					//向右
					for(drct=j+1;drct<d.cols;drct++) if((int)d.at<uchar>(i,drct)==255) break;
					if(drct-j-1>lm) {
						drt=5;lm=drct-j-1;i1=i,j1=max(j+1,drct-stp);
					}
					//向右下
					for(d1=i+1,d2=j+1;d1<d.rows and d2<d.cols;d1++,d2++) if((int)d.at<uchar>(d1,d2)==255) break;
					if(1.414*(d2-j-1)>lm) {
						drt=6;lm=1.414*(d2-j-1);i1=max(i+1,d1-stp),j1=max(j+1,d2-stp);
					}
					//向下
					for(drct=i+1;drct<d.rows;drct++) if((int)d.at<uchar>(drct,j)==255) break;
					if(drct-i-1>lm) {
						drt=7;lm=drct-i-1;i1=max(i+1,drct-stp),j1=j;
					}
					//向左下
					for(d1=i+1,d2=j-1;d1<d.rows and d2>=0;d1++,d2--) if((int)d.at<uchar>(d1,d2)==255) break;
					if(1.414*(j-d2-1)>lm) {
						drt=8;lm=1.414*(j-d2-1);i1=max(i+1,d1-stp),j1=min(j-1,d2+stp);
					}
					if(drt==1){
						d.at<uchar>(i,j+1)=saturate_cast<uchar>(255);
						while(j>j1){
							d.at<uchar>(i,j)=d.at<uchar>(i-1,j+1)=d.at<uchar>(i+1,j+1)=saturate_cast<uchar>(255);
							j--;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}else if(drt==2){
						d.at<uchar>(i+1,j+1)=saturate_cast<uchar>(255);
						while(i>i1){
							d.at<uchar>(i,j)=d.at<uchar>(i,j+1)=d.at<uchar>(i+1,j)=saturate_cast<uchar>(255);
							i--,j--;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}else if(drt==3){
						d.at<uchar>(i+1,j)=saturate_cast<uchar>(255);
						while(i>i1){
							d.at<uchar>(i,j)=d.at<uchar>(i+1,j-1)=d.at<uchar>(i+1,j+1)=saturate_cast<uchar>(255);
							i--;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}else if(drt==4){
						d.at<uchar>(i+1,j-1)=saturate_cast<uchar>(255);
						while(i>i1){
							d.at<uchar>(i,j)=d.at<uchar>(i,j-1)=d.at<uchar>(i+1,j)=saturate_cast<uchar>(255);
							i--,j++;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}else if(drt==5){
						d.at<uchar>(i,j-1)=saturate_cast<uchar>(255);
						while(j<j1){
							d.at<uchar>(i,j)=d.at<uchar>(i-1,j-1)=d.at<uchar>(i+1,j-1)=saturate_cast<uchar>(255);
							j++;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}else if(drt==6){
						d.at<uchar>(i-1,j-1)=saturate_cast<uchar>(255);
						while(i<i1){
							d.at<uchar>(i,j)=d.at<uchar>(i,j-1)=d.at<uchar>(i-1,j)=saturate_cast<uchar>(255);
							i++,j++;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
						
					}else if(drt==7){
						d.at<uchar>(i-1,j)=saturate_cast<uchar>(255);
						while(i<i1){
							d.at<uchar>(i,j)=d.at<uchar>(i-1,j-1)=d.at<uchar>(i-1,j+1)=saturate_cast<uchar>(255);
							i++;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}else if(drt==8){
						d.at<uchar>(i-1,j+1)=saturate_cast<uchar>(255);
						while(i<i1){
							d.at<uchar>(i,j)=d.at<uchar>(i,j+1)=d.at<uchar>(i-1,j)=saturate_cast<uchar>(255);
							i++,j--;
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}
					}
					d.at<uchar>(i,j)=saturate_cast<uchar>(255);
					if(drt==0){
						lm=(area+1)*(area+1);int l=0,a,b;
						//cout<<"now i,j"<<i<<" "<<j<<endl;
						for(i1=max(0,i-area);i1<min(i+area,d.rows);i1++){
						    	for(j1=max(0,j-area);j1<min(j+area,d.cols);j1++){
						    		if((int)d.at<uchar>(i1,j1)==0){
						    			l=(i-i1)*(i-i1)+(j-j1)*(j-j1);
						    			if(l<lm){
						    				lm=l;
						    				a=i1,b=j1;
						    			}
						    		}
						    	}
						}
						if(l){
							i=a,j=b;
							drt=9;
							d.at<uchar>(i,j)=saturate_cast<uchar>(255);
							p2d[p][0]=i;
							p2d[p++][1]=j;
						}else{
							if(startpoint){
								if(p-p0<=5*area){
									p=p0; 
									//cout<<"pass"<<endl;
								}
								else{
									p2d[p][0]=-1;
									p2d[p++][1]=-1;
								}
								startpoint=false;
							}
							else{
								//逆置
								for(t=0;t<(p-p0)/2;t++){
									temp=p2d[p0+t][0],p2d[p0+t][0]=p2d[p-t-1][0],p2d[p-t-1][0]=temp;
									temp=p2d[p0+t][1],p2d[p0+t][1]=p2d[p-t-1][1],p2d[p-t-1][1]=temp;
								}
								//回到中间点
								i=x-1,j=y+1;
								startpoint=true;
								drt=9;
							}
						}
						//cout<<"next i,j"<<i<<" "<<j<<endl;
					}
				} 
				//imshow("process",d);waitKey(0);
			}
		}
	}
	return p;
}
void draw_outline(){
	Mat b,c,d;int th=90;float picmaxsize=276; 
	Mat a=imread("robot.jpg");
	//if(picmaxsize>=TH.rball/sqrt(2)) exit(0);
	float scale=picmaxsize/max(a.rows,a.cols);
	int p2d[3000][2],p;
	if(!a.empty()){
		cvtColor(a,b,COLOR_BGR2GRAY);
		threshold(b,c,th,255,THRESH_BINARY);
		resize(c,d, Size(),scale,scale);
		imshow("lk",d);waitKey(0);
		p=get_outline(p2d,d,4);
		cout<<p<<"points"<<endl;
		imshow("bp",d);waitKey(0);
		draw_curve(p2d,p,d.rows,d.cols);
    	}else cout<<"read error"<<endl;
}
void draw_pic(){
    int pre;
    //int th=245;//爱心
    //int th=100;Mat a=imread("./dog.jpg");float scale=100.0/max(a.rows,a.cols);
    int th=86;Mat a=imread("./boss.jpg");float scale=140.0/max(a.rows,a.cols);
    Mat b,c,d;
    //cout<<a.size()<endl;
    if(!a.empty()){
	    resize(a, b, Size(),scale,scale);
	    cvtColor(b,c,COLOR_BGR2GRAY);
	    threshold(c,d,th,255,THRESH_BINARY);
	    //threshold(c,e,th,255,THRESH_BINARY);
	    /*
	    vector<vector<Point>> contours;
	    vector<Vec4i> hierarchy;
	    findContours(d, contours, hierarchy, RETR_EXTERNAL,CHAIN_APPROX_NONE);
	    e = Mat::zeros(d.size(), CV_8UC3);
	    drawContours(e, contours,5, Scalar(0, 255, 0),1);
	    imshow("rst",e);
	    waitKey(0);
	    for(int i=0;i<contours.size();i++){
	    	drawContours(e, contours,i, Scalar(0, 255, 0),1);
	    	imshow("rst",e);
	    	waitKey(0);
	    	e = Mat::zeros(d.size(), CV_8UC3);
	    }
	    */
	    //for(int i=0;i<c.rows;i++)
	    //	for(int j=0;j<c.cols;j++)
	    //		cout<<int(c.at<uchar>(i,j))<<" ";
	    //imshow("gr",c);
	    //waitKey(0);
	    //cout<<d.size()<<endl;
	    imshow("bp",d);
	    waitKey(0);
	    float cz=121,cy=0,cx=200;//1kg
	    int sleeptime=200000;
	    while(1){
		    TH.pos[0]=cx;
		    TH.pos[1]=cy;
		    TH.pos[2]=cz+20;
		    TH.pos[3]=0,TH.pos[4]=pi,TH.pos[5]=0;
		    float nmax=500;float nm[6]={nmax,nmax,nmax,nmax,nmax,nmax};setn(nm);
		    move_to_pos(2);
		    sleep(4);
		    for(int i=0;i<d.rows;i++){
		    	for(int j=0;j<d.cols;j++){
		    		//扫描落笔
		    		if((int)d.at<uchar>(i,j)==0){
		    			TH.pos[0]=cx+d.rows/2-i;TH.pos[1]=cy+d.cols/2-j;
		    			move_to_pos(0);
		    			usleep(sleeptime);
					TH.pos[2]=cz-5;
			    		move_to_pos(0);
			    		usleep(sleeptime);
		    			TH.pos[2]=cz;
		    			move_to_pos(0);
		    			usleep(sleeptime);
		    		}
		    	}
		    	i++;
		    	for(int j=d.cols-1;j>=0;j--){
		    		//扫描落笔
		    		if((int)d.at<uchar>(i,j)==0){
		    			TH.pos[0]=cx+d.rows/2-i;TH.pos[1]=cy+d.cols/2-j;
		    			move_to_pos(0);
		    			usleep(sleeptime);
					TH.pos[2]=cz-5;
			    		move_to_pos(0);
			    		usleep(sleeptime);
		    			TH.pos[2]=cz;
		    			move_to_pos(0);
		    			usleep(sleeptime);
		    		}
		    	}
		    	//usleep(1800000);
		    }
		    TH.pos[2]=cz+20;
		    move_to_pos(0);
		    sleep(60);
	    }
    }
}

void draw_circle(){
    float theta=0,ts=0.01;
    float r=50,cz=260,cy=0;
    TH.pos[2]=r*cos(theta)+cz,TH.pos[1]=r*sin(theta)+cy,TH.pos[0]=-240;
    //TH.pos[3]=0,TH.pos[4]=pi,TH.pos[5]=0;
    //show_value("goto pos(x6,y6,z6,y,p,r):",TH.pos);
    move_to_pos(2);
    sleep(5);
    float nmax=500;float nm[6]={nmax,nmax,nmax,nmax,nmax,nmax};setn(nm);
    int sleeptime=10000;
    while(1){
    	TH.pos[2]=r*cos(theta)+cz,TH.pos[1]=r*sin(theta)+cy;
    	//show_value("goto pos(x6,y6,z6,y,p,r):",TH.pos);
    	move_to_pos(0);
    	theta+=ts;
    	usleep(sleeptime);
    }
}

void solve(const char* path,int mode,float t)
{
	FILE* fp = std::fopen(path, "r");
	if(fp) 
	{
		int num=0;
		float data[2500][6];
		memset(data,0,sizeof data);
		char buffer[1024];
        while (std::fgets(buffer, sizeof(buffer), fp)) 
		{
            std::string line(buffer);
	        std::stringstream ss(line);
	        std::string token;
			//file_logger->info(buffer);
	        int cnt=0;
    	    while (std::getline(ss, token, ',')) 
    	    {
				float number = std::atof(token.c_str());
				data[num][cnt]=number;
				if(cnt>=3) data[num][cnt]=number*pi/180;
				cnt++;
    	     }
			 num++;
        }
        std::fclose(fp);
		for(int i=0;i<num;i++)
		{
			for(int j=0;j<6;j++)
			{
				TH.pos[j]=data[i][j];
			}
			move_to_pos(mode);
			show_value("goto pos(x6,y6,z6,y,p,r):",TH.pos);
			
			sleep(t);
		}
	}
	else cout<<"open csv failed"<<endl;

}



void test_04(int mode)
{
	auto file_logger = spdlog::rotating_logger_mt("daily_logger", "/home/zx/move/src/test.log", 1024 * 300, 1);
    spdlog::set_default_logger(file_logger);
	/*auto file_logger = spdlog::daily_logger_mt("daily_logger", "/home/zx/move/src/daily_4.txt", 00, 00);
	auto file_logger = spdlog::basic_logger_mt("file", "/home/zx/move/src/logfile4_0.txt");
	auto file_logger = spdlog::basic_logger_mt("file", "/home/zx/move/src/logfile4_1.txt");
	auto file_logger = spdlog::basic_logger_mt("file", "/home/zx/move/src/logfile4_2.txt");*/
	file_logger->set_level(spdlog::level::debug);
	//spdlog::flush_every(std::chrono::seconds(10));
	FILE* fp = std::fopen("/home/zx/move/src/c4.csv", "r");
	/*bool f=check_file_test("log4.txt");
	if(!f) f=generate_file_test("log4.txt");*/
    if (fp) 
	{
        char buffer[1024];
        while (std::fgets(buffer, sizeof(buffer), fp)) 
		{
            std::string line(buffer);
			//if(f) write_into_file("log4.txt", buffer);
	        std::stringstream ss(line);
	        std::string token;
			file_logger->info(buffer);
	        int cnt=0;
    	    while (std::getline(ss, token, ',')) 
    	    {
        		float number = std::atof(token.c_str());
        		TH.pos[cnt]=number;
        		if(cnt>=3)TH.pos[cnt]=number*pi/180;
        		cnt++;
    	     }
    	     move_to_pos(mode);
    	     show_value("goto pos(x6,y6,z6,y,p,r):",TH.pos);sleep(1);
        }
        std::fclose(fp);
	}
}

void test_07(int mode)
{
	auto file_logger = spdlog::daily_logger_mt("daily_logger", "/home/zx/move/src/daily_7.txt", 00, 00);
	//auto file_logger = spdlog::basic_logger_mt("file", "/home/zx/move/src/logfile_7.txt");
	file_logger->set_level(spdlog::level::debug);
	FILE* fp = std::fopen("/home/zx/move/src/c7.csv", "r");
    if (fp) 
	{
        char buffer[1024];
        while (std::fgets(buffer, sizeof(buffer), fp)) 
		{
            std::string line(buffer);
	        std::stringstream ss(line);
	        std::string token;
			//file_logger->info(buffer);
	        int cnt=0;
    	    while (std::getline(ss, token, ',')) 
    	    {
				float number = std::atof(token.c_str());
				TH.pos[cnt]=number;
				if(cnt>=3)TH.pos[cnt]=number*pi/180;
				cnt++;
    	     }
			 move_to_pos(mode);
    	     show_value("goto pos(x6,y6,z6,y,p,r):",TH.pos);sleep(1);
        }
        std::fclose(fp);
	}
}
