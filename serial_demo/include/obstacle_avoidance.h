#include<opencv2/opencv.hpp>
class obstavd_solv{
	private:
		float trace[100][2],thstp,ep[2];
		float winsize=300;float drct0[2];
		int gap=winsize/25,p;//轮廓最小间距像素点数
		Mat map;
		void find_colsp_point(int *i,int *j){
			*i=trace[p][0],*j=trace[p][1];
			do{
				*i+=drct0[0],*j+=drct0[1];
				if(*i<0 or *i>=map.rows or *j<0 or *j>=map.cols) break;
			}while((int)map.at<uchar>(*i,*j)==255);//要求一定有闭合外边界
		}
		bool deep_prior_trace(float agl){
			drct0[0]=ep[0]-trace[p][0],drct0[1]=ep[1]-trace[p][1];//下一步检索向量
			int i,j;
			vec2d_rescale(drct0,1,drct0);
			//找到当前点走向终点的路上第一个障碍
			find_colsp_point(&i,&j);
			if(i-ep[0]<1 and j-ep[1]<1){//没有障碍，可直达终点
				trace[++p][0]=ep[0],trace[p][1]=ep[1];//结束
				return true;
			}else{
				//cout<<"colps at i "<<i<<" j "<<j<<endl;
				//偏移当前前进方向寻找绕开障碍物的切线
				int p1,p2,i1,j1,i2,j2;
				i1=i2=i,j1=j2=j;
				float theta0=calcu_angle(drct0[1],drct0[0],1),the;
				thstp=0.1/sqrt((i-trace[p][0])*(i-trace[p][0])+(j-trace[p][1])*(j-trace[p][1]));
				//cout<<"step: "<<thstp<<endl;
				for(float theta=thstp;theta<pi;theta+=thstp){
					the=theta0+theta;if(the<-pi) the+=2*pi;if(the>pi) the-=2*pi;
					drct0[0]=cos(the),drct0[1]=sin(the);
					find_colsp_point(&p1,&p2);
					if((p1-i1)*(p1-i1)+(p2-j1)*(p2-j1)>gap*gap and acos(cos(agl-the))<0.7*pi){
						trace[++p][0]=(i1+p1)/2,trace[p][1]=(j1+p2)/2;//跳
						//cout<<"at p "<<p<<" theta "<<the<<" jump to i: "<<trace[p][0]<<" j: "<<trace[p][1]<<endl;
						map.at<uchar>(trace[p][0],trace[p][1])=0;//imshow("obsmap.pgm",map);waitKey(0);
						if(deep_prior_trace(the)) return true;else{i1=p1,j1=p2;}
					}else{
						i1=p1,j1=p2;
					}
					the=theta0-theta;if(the<-pi) the+=2*pi;if(the>pi) the-=2*pi;
					drct0[0]=cos(the),drct0[1]=sin(the);
					find_colsp_point(&p1,&p2);
					if((p1-i2)*(p1-i2)+(p2-j2)*(p2-j2)>gap*gap and acos(cos(agl-the))<0.7*pi){
						trace[++p][0]=(i2+p1)/2,trace[p][1]=(j2+p2)/2;//跳
						//cout<<"at p "<<p<<" theta "<<the<<" jump to i: "<<trace[p][0]<<" j: "<<trace[p][1]<<endl;
						map.at<uchar>(trace[p][0],trace[p][1])=0;//imshow("obsmap.pgm",map);waitKey(0);
						if(deep_prior_trace(the)) return true;else{i2=p1,j2=p2;}
					}else{
						i2=p1,j2=p2;
					}
				}
				p--;return false;
			}
		}

   	

		void init_map(string name){
			Mat a,b,c,d;
			int color,th=170;
			a=imread(name);
			float scale=winsize/max(a.rows,a.cols);
			Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
			if(!a.empty()){
				cvtColor(a,b,COLOR_BGR2GRAY);
				resize(b,c, Size(),scale,scale);
				threshold(c,d,th,255,THRESH_BINARY);
				imshow("d",d);waitKey(0);
   				//dilate(d,map, element);
   				erode(d,map, element);
				imshow("map",map);waitKey(0);
				imwrite("./toyobsmap.pgm", map);
			}
		}
	public: 
		void plan_trace(float startpoint[2],float endpoint[2]){
			map=imread("./toyobsmap.pgm",0);
			if(map.empty()){
				//init_map("./map.jpg");
				init_map("./toyground.jpg");
				map=imread("./toyobsmap.pgm",0);
			}
			trace[0][0]=map.rows*startpoint[0],trace[0][1]=map.cols*startpoint[1];
			ep[0]=map.rows*endpoint[0],ep[1]=map.cols*endpoint[1];
			map.at<uchar>(trace[0][0],trace[0][1])=0,map.at<uchar>(ep[0],ep[1])=0;
			imshow("obsmap.pgm",map);waitKey(0);
			p=0;
			if(deep_prior_trace(0)){
				cout<<"trace points num "<<p<<endl;
				float v[2],v0[2],l,t;
				for(int i=0;i<p;i++){
					cout<<" jump to i: "<<trace[i][0]<<" j: "<<trace[i][1]<<endl;
					v[0]=trace[i+1][0]-trace[i][0],v[1]=trace[i+1][1]-trace[i][1];
					l=vec_length(v);
					vec2d_rescale(v,1,v0);
					for(t=0;t<=l;t++){
						map.at<uchar>(trace[i][0]+t*v0[0],trace[i][1]+t*v0[1])=0;
					}
				}
				imshow("planed_trace",map);waitKey(0);
			}

		}
};
