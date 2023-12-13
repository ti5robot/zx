#include<mathfunc.h>
#include<iostream>
#include<time.h>
#include <tool.h>
using namespace std;	
class robotArm
{ 
	private:
		//float l[5]={189.5,300,115.5,184,95.5},width=131.5,rl1=45.5;//3kg
		//float l[5]={249,450,165.5,284,96},width=105,rl1=60;//5kg
		//float l[5]={137,190,65,125,65},width=112,rl1=30;//1kg
		//float l[5]={137,190,65,125,68},width=112,rl1=30;//NEW1kg
		float l[5]={158.6,260,85,174.5,87.5},width=128,rl1=34;//2kg
		float l15;
		int modea,modeb;
		float oriT06[4][4];
		float j2=pi-acos(l[0]/l[1]);
		float j3=pi/2+atan(2*sqrt(l[2]*l[3]/(width*width)));
		float P5[3],P6[3];
		float prcj=0.017;
		float scd=sin(prcj)*sin(prcj);
		float prc1=float(int(10000*sin(prcj)+1))/10000;
		float prc2=prc1*(prc1+2);
		float prc3=prc1*(3+prc1+prc1*prc1)+prc2;
		float s1,c1,s2,c2,c2j2,s3,c3,c2j3,t3,s23,c23,c2j23,s4,c4,s5,c5,s6,c6;
		bool checkjbypos(){
			bool token=false;
			j2Matrix();	
			if( abs(oriT06[0][0]-T06[0][0])<prc2 and abs(oriT06[0][1]-T06[0][1])<prc3 and abs(oriT06[0][2]-T06[0][2])<prc3 and abs(oriT06[0][3]-T06[0][3])<1 
			and abs(oriT06[1][0]-T06[1][0])<prc2 and abs(oriT06[1][1]-T06[1][1])<prc3 and abs(oriT06[1][2]-T06[1][2])<prc3 and abs(oriT06[1][3]-T06[1][3])<1
			and abs(oriT06[2][0]-T06[2][0])<prc1 and abs(oriT06[2][1]-T06[2][1])<prc2 and abs(oriT06[2][2]-T06[2][2])<prc2 and abs(oriT06[2][3]-T06[2][3])<1)
				token=true;
            		return token;
		};		
		bool checkcollision(){
			bool token=false;
			if(abs(j[1])>j2) token=true;
			else if(abs(j[2])>j3) token=true;
			return token;
		};	
		void j2Matrix(){
			T06[0][0]=((s1*s4 - c1*c4*c23)*s5 + s23*c1*c5)*c6 - (s1*c4 + s4*c1*c23)*s6;
			T06[0][1]=((-s1*s4 + c1*c4*c23)*s5 - s23*c1*c5)*s6 - (s1*c4 + s4*c1*c23)*c6;
			T06[0][2]=(s1*s4 - c1*c4*c23)*c5 - s5*s23*c1;
			T06[0][3]=l[4]*s1*s4*c5 + l[1]*s2*c1 - l[4]*s5*s23*c1 + (l[2]+l[3])*s23*c1 - l[4]*c1*c4*c5*c23;
			T06[1][0]=-((s1*c4*c23 + s4*c1)*s5 - s1*s23*c5)*c6 - (s1*s4*c23 - c1*c4)*s6;
			T06[1][1]=((s1*c4*c23 + s4*c1)*s5 - s1*s23*c5)*s6 - (s1*s4*c23 - c1*c4)*c6;	
            		T06[1][2]=-(s1*c4*c23 + s4*c1)*c5 - s1*s5*s23;
			T06[1][3]=l[1]*s1*s2 - l[4]*s1*s5*s23 + (l[2]+l[3])*s1*s23 - l[4]*s1*c4*c5*c23 - l[4]*s4*c1*c5;
			T06[2][0]=(s5*s23*c4 + c5*c23)*c6 + s4*s6*s23;
			T06[2][1]=-(s5*s23*c4 + c5*c23)*s6 + s4*s23*c6;
			T06[2][2]=-s5*c23 + s23*c4*c5;
			T06[2][3]=-l[4]*s5*c23 + l[4]*s23*c4*c5 + l[1]*c2 + (l[2]+l[3])*c23 + l[0];
			T06[3][0]=T06[3][1]=T06[3][2]=0;
			T06[3][3]=1;
		};
		int gotostep(int now,bool recall,bool dft[3]){
			int step;
			if(recall){
			    for(step=now;step>=0;step--){
				    if(dft[step]) break;
				    else dft[step]=true;
		            }
		            if(step>-1) dft[step]=false;
			}
			else step=now+1;
			return step;
		};
		bool Matrix2j(){
                        int step=0;
			bool dft[3]={true,true,true};
			float az15,a135,a513,d15[3]={P5[0],P5[1],P5[2]-l[0]},d56_3[3],X6_5[3];
			float n[3],v[3];
			az15=acos(d15[2]/l15);
                        a135=solve_trangle(l[1],l[2]+l[3],l15);
                        a513=solve_trangle(l15,l[1],l[2]+l[3]);
			Mcopy(T06,oriT06);
			while(-1<step and step<3){
				if(step==0){
					if(d15[0]==0 and d15[1]==0){
						j[0]=0;
						dft[step]=false;
						step++;
						continue;
					} 
					n[0]=d15[1],n[1]=-d15[0],n[2]=0;
					v[0]=0,v[1]=-1,v[2]=0;
					if(dft[step]){
						if(n[1]>0) n[0]=-n[0],n[1]=-n[1];
						j[0]=vec_angle(v,n,n[0]);
					}
					else{
						if(n[1]<0) n[0]=-n[0],n[1]=-n[1];
						j[0]=vec_angle(v,n,n[0]);
					}
				 	s1=sin(j[0]),c1=cos(j[0]);
					step++;	
				}
				else if(step==1){
					if(l15==rball){
						j[1]=j[2]=0;
						dft[step]=false;
						step++;
						continue;
					}
                    			if(P5[0]<0 and abs(j[0])<=pi/2) az15=-az15;
					else if(P5[0]>0 and abs(j[0])>pi/2) az15=-az15;
					if(dft[step]){
						j[1]=az15+a513;
                    				j[2]=pi-a135;
					}
					else{
                    				j[1]=az15-a513;
	                    			j[2]=a135-pi;
					}
					s2=sin(j[1]),c2=cos(j[1]),c2j2=cos(2*j[1]);
					s3=sin(j[2]),c3=cos(j[2]),c2j3=cos(2*j[2]),t3=s3/c3;
					s23=sin(j[1]-j[2]),c23=cos(j[1]-j[2]),c2j23=cos(2*j[1]-2*j[2]);
					step=gotostep(step,checkcollision(),dft);
				}
				else if(step==2){
					getd56_3(d56_3,P5,P6);
					if(d56_3[0]==0 and d56_3[2]==0){
						j[3]=0;
						step++;
						continue;
					}
					n[0]=-d56_3[2],n[1]=0,n[2]=d56_3[0];
					v[0]=0,v[1]=0,v[2]=1;
					if(dft[step]){
						if(n[2]<0) n[0]=-n[0],n[2]=-n[2];
						j[3]=vec_angle(v,n,n[0]);
					}
					else{
						if(n[2]>0) n[0]=-n[0],n[2]=-n[2];
                        			j[3]=vec_angle(v,n,n[0]);
					}
					v[0]=-n[2],v[1]=0,v[2]=n[0];
					j[4]=vec_angle(v,d56_3,-d56_3[1]);
					s4=sin(j[3]),c4=cos(j[3]);
					s5=sin(j[4]),c5=cos(j[4]);
					getX6_5(X6_5);
				        v[0]=1,v[1]=0,v[2]=0;
				        j[5]=vec_angle(v,X6_5,-X6_5[2]);
					s6=sin(j[5]),c6=cos(j[5]);
                    			step=gotostep(step,!checkjbypos(),dft);
				}
			}
			if(step==3) return true;
			else return false;		
		};
		void Pos2Matrix()
		{
			float r=pos[5],p=pos[4],y=pos[3];
			float 
			sp=sin(p),cp=cos(p),
			sy=sin(y),cy=cos(y),
			sr=sin(r),cr=cos(r);
			T06[0][0]=cp*cy;
			T06[0][1]=sp*sr*cy - sy*cr;
			T06[0][2]=sp*cr*cy + sr*sy;
			T06[0][3]=pos[0];
			T06[1][0]=sy*cp;
			T06[1][1]=sp*sr*sy + cr*cy;
			T06[1][2]=sp*sy*cr - sr*cy;
			T06[1][3]=pos[1];
			T06[2][0]=-sp;
			T06[2][1]=sr*cp;
			T06[2][2]=cp*cr;
			T06[2][3]=pos[2];
			T06[3][0]=T06[3][1]=T06[3][2]=0;
			T06[3][3]=1;
		};
		
		void getd56_3(float d56_3[3],float P5[3],float P6[3])
		{
			d56_3[0]=(P6[0]-P5[0])*c1*c23 + (P6[1]-P5[1])*s1*c23 + (P5[2]-P6[2])*s23;
			d56_3[1]=(P6[0]-P5[0])*s23*c1 + (P6[1]-P5[1])*s1*s23 + (P6[2]-P5[2])*c23;
			d56_3[2]=(P6[0]-P5[0])*s1 + (P5[1]-P6[1])*c1;
			
		};
		void getX6_5(float X6_5[3])//after T06
		{
			X6_5[0]=(oriT06[0][0] + oriT06[0][3])*(s1*s4*s5 - s2*s3*s5*c1*c4 + s2*c1*c3*c5 - s3*c1*c2*c5 - s5*c1*c2*c3*c4) + (oriT06[1][0] + oriT06[1][3])*(-s1*s2*s3*s5*c4 + s1*s2*c3*c5 - s1*s3*c2*c5 - s1*s5*c2*c3*c4 - s4*s5*c1) + (oriT06[2][0] + oriT06[2][3])*(s2*s5*c4 - s3*s5*c4*c23 + s3*s23*c5 + c2*c5)/c3 - l[0]*s2*s5*c4/c3 + l[1]*s3*s5*c4 + l[0]*s5*c4*c23*t3 - l[0]*s23*c5*t3 - l[0]*c2*c5/c3 - l[1]*c3*c5 - (l[2]+l[3])*c5;
			X6_5[1]=(oriT06[0][0] + oriT06[0][3])*(s1*s4*c5 - s5*s23*c1 - c1*c4*c5*c23) + (oriT06[1][0] + oriT06[1][3])*(-s1*s5*s23 - s1*c4*c5*c23 - s4*c1*c5) + (oriT06[2][0] + oriT06[2][3])*(-s5*c23 + s23*c4*c5) + l[1]*s3*c4*c5 + l[1]*s5*c3 + l[0]*s5*c23 + (l[2]+l[3])*s5 - l[0]*s23*c4*c5-l[4];
			X6_5[2]=(oriT06[0][0] + oriT06[0][3])*((c2j2 + c2j3 + c2j23 + 1)*s4*c1/4 + s1*c2*c3*c4)/(c2*c3) + (oriT06[1][0] + oriT06[1][3])*((c2j2 + c2j3 + c2j23 + 1)*s1*s4/4 - c1*c2*c3*c4)/(c2*c3) - (oriT06[2][0] + oriT06[2][3])*(s2 + sin(j[1] - 2*j[2]))*s4/(2*c3) + (-l[1]*s3 + l[0]*s23)*s4;
		};
		
		bool Matrix2Pos(){
                    bool solvable;
		    float k1=T06[0][0],k4=T06[1][0],k7=T06[2][0],k8=T06[2][1],k9=T06[2][2];
		    pos[0]=T06[0][3];
		    pos[1]=T06[1][3];
		    pos[2]=T06[2][3];
		    float r,p,y,sp,cp2,cp,cy,sy;
		    float a,b;
		    sp=-k7;cp2=1-sp*sp;
		    if(cp2>=scd){
		    	solvable=true;
			a=calcu_angle(k1*k8+k4*k9,k1*k9-k4*k8,cp2)+modea*pi;//r+y
			b=calcu_angle(k1*k8-k4*k9,k1*k9+k4*k8,cp2)+modeb*pi;//r-y
			r=(a+b)/2,y=(a-b)/2;
			sy=sin(y),cy=cos(y);
			if(abs(cy)>=scd) cp=k1/cy;
			else cp=k4/sy;
			p=calcu_angle(sp,cp,1);
			pos[5]=r;
			pos[4]=p;
			pos[3]=y;
		    }
		    else{
		        solvable=false;
			pos[4]=calcu_angle(-k7,0,1);
			pos[3]=pos[5]=0;
		    }
            	    return solvable;
		};
	
	public:
		float j[6]={0,0,0,0,0,0};
		int rball=l[1]+l[2]+l[3];
        	float pos[6];
        	float T06[4][4];
        	bool checkinputj(){
			for(int i=0;i<6;i++)
				if(abs(j[i])>pi) return false;
			if(!checkcollision()) return true;
			else return false;
		}
		bool checkinputpos(){
			bool token=false;
			if(pos[2]>=0){
				P6[0]=pos[0],P6[1]=pos[1],P6[2]=pos[2];
				float
				sy=sin(pos[3]),cy=cos(pos[3]),
				sp=sin(pos[4]),cp=cos(pos[4]),
				sr=sin(pos[5]),cr=cos(pos[5]);
				float d56[3],v[3]={sp*cr*cy + sr*sy,sp*sy*cr-sr*cy,cp*cr};
				vec_rescale(v,l[4],d56);
				P5[0]=P6[0]-d56[0],P5[1]=P6[1]-d56[1],P5[2]=P6[2]-d56[2];
				l15=sqrt(P5[0]*P5[0]+P5[1]*P5[1]+(P5[2]-l[0])*(P5[2]-l[0]));
				if(P5[2]>=0 and l15<=rball and l15>abs(l[1]-l[2]-l[3])) token=true;
				if(P5[2]<l[0]+rl1 and sqrt(P5[0]*P5[0]+P5[1]*P5[1])-l[4]-rl1<0) token=false;
			}
			return token;
		}
		bool forward_move()
		{
			modea=0,modeb=0;
			s1=sin(j[0]),c1=cos(j[0]);
			s2=sin(j[1]),c2=cos(j[1]);
			s3=sin(j[2]),c3=cos(j[2]);
			s4=sin(j[3]),c4=cos(j[3]);
			s5=sin(j[4]),c5=cos(j[4]);
			s6=sin(j[5]),c6=cos(j[5]);
			s23=sin(j[1]-j[2]),c23=cos(j[1]-j[2]);
			j2Matrix();
			return Matrix2Pos();
		};
		
                bool backward_move()
		{
			if(pos[5]+pos[3]>pi) modea=2;
			else if(pos[5]+pos[3]<-pi) modea=-2;
			else modea=0;
			if(pos[5]-pos[3]>pi) modeb=2;
			else if(pos[5]-pos[3]<-pi) modeb=-2;
			else modeb=0;
			Pos2Matrix();
			return Matrix2j();
		};
		void test(){
		    float start,end,avgtime=0;
		    int g=20,t=0,num=0;
		    float p1=0,p2=-2.2508,p3=0;
		    int n,solvj,i=0,r6=l[4]+rball;
		    float rate,s=0;    
		    float x,y,z;
		    for(p1=-3.14;p1<3.14;p1+=0.3){
	                for(p2=-3.1308;p2<3.13;p2+=0.3){
		            for(p3=-3.14;p3<3.14;p3+=0.3){
					   	solvj=0,n=0;
						for(z=1;z<l[0]+r6-1;z+=g){
						    for(x=-r6+1;x<r6-1;x+=g){
						     	for(y=-r6+1;y<r6-1;y+=g){
									pos[0]=x,pos[1]=y,pos[2]=z;
									pos[5]=p3,pos[4]=p2,pos[3]=p1;
									if(checkinputpos()){
										//start = clock();backward_move();end = clock();avgtime=((end-start)+t*avgtime)/(t+1);t++;
										n++;num++;if(backward_move()) solvj++;else if(checkcollision()) {n--;num--;}
									}
							    }
						    }
						}
						rate=float(solvj)/n;i++;s+=rate;
						//cout<<"rpy="<<p1<<" "<<p2<<" "<<p3<<". total points:"<<n<<".  solvable joints rate: "<<rate<<endl;
		             }
	                 }
		     }
		    //cout<<"avg time:"<<avgtime<<endl;
		    cout<<"total solvable samples:"<<num<<" mean solvable rate:"<<s/i<<endl;
		};	
};
