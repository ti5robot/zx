#ifndef ROBOTMOVE_H
#define ROBOTMOVE_H


#include <pos_trans.h>
#include <communication.h>
#include "SingleCaninterface.h"
#include "motortypehelper.h"
#define NMAX 1000
#define USLEEPTIME 3000
int AT=25;
float AG=0.03;//>=4*USLEEPTIME
float TG=AT*AG;
float scale=101;
float n2p=655.36;
float j2p=scale*65536/2/pi;
class robotArm TH;
float bais[IDNUM]={0,0,0,0,0,0};
float min_time=0;
uint8_t canidList[IDNUM];
void setn(float npL[IDNUM]){
    //uint8_t ASC[3*BUF_SIZE]={0};
    uint32_t unpL1[IDNUM],unpL2[IDNUM];
    for(int i=0;i<IDNUM;i++){
	unpL1[i]=abs(int(npL[i]));
	unpL2[i]=-unpL1[i];
    }
    SCanInterface *canInterface =SCanInterface::getInstance();
    canInterface->setParameter(canidList,unpL1, MotorTypeHelper::REG_MAX_APP_SPEED,IDNUM);
    canInterface->setParameter(canidList,unpL2, MotorTypeHelper::REG_MIN_APP_SPEED,IDNUM);
};
void movebyn(float npL[IDNUM])
{
    uint32_t unpL[IDNUM];
    for(int i=0;i<IDNUM;i++) unpL[i]=int(npL[i]);
    SCanInterface *canInterface =SCanInterface::getInstance();
    canInterface->setParameter(canidList,unpL, MotorTypeHelper::REG_TARGET_SPEED,IDNUM);
}

void ACTmove(float tm,int mode,float nparameterList[][IDNUM]){
	int i,t;
	//for(t=0;t<AT;t++) show_value("nlist",nparameterList[t]);
	uint32_t uspL[IDNUM];
        //uint8_t ASC[3*BUF_SIZE]={0};
        float npl[IDNUM];
	for(i=0;i<IDNUM;i++) npl[i]=NMAX;
	setn(npl);cout<<"set NMAX"<<endl;
        for(i=0;i<IDNUM;i++) {uspL[i]=int((TH.j[i]+bais[i])*j2p);}
        SCanInterface *canInterface =SCanInterface::getInstance();
	if(mode>0){
		cout<<"tm:"<<tm<<endl;
		if(mode==2 and tm>0){
			for(t=0;t<AT-1;t++){
		                cout<<"1111"<<endl;
				movebyn(nparameterList[t]);//nmax不重置，速度环也会受到限制
		                cout<<"2222"<<endl;
				usleep(1000000*AG-USLEEPTIME);
			}
		                cout<<"3333"<<endl;
			movebyn(nparameterList[t]);
		                cout<<"4444"<<endl;
			usleep(1000000*tm-USLEEPTIME);
			for(t=AT-2;t>0;t--){
				movebyn(nparameterList[t]);
				usleep(1000000*AG-USLEEPTIME);
			}
			movebyn(nparameterList[0]);		
		}
		setn(nparameterList[0]);
	}
	canInterface->setParameter(canidList,uspL, MotorTypeHelper::REG_TARGET_POSITION,IDNUM);
}
void UPRmove(float tm,int mode,float nparameterList[][IDNUM]){
	int t;int lmsg;
	float msg[1024];
	if(tm>0){
		msg[0]=1;int j=1;
		for(t=0;t<AT-1;t++){
			for(int i=0;i<IDNUM;i++) msg[j++]=nparameterList[t][i]*3.6/101;msg[j++]=AG;
		}
		for(int i=0;i<IDNUM;i++) msg[j++]=nparameterList[t][i]*3.6/101;msg[j++]=tm;
		for(t=AT-2;t>-1;t--){
			for(int i=0;i<IDNUM;i++) msg[j++]=nparameterList[t][i]*3.6/101;msg[j++]=AG;
		}
		lmsg=1+(2*AT-1)*(IDNUM+1);
	}else if(mode==1 or mode==2){
		msg[0]=0;
		for(int i=0;i<IDNUM;i++) msg[i+1]=nparameterList[0][i]*3.6/101;
		lmsg=IDNUM+1;
	}else{
		msg[0]=0;for(int i=0;i<IDNUM;i++) msg[i+1]=NMAX*3.6/101;
		lmsg=IDNUM+1;
	}
	for(int i=0;i<IDNUM;i++) msg[lmsg+i]=TH.j[i]*180/pi;
	lmsg+=6;
	send(clientFd,msg,4*lmsg,0);
}
/*
float calcuC(int nmax,float k){
        return k*TG*TG*TG/3+n2p*nmax*TG;
}
int calcustep(float t,float k,int nmax,float C){
        return k*t*t*t/3+n2p*nmax*t+C;
}
*/
float calcuk(int nmax){
	return -n2p*nmax/TG/TG;
}
float calcuvmax(int gap,float tm){
	return gap/(tm+(4*AT+1)*(AT-1)*AG/3/AT);
}
float steppersecond(float t,float k,int nmax){
	return k*t*t+n2p*nmax;
}
void plan_move(int mode){
        float curt_jv[IDNUM];
        uint32_t parameterList[IDNUM];
	float nparameterList[AT][IDNUM];
	int p=0,gap[IDNUM];
	float C,k,tm=0;
	int i,t,im=0;
	get_canidlist(canidList);
	SCanInterface *canInterface =SCanInterface::getInstance();
	if(mode!=0){
		if(canInterface->getParameter(canidList, parameterList, MotorTypeHelper::REG_CURRENT_POSITION,IDNUM)){
		    	for(int i=0;i<IDNUM;i++) curt_jv[i]=int(parameterList[i])/j2p-bais[i];//jv of math model
		    	//if(getCommand(parameterList,4)) show_value("I:",parameterList);
			//for(int i=0;i<IDNUM;i++) curt_jv[i]=0;
			for(i=0;i<IDNUM;i++){
				gap[i]=(TH.j[i]-curt_jv[i])*j2p;
			}
			for(i=1;i<IDNUM;i++) if(abs(gap[i])>abs(gap[im])) im=i;
			if(mode==1){
				float t=abs(gap[im])/NMAX/n2p;
				if(t<min_time) t=min_time;
				for(i=0;i<IDNUM;i++) nparameterList[0][i]=gap[i]/t/n2p;
			}
			else if(mode==2){
				nparameterList[AT-1][im]=gap[im]>0?NMAX:-NMAX;
				k=calcuk(nparameterList[AT-1][im]);
				for(t=0;t<AT-1;t++){
					nparameterList[t][im]=steppersecond(AG*(t-AT+1),k,nparameterList[AT-1][im])/n2p;
					p+=n2p*nparameterList[t][im]*AG;
				}
				tm=(gap[im]-2*p)/nparameterList[AT-1][im]/n2p;
				if(tm>0){
					//cout<<"tm:"<<tm<<endl;
					for(i=0;i<IDNUM;i++){
						if(i==im) continue;
						nparameterList[AT-1][i]=calcuvmax(gap[i],tm)/n2p;
						k=calcuk(nparameterList[AT-1][i]);
						for(t=0;t<AT-1;t++){
							nparameterList[t][i]=steppersecond(AG*(t-AT+1),k,nparameterList[AT-1][i])/n2p;
						}
					}
				}
				else for(i=0;i<IDNUM;i++) nparameterList[0][i]=float(gap[i])/2/TG/n2p;
			}
		}
		else cout<<"recieve curt_jv failed."<<endl;
	}
	UPRmove(tm,mode,nparameterList);
	ACTmove(tm,mode,nparameterList);
};

bool move_to_joint(int mode){
    if(TH.checkinputj()){
    	plan_move(mode);
    	//show_value("calcu pos:",TH.pos);
    	if(TH.forward_move()){
    		show_value("calcu pos:",TH.pos);
    	}
    	else cout<<"p="<<TH.pos[4]<<" and"<<(TH.pos[4]>0?"r-y ":"r+y ")<<"should equal:"<<(TH.pos[4]>0?calcu_angle(TH.T06[0][1],TH.T06[0][2],1):calcu_angle(-TH.T06[0][1],-TH.T06[0][2],1))<<endl;
	return true;
    }
    else{
	cout<<"illegal j."<<endl;show_value("j:",TH.j);
	return false;
    }

};
bool move_to_pos(int mode){
    if(TH.checkinputpos()){
	if(TH.backward_move()){
		//show_value("calcu j:",TH.j);
		plan_move(mode);
	}
	//else show_value("unsolvable pos: ",TH.pos);
	return true;
    }
    else{
	cout<<"illegal pos."<<endl;show_value("pos:",TH.pos);
	return false;
    }
};

#endif
