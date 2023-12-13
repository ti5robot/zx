#include<iostream>
#include<string.h>
#include <termio.h>
using namespace std;
#define IDNUM 6
void Mcopy(float (*C)[4],float (*P)[4]){
    P[0][0]=C[0][0];
    P[0][1]=C[0][1];
    P[0][2]=C[0][2];
    P[0][3]=C[0][3];
    P[1][0]=C[1][0];
    P[1][1]=C[1][1];
    P[1][2]=C[1][2];
    P[1][3]=C[1][3];
    P[2][0]=C[2][0];
    P[2][1]=C[2][1];
    P[2][2]=C[2][2];
    P[2][3]=C[2][3];
}
void show_value(string name,float (*T)[4]){
    cout<<name<<endl;
    for(int i=0;i<3;i++){
		for(int k=0;k<4;k++)
		    cout<<T[i][k]<<" ";
		cout<<endl;
    }
    cout<<endl;
}
void copy_value(float* copy,float* paste){
    for(int i=0;i<IDNUM;i++)
        paste[i]=copy[i];
};
void show_value(string name,float* value){
    cout<<name<<endl;
    for(int i=0;i<IDNUM;i++)
        cout<<value[i]<<" ";
    cout<<endl;
};
void show_value(string name,uint32_t* value){
    cout<<name<<endl;
    for(int i=0;i<IDNUM;i++)
        cout<<int(value[i])<<" ";
    cout<<endl;
};

void get_cmdlist(uint8_t*L,uint8_t c){
    for(int i=0;i<IDNUM;i++) L[i]=c;
};
void get_canidlist(uint8_t*cL){
	cout<<IDNUM<<endl;
    for(int i=0;i<IDNUM;i++) cL[i]=i+1;
};
void get_paralist(uint32_t* L,uint32_t c){
        for(int i=0;i<IDNUM;i++) L[i]=c;
};
int getch()//nonblock
{ 
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt; 
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  newt.c_cc[VMIN] = 0; 
  newt.c_cc[VTIME] = 0;
  
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings 
  int c = getchar();  // read character (non-blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
 }

char scanKeyboard()//block
{
    struct termios stored_settings,new_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    
    tcgetattr(0,&stored_settings);
    
    tcsetattr(0,TCSANOW,&new_settings);
    char in = getchar();
    tcsetattr(0,TCSANOW,&stored_settings);
    return in; 
};
