#include<math.h>
#define pi M_PI	
void vec_rescale(float v[3],float l,float nv[3])
{
	float k=l/sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); 
	nv[0]=v[0]*k,nv[1]=v[1]*k,nv[2]=v[2]*k;
};
float vec_length(float v[2]){
	return sqrt(v[0]*v[0]+v[1]*v[1]);
};
void vec2d_rescale(float v1[2],float l,float v2[2])
{
	float k=l/vec_length(v1);
	v2[0]=v1[0]*k,v2[1]=v1[1]*k;
};

float vec_angle(float v1[3],float v2[3],float f)
{
	float e1[3],e2[3];
	vec_rescale(v1,1,e1);
	vec_rescale(v2,1,e2);
	float angle=acos(e1[0]*e2[0]+e1[1]*e2[1]+e1[2]*e2[2]);
	if (f<0) angle=-angle;
	return angle;
};
float solve_trangle(float a,float b,float c)
{
	return acos((a*a+b*b-c*c)/(2*a*b));
};
void vec_cross(float v1[3],float v2[3],float v[3]){
	v[0]=v1[1]*v2[2]-v1[2]*v2[1];
	v[1]=v1[2]*v2[0]-v1[0]*v2[2];
	v[2]=v1[0]*v2[1]-v1[1]*v2[0];
}
float calcu_angle(float sa,float ca,float cp2){
    float a;
    if(abs(ca)<abs(sa)){
        a=acos(ca/cp2);
		a=sa<0?-a:a;//if near 0 then =0
    }
    else{
	sa/=cp2;
	a=asin(sa);
        if(ca<0) a=sa>0?pi-a:-pi-a;//[-pi,pi) 
    }
    return a;
};
