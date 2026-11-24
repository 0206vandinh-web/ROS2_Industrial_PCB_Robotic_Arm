#include "kin.h"
#include "dh_values.h"
#include <math.h>
#include <string.h>
static void ident(double *t){memset(t,0,16*sizeof(double)); for(int i=0;i<4;i++)t[5*i]=1;}
static void mul(double *a,const double *b){
 double v[16]={0};
 for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)v[4*i+j]+=a[4*i+k]*b[4*k+j];
 memcpy(a,v,sizeof(v));
}
static void chain(const double *q,double *t,double J[6][6]){
 double axis[6][3],p[6][3];ident(t);
 for(int i=0;i<6;i++){
  for(int k=0;k<3;k++){axis[i][k]=t[k*4+2];p[i][k]=t[k*4+3];}
  double c=cos(q[i]),s=sin(q[i]),ca=cos(ALPHA[i]),sa=sin(ALPHA[i]);
  double h[16]={c,-s*ca,s*sa,A[i]*c,s,c*ca,-c*sa,A[i]*s,0,sa,ca,D[i],0,0,0,1};mul(t,h);
 }
 for(int i=0;i<6;i++)for(int k=0;k<3;k++){
  int u=(k+1)%3,v=(k+2)%3;
  J[k][i]=axis[i][u]*(t[4*v+3]-p[i][v])-axis[i][v]*(t[4*u+3]-p[i][u]);
  J[k+3][i]=axis[i][k];
 }
}
void c3_fk(const double q[6],double t[16]){double j[6][6];chain(q,t,j);}
static void gram(double j[6][6],double g[6][6]){
 for(int i=0;i<6;i++)for(int k=0;k<6;k++){g[i][k]=0;for(int r=0;r<6;r++)g[i][k]+=j[i][r]*j[k][r];}
}
double c3_sigma(const double q[6]){
 double t[16],j[6][6],g[6][6];chain(q,t,j);gram(j,g);
 for(int n=0;n<100;n++){
  int p=0,r=1;
  for(int i=0;i<6;i++)for(int k=i+1;k<6;k++)if(fabs(g[i][k])>fabs(g[p][r])){p=i;r=k;}
  if(fabs(g[p][r])<1e-12)break;
  double a=.5*atan2(2*g[p][r],g[r][r]-g[p][p]),c=cos(a),s=sin(a);
  for(int i=0;i<6;i++){double x=g[p][i],y=g[r][i];g[p][i]=c*x-s*y;g[r][i]=s*x+c*y;}
  for(int i=0;i<6;i++){double x=g[i][p],y=g[i][r];g[i][p]=c*x-s*y;g[i][r]=s*x+c*y;}
 }
 double m=g[0][0];for(int i=1;i<6;i++)m=fmin(m,g[i][i]);return sqrt(fmax(0,m));
}
int c3_ik(const double dest[16],const double seed[6],double out[6]){
 memcpy(out,seed,6*sizeof(double));
 for(int n=0;n<240;n++){
  double t[16],j[6][6],g[6][6],e[6]={0};chain(out,t,j);gram(j,g);
  double trace=0;for(int k=0;k<3;k++)for(int b=0;b<3;b++)trace+=dest[4*k+b]*t[4*k+b];
  for(int k=0;k<3;k++){e[k]=dest[4*k+3]-t[4*k+3];int u=(k+1)%3,v=(k+2)%3;
   for(int b=0;b<3;b++)e[k+3]+=.5*(t[4*u+b]*dest[4*v+b]-t[4*v+b]*dest[4*u+b]);}
  if(hypot(hypot(e[0],e[1]),e[2])<.000012 && trace>2.99999995)return 1;
  /* Damped least squares; positive definite Gram matrix after damping. */
  for(int i=0;i<6;i++)g[i][i]+=.000025;
  for(int i=0;i<6;i++){double pivot=g[i][i];for(int k=i;k<6;k++)g[i][k]/=pivot;e[i]/=pivot;
   for(int r=0;r<6;r++)if(r!=i){double f=g[r][i];for(int k=i;k<6;k++)g[r][k]-=f*g[i][k];e[r]-=f*e[i];}}
  for(int i=0;i<6;i++){double step=0;for(int r=0;r<6;r++)step+=j[r][i]*e[r];out[i]=fmax(-LIMIT[i],fmin(LIMIT[i],out[i]+fmax(-.18,fmin(.18,step))));}
 }
 return 0;
}
