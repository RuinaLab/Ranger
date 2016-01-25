/*First implementation that uses own lu decomposition 
  12/14/2009*/
// Modified from heelstrikeMEXm.c in 20091019Zop 


/* [by MPK] 

[zdot,Ry] = singleMEX(time,state,parameters)

This function appears to take the state of the robot at time and compute the reaction forces and time derivatives.
there are 4 coefficients at the end of parameters that are used to compute the actuator effort at time. These 
five numbers are used to compute the actuator efforts at the hip and anke. Time is not used for anything else.
*/


#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#define nmat2 8 //should be equal to state space else error 10/12/2009
#define NMAT 12 //should be atleast nmat+1, here nmat = 6


void reshapeVEC2MAT(int , int ,int , double *, double *);
void reshapeMAT2VEC(int , int , double *, double *);
void zeros(int, int , double *);
void zeros3(int, int, int, double *);
void transpose(int , int , double *);
void inverse(int ,int ,double *,double *);
void multiplyMAT2MAT(int , int , int , int , double *, double *, double *);
void multiplyMAT2VEC(int , int , int , double *, double *, double *);
void subtract(int, int , double *, double *, double *);
void solve_ludecomp(int, double [][NMAT], double [], double []);
void ludecomp(int, double [][NMAT], int []);
void solve(int, double [][NMAT], double [], double [], int []);

//************************************************************
//
// THIS FUNCTION IS DESIGNED TO BE CALLED DIRECTLY FROM MATLAB
//
//*************************************************************

//
//    Gateway function is near bottom of code!
//


// The variables that are defined here are available to all functions defined in this file.   
static double M, m, I, l, c, w, r, d, g, k, gam, Jm, k_st, k_sw, C_st, C_sw;
static double R, V0, I0, G_h, G_a, K, I_MAX, e_r, mu_h, C1_h, C0_h, mu_st, C1_st, C0_st, C1m_st, C0m_st, ElectricalW, eps, eps_f;
static double coefs[100];
static double h0, h1, a0, a1, b0, b1;
static double t;

void single_stance(double *f, double *y, double *Ry)
{
//const int nmat = 8; //nmat should be equal to state space else error 10/12/2009
double AA[nmat2][nmat2], bb[nmat2], AAinv[nmat2][nmat2], udot[nmat2];
double q1, q2, q3, q4, q5, q6, u1, u2, u3, u4, u5, u6;
double xh, vx_h, yh, vy_h, xhddot, yhddot, ud1, ud2, ud3, ud4, ud5, ud6;
double Th, Ta, Tb, Thip, Ih, Ia, Ib, Vh, Va, Vb, Td_a, Td_b, Tatemp, Tbtemp, Tf_st;
double DE, E, val1, val2, val3; 
double fac1, fac2, fac3, Ph, Pa, Pb;
double Py;
double M11, M12, M13, M14;
double M21, M22, M23, M24;
double M31, M32, M33, M34;
double M41, M42, M43, M44;
double RHS1, RHS2, RHS3, RHS4;


q1    = y[0]   ;  
u1    = y[1]   ;  
q2    = y[2]   ;
u2    = y[3]   ;
q6    = y[4]   ;                          
u6    = y[5]   ; 
q3    = y[6]   ;                          
u3    = y[7]   ; 
q4    = y[8]   ;                          
u4    = y[9]   ; 
q5    = y[10]   ;                          
u5    = y[11]   ; 
E     = y[12]   ;
xh    = y[13]   ;  
vx_h  = y[14]   ;  
yh    = y[15]   ;
vy_h  = y[16]   ; 

Ih = h0 + h1*t;
Ia = a0 + a1*t;
Ib = 0;//b0 + b1*t;

Vh = Ih*(R+V0/sqrt(Ih*Ih+I0*I0)) + G_h*K*u3;
Va = Ia*(R+V0/sqrt(Ia*Ia+I0*I0)) + G_a*K*u6;
Vb = 0;

Th = G_h*K*Ih*(1-mu_h*tanh(eps_f*Ih*u3))-C1_h*u3-C0_h*tanh(eps_f*u3);
Ta = G_a*K*Ia*(1-mu_st*tanh(eps_f*Ia*u6))-C1m_st*u6-C0m_st*tanh(eps_f*u6);
Tb = 0;//G_a*K*Ib;

Ph = Vh*Ih;
Pa = Va*Ia;
Pb = 0;//Vb*Ib;

// Square root smoothing //
// Always positive //
//eps = 0.01;
fac1 = sqrt(Ph*Ph+eps*eps);
fac2 = sqrt(Pa*Pa+eps*eps);
fac3 = sqrt(Pb*Pb+eps*eps);
val1 = (1+e_r)*((Ph-fac1)/2) + fac1;
val2 = (1+e_r)*((Pa-fac2)/2) + fac2;
val3 = 0;//(1+e_r)*((Pb-fac3)/2) + fac3;

// Arc tangent smoothing //
// Goes a little negative //
/*       eps = 0.0001; %1e-4 to get about -1e-5
%       fac1 = (2/pi)*atan(Ph/eps);
%       fac2 = (2/pi)*atan(Pa/eps);
%       fac3 = (2/pi)*atan(Pb/eps);
%       val1 = (1+e_r)*((Ph/2)*(1-fac1)) + Ph*fac1;
%       val2 = (1+e_r)*((Pa/2)*(1-fac2)) + Pa*fac2;
%       val3 = (1+e_r)*((Pb/2)*(1-fac3)) + Pb*fac3;*/


DE = val1 + val2 + val3;

Thip = -k*q3; 
Td_a = -C_st*u1; //-C_st*u2;//
Td_b = 0;//-C_sw*u4;
Tf_st = -C1_st*u2-C0_st*tanh(eps_f*u2);
Tatemp = k_st*(q6-q2);
Tbtemp = 0;//k_sw*(q5-q4);

M11 = -4*m*r*l*cos(q1+q2)-2*m*r*w*sin(q1+q2)+4*m*r*d*cos(q1)+2*m*r*c*cos(q1+q2)-2*M*r*l*cos(q1+q2)+2*m*r*w*sin(q3-q1-q2)+2*m*r*c*cos(q3-q1-q2)+2*M*r*d*cos(q1)+2*m*r*r+2*m*d*c*cos(q3-q2)+2*m*d*w*sin(q3-q2)-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)+M*r*r+2*m*c*c-2*m*l*c+2*I-2*m*d*w*sin(q2)+2*m*d*c*cos(q2)-4*m*d*l*cos(q2)+2*m*w*w+M*l*l+2*m*d*d+2*m*l*l+M*d*d-2*M*d*l*cos(q2);
M12 = -2*m*c*l*cos(q3)-2*m*r*l*cos(q1+q2)-m*r*w*sin(q1+q2)+m*r*c*cos(q1+q2)-M*r*l*cos(q1+q2)+m*r*w*sin(q3-q1-q2)+m*r*c*cos(q3-q1-q2)+m*d*c*cos(q3-q2)-2*m*l*w*sin(q3)+m*d*w*sin(q3-q2)-2*m*d*l*cos(q2)+m*d*c*cos(q2)-m*d*w*sin(q2)+2*m*c*c-2*m*l*c+2*I+2*m*w*w+M*l*l+2*m*l*l-M*d*l*cos(q2);
M13 = 0;
M14 = -m*r*w*sin(q3-q1-q2)-m*r*c*cos(q3-q1-q2)-m*w*w-m*c*c-I+m*l*c*cos(q3)+m*l*w*sin(q3)-m*d*c*cos(q3-q2)-m*d*w*sin(q3-q2);
M21 = m*r*w*sin(q3-q1-q2)+m*r*c*cos(q3-q1-q2)+2*m*w*w+2*m*c*c+M*l*l-M*l*d*cos(q2)+2*m*l*l-M*l*cos(q1+q2)*r+2*I-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)+m*d*c*cos(q3-q2)+m*d*w*sin(q3-q2)-m*w*d*sin(q2)+m*c*d*cos(q2)-2*m*l*d*cos(q2)-2*m*l*cos(q1+q2)*r-m*w*sin(q1+q2)*r+m*c*cos(q1+q2)*r-2*m*l*c;
M22 = 2*m*w*w+2*m*c*c+M*l*l+2*m*l*l+2*I-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)-2*m*l*c;
M23 = 0;
M24 = -m*c*c-m*w*w+m*l*w*sin(q3)+m*l*c*cos(q3)-I;
M31 = 0;
M32 = 0;
M33 = 1;
M34 = 0;
M41 = -m*w*sin(q3-q1-q2)*r-m*c*c-m*w*w-m*c*cos(q3-q1-q2)*r-I+m*c*l*cos(q3)-m*w*d*sin(q3-q2)-m*c*d*cos(q3-q2)+m*w*l*sin(q3);
M42 = m*w*l*sin(q3)+m*c*l*cos(q3)-m*c*c-m*w*w-I;
M43 = 0;
M44 = m*w*w+m*c*c+I;
RHS1 = m*g*((d*sin(q1)-l*sin(q1+q2)+c*sin(q1+q2)+w*cos(q1+q2))*cos(gam)-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(q1+q2)-w*sin(q1+q2))*sin(gam))+m*g*((d*sin(q1)-l*sin(q1+q2)-c*sin(q3-q1-q2)+w*cos(q3-q1-q2))*cos(gam)-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(q3-q1-q2)+w*sin(q3-q1-q2))*sin(gam))+M*g*((d*sin(q1)-l*sin(q1+q2))*cos(gam)-(r+d*cos(q1)-l*cos(q1+q2))*sin(gam))-m*((-d*sin(q1)+l*sin(q1+q2)-c*sin(q1+q2)-w*cos(q1+q2))*(((-d*cos(q1)+l*cos(q1+q2))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2-(u1+u2)*(u1+u2)*(c*cos(q1+q2)-w*sin(q1+q2)))-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(q1+q2)-w*sin(q1+q2))*(((d*sin(q1)-l*sin(q1+q2))*u1-l*sin(q1+q2)*u2)*u1+(-l*sin(q1+q2)*u1-l*sin(q1+q2)*u2)*u2-(u1+u2)*(u1+u2)*(-c*sin(q1+q2)-w*cos(q1+q2))))-m*((-d*sin(q1)+l*sin(q1+q2)+c*sin(q3-q1-q2)-w*cos(q3-q1-q2))*(((-d*cos(q1)+l*cos(q1+q2))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2-(-u3+u1+u2)*(-u3+u1+u2)*(c*cos(q3-q1-q2)+w*sin(q3-q1-q2)))-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(q3-q1-q2)+w*sin(q3-q1-q2))*(((d*sin(q1)-l*sin(q1+q2))*u1-l*sin(q1+q2)*u2)*u1+(-l*sin(q1+q2)*u1-l*sin(q1+q2)*u2)*u2-(-u3+u1+u2)*(-u3+u1+u2)*(c*sin(q3-q1-q2)-w*cos(q3-q1-q2))))-M*((-d*sin(q1)+l*sin(q1+q2))*(((-d*cos(q1)+l*cos(q1+q2))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2)-(r+d*cos(q1)-l*cos(q1+q2))*(((d*sin(q1)-l*sin(q1+q2))*u1-l*sin(q1+q2)*u2)*u1+(-l*sin(q1+q2)*u1-l*sin(q1+q2)*u2)*u2));
RHS2 = -2*m*g*l*sin(-gam+q1+q2)+m*g*c*sin(-gam+q1+q2)+m*g*w*cos(-gam+q1+q2)-m*g*c*sin(gam+q3-q1-q2)+m*g*w*cos(gam+q3-q1-q2)-M*g*l*sin(-gam+q1+q2)+2*m*u1*u1*d*l*sin(q2)-m*u1*u1*d*c*sin(q2)-m*u1*u1*d*w*cos(q2)-m*w*u1*u1*d*cos(q3-q2)+m*l*u3*u3*c*sin(q3)-m*l*u3*u3*w*cos(q3)-2*m*l*u3*u2*c*sin(q3)+2*m*l*u3*u2*w*cos(q3)+m*c*u1*u1*d*sin(q3-q2)-2*m*l*u3*u1*c*sin(q3)+2*m*l*u3*u1*w*cos(q3)+M*l*u1*u1*d*sin(q2);
RHS3 = 0;
RHS4 = m*g*c*sin(gam+q3-q1-q2)-m*g*w*cos(gam+q3-q1-q2)-m*w*l*u1*u1*cos(q3)+m*c*l*u2*u2*sin(q3)-2*m*w*u1*l*u2*cos(q3)-m*w*l*u2*u2*cos(q3)+m*c*l*u1*u1*sin(q3)+m*w*d*u1*u1*cos(q3-q2)+2*m*c*u1*l*u2*sin(q3)-m*c*d*u1*u1*sin(q3-q2);

RHS1 = RHS1 + Td_a;
RHS2 = RHS2 + Tatemp + Tf_st;
RHS3 = RHS3 + (Ta - Tatemp)/Jm;
RHS4 = RHS4 + Thip + Th;

zeros(nmat2,nmat2,&AA[0][0]);
AA[0][0] = 1.0;
AA[2][2] = 1.0;
AA[4][4] = 1.0;
AA[6][6] = 1.0;

AA[1][1] = M11;
AA[1][3] = M12;
AA[1][5] = M13;
AA[1][7] = M14;
AA[3][1] = M21;
AA[3][3] = M22;
AA[3][5] = M23;
AA[3][7] = M24;
AA[5][1] = M31;
AA[5][3] = M32;
AA[5][5] = M33;
AA[5][7] = M34;
AA[7][1] = M41;
AA[7][3] = M42;
AA[7][5] = M43;
AA[7][7] = M44;

bb[0] = u1;
bb[1] = RHS1;
bb[2] = u2;
bb[3] = RHS2;
bb[4] = u6;
bb[5] = RHS3;
bb[6] = u3;
bb[7] = RHS4;

//This inverse matches with matlab inverse to 15 significant digits. 
inverse(nmat2,nmat2,&AA[0][0],&AAinv[0][0]);

multiplyMAT2VEC(nmat2,nmat2,nmat2,&AAinv[0][0],&bb[0],&udot[0]);


ud1 = udot[1];
ud2 = udot[3];
ud6 = udot[5];
ud3 = udot[7];
ud4 = 0;
ud5 = 0; 
Py= 0;
Ry[0] = m*(2*((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+2*(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2+2*(l*sin(q1+q2)-d*sin(q1))*ud1+2*l*sin(q1+q2)*ud2-(u1+u2)*(u1+u2)*(c*cos(q1+q2)-w*sin(q1+q2))+(ud1+ud2)*(-c*sin(q1+q2)-w*cos(q1+q2))-(-u3+u1+u2)*(-u3+u1+u2)*(c*cos(-q3+q1+q2)-w*sin(-q3+q1+q2))+(-ud3+ud1+ud2)*(-c*sin(-q3+q1+q2)-w*cos(-q3+q1+q2)))+M*(((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2+(l*sin(q1+q2)-d*sin(q1))*ud1+l*sin(q1+q2)*ud2)+(2*m+M)*g;

xhddot = (-l*sin(q1+q2)+d*sin(q1))*u1*u1-2*u1*l*sin(q1+q2)*u2+(l*cos(q1+q2)-d*cos(q1)-r)*ud1+l*cos(q1+q2)*ud2-l*sin(q1+q2)*u2*u2;
yhddot = (l*cos(q1+q2)-d*cos(q1))*u1*u1+2*u1*l*cos(q1+q2)*u2+(l*sin(q1+q2)-d*sin(q1))*ud1+l*sin(q1+q2)*ud2+l*cos(q1+q2)*u2*u2;


f[0] = u1;
f[1] = ud1;
f[2] = u2;
f[3] = ud2;
f[4] = u6;
f[5] = ud6;
f[6] = u3;
f[7] = ud3;
f[8] = u4;
f[9] = ud4;
f[10] = u5;
f[11] = ud5;
f[12] = DE;
f[13] = vx_h;
f[14] = xhddot;
f[15] = vy_h;
f[16] = yhddot;

}


/* 
 *  Gateway routine
 */

void mexFunction(   int nlhs, mxArray *plhs[],
					int nrhs, const mxArray *prhs[]		)
{

// Matlab Syntax:
//
//	[Zdot, Ry] = singleMEX(t,Z, params);
//




  double *z, *zdot, *Ry;
  unsigned int ms, ns;
  int i, j;
  double *paramsarray;
  double GL_DIM[16], GL_MOT[19];



  
  /* Get the input argument, x, which is a vector */
t = mxGetScalar(prhs[0]);  
  
ms = mxGetM(prhs[1]); //Get Rows of prhs, prhs[1] is z0
ns = mxGetN(prhs[1]); //Get Columns of prhs
z = mxGetPr(prhs[1]); // Get data elements of prhs 
 
paramsarray = mxGetPr(prhs[2]);


    // Retrieve parameters
    //parms = [GL_DIM GL_MOT coefs];

	/* Here is what is going on. paramsarray is the vector parms that is passed from Matlab.
	it consists of threee block vectors, being GL_DIM, GL_MOT, and coeff. The following three 
	for loops extract the elements of this vector into a vector for GL_DIM, a vector for GL_MOT,
	and four scalars for the motor coefficients. Note that each loop runs from i=0:N, where N is
	the length of the vector that is being written to. Note that there is a second index, j,
	that is indexing the vector that is being read from. Thus GL_DIM extracts the first 16 elements
	of paramsarray, and GL_MOT extracts the next 19, and then coeff extracts the remaining 4;
	THe coeff vector is then broken into four scalars that are used later.
	*/

    j = 0;
    for (i=0;i<=15;i++)
    {
        GL_DIM[i] = paramsarray[j];
        j = j + 1;
    }

    for (i=0;i<=18;i++)
    {
        GL_MOT[i] = paramsarray[j];
        j = j + 1;
    }

    //change depending on number of coefs coming in
    for (i=0;i<=3;i++)
    {
        coefs[i] = paramsarray[j];
        j = j + 1;
    }
    h0 = coefs[0]; h1 = coefs[1];
    a0 = coefs[2]; a1 = coefs[3];
    //b0 = coefs[4]; b1 = coefs[5];

//GL_DIM = [M m I l c w r d g k gam J k_st k_sw C_st C_sw];
    M = GL_DIM[0]; m = GL_DIM[1]; I = GL_DIM[2];
    l = GL_DIM[3]; c = GL_DIM[4]; w = GL_DIM[5];
    r = GL_DIM[6]; d = GL_DIM[7]; g = GL_DIM[8];
    k = GL_DIM[9]; gam = GL_DIM[10]; Jm = GL_DIM[11]; 
    k_st = GL_DIM[12]; k_sw = GL_DIM[13]; 
    C_st = GL_DIM[14]; C_sw = GL_DIM[15];
    
//GL_MOT = [R V0 I0 G_h G_a K I_MAX e_r mu_h C1_h C0_h mu_st C1_st C0_st C1m_st C0m_st ElectricalW eps];
    R = GL_MOT[0]; V0 = GL_MOT[1]; I0 = GL_MOT[2]; 
    G_h = GL_MOT[3]; G_a = GL_MOT[4]; K = GL_MOT[5]; 
    I_MAX= GL_MOT[6]; e_r = GL_MOT[7];
    mu_h = GL_MOT[8]; C1_h = GL_MOT[9]; C0_h = GL_MOT[10];
    mu_st = GL_MOT[11]; C1_st = GL_MOT[12]; C0_st = GL_MOT[13];
    C1m_st = GL_MOT[14]; C0m_st = GL_MOT[15];
    ElectricalW = GL_MOT[16]; eps = GL_MOT[17]; eps_f = GL_MOT[18];


 /* Create a matrix for the return argument */
 // Return is zdot 
  plhs[0] = mxCreateDoubleMatrix(ns, ms, mxREAL); //change to column vector
  plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
  
  /* Dereference arguments */
  zdot = mxGetPr(plhs[0]);
  Ry = mxGetPr(plhs[1]);
  
  /* call the computational routine */
  single_stance(zdot, z, Ry);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void zeros(int row, int col, double *Mat) //generate matrix full of zeros
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
	int i,j;
	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
				*(Mat + i*col+j) = 0.0;		
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void zeros3(int no, int row, int col, double *Mat) //generate matrix full of zeros
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
	int i,j,k;
	for (k=0;k<no;k++) 
		for (i=0;i<row;i++)
			for (j=0;j<col;j++)
				*(Mat + k*row*col + i*col+j) = 0.0;		
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void subtract(int row, int col, double *ptr_A, double *ptr_B, double *ptr_Difference) //subtract two matrices
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
int i, j;
double A[nmat2][nmat2], B[nmat2][nmat2]; //double A[row][col], B[row][col];

	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
		{ A[i][j] = *(ptr_A + i*col+j); B[i][j] = *(ptr_B + i*col+j);}		

	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
			*(ptr_Difference + i*col+j) = A[i][j] - B[i][j];
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void multiplyMAT2MAT(int row_A, int col_A, int row_B, int col_B, double *ptr_A, double *ptr_B, double *ptr_Multiply) //Multiply two matrices
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
int i, j, p;
double A[nmat2][nmat2], B[nmat2][nmat2], sum; //double A[row_A][col_A], B[row_B][col_B], sum;

if (col_A != row_B)
	{ printf("ERROR: Columns of A should equal rows of B \n"); exit(1); }

	for (i=0;i<row_A;i++)
		for (j=0;j<col_A;j++)
			A[i][j] = *(ptr_A + i*col_A+j);
		
	for (i=0;i<row_B;i++)
		for (j=0;j<col_B;j++)
				 B[i][j] = *(ptr_B + i*col_B+j);		

	for (i=0;i<row_A;i++)
		for (j=0;j<col_B;j++)
		{
			sum = 0;
			for (p=0;p<col_A;p++)
				sum = sum + A[i][p]*B[p][j];
			
			*(ptr_Multiply + i*col_B+j) = sum;
		}

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void multiplyMAT2VEC(int row_A, int col_A, int row_b, double *ptr_A, double *ptr_b, double *ptr_x) //Multiply matrix to vector
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
int i, j, p;
double A[nmat2][nmat2], b[nmat2], sum; //double A[row_A][col_A], b[row_b], sum;

if (col_A != row_b)
	{ printf("ERROR: Columns of A should equal rows of b \n"); exit(1); }

	for (i=0;i<row_A;i++)
		for (j=0;j<col_A;j++)
			A[i][j] = *(ptr_A + i*col_A+j);
	
	for (i=0;i<row_b;i++)
			b[i] = *(ptr_b + i);		

	for (i=0;i<row_A;i++)
		{
			sum = 0.0;
			for (p=0;p<col_A;p++)
				sum = sum + A[i][p]*b[p];
			
			*(ptr_x + i) = sum;
		}

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void transpose(int row, int col, double *ptr_A) //Take transpose of a matrix
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
	int i,j;
	double A[nmat2][nmat2]; //double A[col][row]; //Is the transpose
	
	if (row != col)
	{ printf("ERROR: Transpose currently works only on square matrices \n"); exit(1); }
	
	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
			A[j][i] = *(ptr_A + i*col+j); //get the transpose
			
	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
			*(ptr_A + i*col+j) = A[i][j]; //stash back and send
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void reshapeMAT2VEC(int row,int col, double *ptr_A, double *ptr_a) //Matrix to vector
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
	int i,j;
	double A[nmat2][nmat2]; //double A[col][row];
	
	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
			A[j][i] = *(ptr_A + i*col+j); //get the transpose
			
	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
			*(ptr_a + i*col+j) = A[i][j]; //stash back and send

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void reshapeVEC2MAT(int row_a, int row_A,int col_A, double *ptr_a, double *ptr_A) //Vector to a matrix
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
int i, j, k;
double A[nmat2][nmat2]; //double A[col_A][row_A]; //We are going to first assemble the transpose

if (row_a != row_A*col_A)
	{ printf("ERROR: Checks size of input matrices \n"); exit(1); }
	
	k = 0;
	for (i=0;i<row_A;i++)
		for (j=0;j<col_A;j++)
		{		
			A[j][i] = *(ptr_a + k); //get the transpose
			k = k + 1;
		}
		
	for (i=0;i<row_A;i++)
		for (j=0;j<col_A;j++)
			*(ptr_A + i*col_A+j) = A[i][j]; //stash back and send
	
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void inverse(int row,int col,double *ptr_A,double *ptr_Ainv) //invert matrix
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
{
int i, j, p,q;
double A[NMAT][NMAT], x[NMAT], b[NMAT]; //should be NMAT else will give error
double Ainv[nmat2][nmat2];  //double Ainv[row][col];

if (row != col)
	{ printf("ERROR: Matrix is not square and cannot be inverted \n"); exit(1); }

	for (i=1;i<=row;i++)
	{
			for (j=1;j<=row;j++)	
				{
				x[j] = 0;
			    if (i==j) b[j] = 1.0; else b[j] = 0.0;
				}
		   
		   	for (p=0;p<row;p++)
				for (q=0;q<col;q++)
					A[p+1][q+1] = *(ptr_A + p*col+q);	//Matrix starts from A[1][1]...
					
			solve_ludecomp(row, A, b, x);
			for (j=1;j<=row;j++)
				Ainv[i-1][j-1] = x[j];
	}		 
			
	transpose(row,col,&Ainv[0][0]);
	
	for (i=0;i<row;i++)
		for (j=0;j<col;j++)
			*(ptr_Ainv + i*row+j) = Ainv[i][j];
	
}

////////////////////////////////////////////////
//////////// Solve Ax = b //////////////////////
///////////////////////////////////////////////

/*
  Taken from http://comp.cs.ehime-u.ac.jp/~ogata/nac/index.html
  solve a linear equation Ax = b 
  by a LU decomposition PA = LU (with partial pivoting), 
  where L is a lower triangular matrix, 
  U is a upper triangular matrix and 
  P is a permutation matrix, 
  and estimate the condition number of A 
  by Natori-Tsukamoto's method
  
  copyright : Hidenori Ogata, 6 April 2004, ver.1
                             12 April 2004, ver.1.01
  
  (revisions)
  ver.1.01: Mistakes in the comment here are corrected.
  
  (usage)
  solve_ludecomp(nmat, a, b, x);
  ... if the condition number is not needed
  solve_cond_ludecomp(nmat, a, b, x, &cond);
  ... if the condition number is needed
  
  The parameters are as follows.
  (input)
  nmat    int             : the matrix size (must be < NMAT)
  a       double [][NMAT] : the coefficient matrix
  b       double []       : the r.h.s. vector
  (output)
  x       double []       : the solution
  cond    double *        : the condition number of A
  (remark)
  After the computation, 
  the elements of L are stored in the lower triangular part of the array 
  a[i][j], i > j, and 
  the elements of U are stored in the upper triangular part of the array 
  a[i][j], i <= j. 
  
*/

/*
  solve a linear equation Ax = b by the Gauss elimination method 
  with partial pivotting
  (input)
  nmat    int             : the matrix size (must be < NMAT)
  a       double [][NMAT] : the coefficient matrix
  b       double []       : the r.h.s. vector
  (output)
  x       double []       : the solution
  (remark)
  After the computation by the function "ludecomp", 
  the elements of L are stored in the lower triangular part of the array 
  a[i][j], i > j, and 
  the elements of U are stored in the upper triangular part of the array 
  a[i][j], i <= j. 
*/
void solve_ludecomp(int nmat, double a[][NMAT], double b[], double x[])
{
  int i, j;
  int p[NMAT];	/* index vector for partial pivotting */
  double a0[NMAT][NMAT];
  
  if (nmat >= NMAT)
    {
      mexPrintf("nmat (the matrix size)=%d\n", nmat);
      mexPrintf("nmat must be <= %d\n", NMAT-1);
      exit(0);
    }
  
  ludecomp(nmat, a, p);
  
  solve(nmat, a, b, x, p);
}
/*
  LU decomposition of the matrix A
  with partial pivotting
  
  (remark)
  After the computation by the function "ludecomp", 
  the elements of L are stored in the lower triangular part of the array 
  a[p[i]][j], i > j, and 
  the elements of U are stored in the upper triangular part of the array 
  a[p[i]][j], i <= j. 
*/
void ludecomp(int nmat, double a[][NMAT], int p[])
{
  int i, j, k, k1, l, kk;
  double akmax;
  
  for (i=1; i<=nmat; ++i)
    p[i] = i;
	
  /*
    the Gauss elimination 
  */
  for (k=1; k<=nmat-1; ++k)
    {
      k1 = k + 1;
      /*
	partial pivotting
      */
      l = k;
      akmax = fabs(a[p[k]][k]);
      for (i=k1; i<=nmat; ++i)
	if (fabs(a[p[i]][k]) > akmax)
	  {
	    l = i;
	    akmax = fabs(a[p[i]][k]);
	  }
      if (l > k)
	{
	  kk = p[k];
	  p[k] = p[l];
	  p[l] = kk;
	}
      
      for (i=k1; i<=nmat; ++i)
	{
	  if (a[p[k]][k] != 0.0)
	    {
	      a[p[i]][k] /= a[p[k]][k];
	      for (j=k1; j<=nmat; ++j)
		a[p[i]][j] -= a[p[i]][k] * a[p[k]][j];
	    }
	  else
	    {
	      mexPrintf("Step %2d\n", k);
	      mexPrintf("The (%2d,%2d)-element of the matrix is zero.\n", k, k);
	      exit(0);
	    }
	}
    }
}

/*
  solve the equation Ax = b 
  by solving the simultaneous linear equations 
  Ly = Pb, Ux = y.
*/
void solve(int nmat, double a[][NMAT], double b[], double x[], int p[])
{
  double y[NMAT];
  int i, j;
  /*
    solve Ly = Pb by forward substitution
  */
  for (i=1; i<=nmat; ++i)
    {
      y[i] = b[p[i]];
      for (j=1; j<=i-1; ++j)
	y[i] -= a[p[i]][j] * y[j];
    }
  /*
    solve Ux = y by backward substitution 
  */
  for (i=nmat; i>=1; --i)
    {
      x[i] = y[i];
      for (j=nmat; j>=i+1; --j)
        x[i] -= a[p[i]][j] * x[j];
      x[i] /= a[p[i]][i];
    }
}