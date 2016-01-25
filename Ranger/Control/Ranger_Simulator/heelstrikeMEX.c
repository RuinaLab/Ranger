/*First implementation that uses own lu decomposition 
  12/14/2009*/
// Modified from heelstrikeMEXm.c in 20091019Zop 

#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#define nmat2 14
#define NMAT 18 //should be atleast nmat+1, here nmat = 6

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


//GLOBAL variables...  
double M, m, I, l, c, w, r, d, g, k, gam, J, k_st, k_sw, C_st, C_sw;    //GLOBAL variables...

void heelstrike(double *xplus, double *x)
{

double q1, u1, q2, u2, q3, u3, q4, u4, q5, u5, q6, u6;
double r1, v1, r2, v2, r3, v3, r4, v4, r5, v5, r6, v6;

//int nmat = 14; //nmat should be equal to state space else error 10/12/2009
double AA[nmat2][nmat2], AAinv[nmat2][nmat2], bb[nmat2], xdot[nmat2];
double M11, M12, M13, M14, M15, M16, M17, M18;
double M21, M22, M23, M24, M25, M26, M27, M28;
double M31, M32, M33, M34, M35, M36, M37, M38;
double M41, M42, M43, M44, M45, M46, M47, M48;
double M51, M52, M53, M54, M55, M56, M57, M58;
double M61, M62, M63, M64, M65, M66, M67, M68;
double M71, M72, M73, M74, M75, M76, M77, M78;
double M81, M82, M83, M84, M85, M86, M87, M88;
double RHS1, RHS2, RHS3, RHS4, RHS5, RHS6, RHS7, RHS8;

r1    = x[0]   ;	// stance foot abs angle
v1    = x[1]   ;	// stance foot abs rate
r2    = x[2]   ;	// stance foot joint angle
v2    = x[3]   ;	// stance foot joint rate
r6    = x[4]   ;    // stance foot mtor angle                    
v6    = x[5]   ;    // stance foot motr rate
r3    = x[6]   ;    // Hip Angle
v3    = x[7]   ;    // HIp Rate
r4    = x[8]   ;    // Swing Foot Joint Angle
v4    = x[9]   ;    // Swing foot joint rate
r5    = x[10]  ;    // Swing foot motor angle
v5    = x[11]  ;    // Swing foot motor rate

q1 = r1 + r2 - r3 - r4;
q2 = r4;
q3 = -r3;
q4 = r2;
q5 = r6; //not equal to r2, swing leg motor becomes stance leg motor
q6 = r5; //=r4 which is equal to q2 (above). stance leg motor becomes swing leg motor

M11 = 2*m*r*r-2*m*r*w*sin(-q3+q1+q2)+2*m*r*c*cos(-q3+q1+q2)+2*M*r*d*cos(q1)-2*M*r*l*cos(q1+q2)-2*m*r*w*sin(q1+q2)+2*m*r*c*cos(q1+q2)-4*m*r*l*cos(q1+q2)+4*m*r*d*cos(q1)-2*m*l*c-2*m*d*w*sin(-q3+q2)-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)+2*m*d*c*cos(-q3+q2)+2*I+M*d*d-2*m*d*w*sin(q2)+2*m*d*c*cos(q2)-4*m*d*l*cos(q2)+M*r*r+2*m*l*l+2*m*d*d+M*l*l+2*m*c*c+2*m*w*w-2*M*d*l*cos(q2);
M12 = -m*r*w*sin(-q3+q1+q2)+m*r*c*cos(-q3+q1+q2)-M*r*l*cos(q1+q2)-m*r*w*sin(q1+q2)+m*r*c*cos(q1+q2)-2*m*r*l*cos(q1+q2)-2*m*l*c*cos(q3)-m*d*w*sin(-q3+q2)-M*l*d*cos(q2)-2*m*l*c-2*m*l*w*sin(q3)+m*d*c*cos(-q3+q2)+2*I+2*m*l*l+M*l*l-m*d*w*sin(q2)+2*m*c*c+2*m*w*w-2*m*d*l*cos(q2)+m*d*c*cos(q2);
M13 = 0;
M14 = m*l*c*cos(q3)+m*d*w*sin(-q3+q2)+m*l*w*sin(q3)-m*d*c*cos(-q3+q2)-m*r*c*cos(-q3+q1+q2)+m*r*w*sin(-q3+q1+q2)-m*w*w-m*c*c-I;
M15 = 0;
M16 = 0;
M17 = -d*cos(q1)+l*cos(q1+q2)+d*cos(-q3+q1+q2-q4)-l*cos(-q3+q1+q2);
M18 = -d*sin(q1)+l*sin(q1+q2)+d*sin(-q3+q1+q2-q4)-l*sin(-q3+q1+q2);
M21 = m*c*cos(-q3+q1+q2)*r+2*m*c*c-m*d*w*sin(-q3+q2)-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)+m*d*c*cos(-q3+q2)+2*I-M*l*d*cos(q2)+2*m*w*w-M*l*cos(q1+q2)*r-m*w*sin(-q3+q1+q2)*r-2*m*l*cos(q1+q2)*r-m*w*sin(q1+q2)*r+m*c*cos(q1+q2)*r+2*m*l*l+M*l*l-2*m*l*c-2*m*d*l*cos(q2)+m*d*c*cos(q2)-m*d*w*sin(q2);
M22 = 2*m*l*l-2*m*l*c+2*m*c*c+2*m*w*w-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)+M*l*l+2*I;
M23 = 0;
M24 = m*l*c*cos(q3)+m*l*w*sin(q3)-m*c*c-m*w*w-I;
M25 = 0;
M26 = 0;
M27 = l*cos(q1+q2)+r+d*cos(-q3+q1+q2-q4)-l*cos(-q3+q1+q2);
M28 = l*sin(q1+q2)+d*sin(-q3+q1+q2-q4)-l*sin(-q3+q1+q2);
M31 = 0;
M32 = 0;
M33 = 1;
M34 = 0;
M35 = 0;
M36 = 0;
M37 = 0;
M38 = 0;
M41 = m*l*c*cos(q3)+m*l*w*sin(q3)-m*c*d*cos(-q3+q2)+m*w*d*sin(-q3+q2)-m*w*w-m*c*cos(-q3+q1+q2)*r+m*w*sin(-q3+q1+q2)*r-m*c*c-I;
M42 = m*l*c*cos(q3)+m*l*w*sin(q3)-m*c*c-m*w*w-I;
M43 = 0;
M44 = m*c*c+m*w*w+I;
M45 = 0;
M46 = 0;
M47 = -r-d*cos(-q3+q1+q2-q4)+l*cos(-q3+q1+q2);
M48 = -d*sin(-q3+q1+q2-q4)+l*sin(-q3+q1+q2);
M51 = 0;
M52 = 0;
M53 = 0;
M54 = 0;
M55 = 0;
M56 = 0;
M57 = r+d*cos(-q3+q1+q2-q4);
M58 = d*sin(-q3+q1+q2-q4);
M61 = 0;
M62 = 0;
M63 = 0;
M64 = 0;
M65 = 0;
M66 = 1;
M67 = 0;
M68 = 0;
M71 = -d*cos(q1)+l*cos(q1+q2)+d*cos(-q3+q1+q2-q4)-l*cos(-q3+q1+q2);
M72 = l*cos(q1+q2)+r+d*cos(-q3+q1+q2-q4)-l*cos(-q3+q1+q2);
M73 = 0;
M74 = -r-d*cos(-q3+q1+q2-q4)+l*cos(-q3+q1+q2);
M75 = -r-d*cos(-q3+q1+q2-q4);
M76 = 0;
M77 = 0;
M78 = 0;
M81 = -d*sin(q1)+l*sin(q1+q2)+d*sin(-q3+q1+q2-q4)-l*sin(-q3+q1+q2);
M82 = l*sin(q1+q2)+d*sin(-q3+q1+q2-q4)-l*sin(-q3+q1+q2);
M83 = 0;
M84 = -d*sin(-q3+q1+q2-q4)+l*sin(-q3+q1+q2);
M85 = -d*sin(-q3+q1+q2-q4);
M86 = 0;
M87 = 0;
M88 = 0;
RHS1 = m*((-d*sin(-r3+r1+r2-r4)+l*sin(-r3+r1+r2)-c*sin(-r3+r1+r2)-w*cos(-r3+r1+r2))*((-d*sin(r1)+l*sin(r1+r2))*v1+l*sin(r1+r2)*v2+(-v3+v1+v2)*(-c*sin(-r3+r1+r2)-w*cos(-r3+r1+r2)))-(r+d*cos(-r3+r1+r2-r4)-l*cos(-r3+r1+r2)+c*cos(-r3+r1+r2)-w*sin(-r3+r1+r2))*((-d*cos(r1)+l*cos(r1+r2)-r)*v1+l*cos(r1+r2)*v2-(-v3+v1+v2)*(c*cos(-r3+r1+r2)-w*sin(-r3+r1+r2))))+m*((-d*sin(-r3+r1+r2-r4)+l*sin(-r3+r1+r2)-c*sin(r1+r2)-w*cos(r1+r2))*((-d*sin(r1)+l*sin(r1+r2))*v1+l*sin(r1+r2)*v2+(v1+v2)*(-c*sin(r1+r2)-w*cos(r1+r2)))-(r+d*cos(-r3+r1+r2-r4)-l*cos(-r3+r1+r2)+c*cos(r1+r2)-w*sin(r1+r2))*((-d*cos(r1)+l*cos(r1+r2)-r)*v1+l*cos(r1+r2)*v2-(v1+v2)*(c*cos(r1+r2)-w*sin(r1+r2))))+M*((-d*sin(-r3+r1+r2-r4)+l*sin(-r3+r1+r2))*((-d*sin(r1)+l*sin(r1+r2))*v1+l*sin(r1+r2)*v2)-(r+d*cos(-r3+r1+r2-r4)-l*cos(-r3+r1+r2))*((-d*cos(r1)+l*cos(r1+r2)-r)*v1+l*cos(r1+r2)*v2))+I*(2*v1+2*v2-v3);
RHS2 = -m*r*v1*w*sin(r1+r2)-M*l*cos(-r3+r1+r2)*v1*r+m*r*v1*c*cos(-r3+r1+r2)+2*m*w*w*v2+2*m*c*c*v2+2*m*l*l*v2*cos(r3)+2*m*l*l*v1*cos(r3)-m*r*v1*w*sin(-r3+r1+r2)-2*m*l*cos(-r3+r1+r2)*v1*r+M*l*l*v2*cos(r3)+M*l*l*v1*cos(r3)-M*d*v1*l*cos(-r3+r2)+m*l*v3*c+2*m*w*w*v1+2*m*c*c*v1-2*m*l*v1*c-2*m*c*l*v2-m*c*c*v3-m*w*w*v3-m*d*v1*w*sin(-r3+r2)+m*d*v1*c*cos(-r3+r2)+m*r*v1*c*cos(r1+r2)+2*I*v1+2*I*v2-I*v3+m*c*v1*d*cos(r2)-m*w*v1*d*sin(r2)-2*m*l*v2*c*cos(r3)-2*m*d*v1*l*cos(-r3+r2)-2*m*l*v1*c*cos(r3);
RHS3 = v5;
RHS4 = -m*r*v1*c*cos(r1+r2)+m*r*v1*w*sin(r1+r2)-m*c*v1*d*cos(r2)-m*c*c*v2-m*c*c*v1+m*w*v1*d*sin(r2)-m*w*w*v2-m*w*w*v1+m*c*l*v2+m*l*v1*c-I*v1-I*v2;
RHS5 = 0;
RHS6 = v6;
RHS7 = 0;
RHS8 = 0;


zeros(nmat2,nmat2,&AA[0][0]);
AA[0][0] = 1.0;
AA[2][2] = 1.0;
AA[4][4] = 1.0;
AA[6][6] = 1.0;
AA[8][8] = 1.0;
AA[10][10] = 1.0;

AA[1][1] = M11;
AA[1][3] = M12;
AA[1][5] = M13;
AA[1][7] = M14;
AA[1][9] = M15;
AA[1][11] = M16;

// ###################
// AA[1][12] = M17; //Pranav -- Error
AA[1][12] = -M17;      //Javad Vesion
// ###################
//AA[1][13] = M18;   //Pranav -- Error
AA[1][13] = -M18;   //Javad Vesion

AA[3][1] = M21;
AA[3][3] = M22;
AA[3][5] = M23;
AA[3][7] = M24;
AA[3][9] = M25;
AA[3][11] = M26;

// ###################
//AA[3][12] = M27;      //Pranav -- Error
AA[3][12] = -M27;      //Javad Vesion
// ###################
//AA[3][13] = M28;     //Pranav -- Error
AA[3][13] = -M28;      //Javad Vesion

AA[5][1] = M31;
AA[5][3] = M32;
AA[5][5] = M33;
AA[5][7] = M34;
AA[5][9] = M35;
AA[5][11] = M36;
AA[5][12] = M37;
AA[5][13] = M38;
AA[7][1] = M41;
AA[7][3] = M42;
AA[7][5] = M43;
AA[7][7] = M44;
AA[7][9] = M45;
AA[7][11] = M46;

// ###################
//AA[7][12] = M47;    //Pranav -- Error
AA[7][12] = -M47;     //Javad Vesion
// ###################
//AA[7][13] = M48;    //Pranav -- Error
AA[7][13] = -M48;     //Javad Vesion

AA[9][1] = M51;
AA[9][3] = M52;
AA[9][5] = M53;
AA[9][7] = M54;
AA[9][9] = M55;
AA[9][11] = M56;

// ###################
//AA[9][12] = M57;     //Pranav -- Error
AA[9][12] = -M57;    //Javad Vesion
// ###################
//AA[9][13] = M58;    //Pranav -- Error
AA[9][13] = -M58;    //Javad Vesion

AA[11][1] = M61;
AA[11][3] = M62;
AA[11][5] = M63;
AA[11][7] = M64;
AA[11][9] = M65;
AA[11][11] = M66;
AA[11][12] = M67;
AA[11][13] = M68;
AA[12][1] = M71;
AA[12][3] = M72;
AA[12][5] = M73;
AA[12][7] = M74;
AA[12][9] = M75;
AA[12][11] = M76;
AA[12][12] = M77;
AA[12][13] = M78;
AA[13][1] = M81;
AA[13][3] = M82;
AA[13][5] = M83;
AA[13][7] = M84;
AA[13][9] = M85;
AA[13][11] = M86;
AA[13][12] = M87;
AA[13][13] = M88;

bb[0] = r1 + r2 - r3 - r4;
bb[1] = RHS1;
bb[2] = r4;
bb[3] = RHS2;
bb[4] = r5;
bb[5] = RHS3;
bb[6] = -r3;
bb[7] = RHS4;
bb[8] = r2;
bb[9] = RHS5;
bb[10] = r6;
bb[11] = RHS6;
bb[12] = RHS7;
bb[13] = RHS8;


//This inverse matches with matlab inverse to 15 significant digits. 
inverse(nmat2,nmat2,&AA[0][0],&AAinv[0][0]);

multiplyMAT2VEC(nmat2,nmat2,nmat2,&AAinv[0][0],&bb[0],&xdot[0]);


u1 = xdot[1];
u2 = xdot[3];
u6 = xdot[5];
u3 = xdot[7];
u4 = xdot[9];
u5 = xdot[11];

xplus[0] = q1;
xplus[1] = u1;
xplus[2] = q2;
xplus[3] = u2;
xplus[4] = q6;
xplus[5] = u6;
xplus[6] = q3;
xplus[7] = u3;
xplus[8] = q4;
xplus[9] = u4;
xplus[10] = q5;
xplus[11] = u5;


}


/* 
 *  Gateway routine
 */

void mexFunction(
  int nlhs, mxArray *plhs[],
  int nrhs, const mxArray *prhs[])
{
  double *x, *xplus;
  unsigned int ms, ns;
  double *GL_DIM;


  // Matlab Syntax:
//
//	[Xplus] = heelstrikeMEX(Xminus params);
//

  /* mexPrintf("Mp = %f\n", Mp); */
  
  /* Get the input argument, x, which is a vector */
ms = mxGetM(prhs[0]); //Get Rows of prhs, prhs[1] is z0
ns = mxGetN(prhs[0]); //Get Columns of prhs

x = mxGetPr(prhs[0]); // Get data elements of prhs 
 
GL_DIM = mxGetPr(prhs[1]);

//[M m I l c w r d g k gam J k_st k_sw C_st C_sw]
M = GL_DIM[0]; m = GL_DIM[1]; I = GL_DIM[2];
l = GL_DIM[3]; c = GL_DIM[4]; w = GL_DIM[5];
r = GL_DIM[6]; d = GL_DIM[7]; g = GL_DIM[8];
k = GL_DIM[9]; gam = GL_DIM[10]; J = GL_DIM[11]; 
k_st = GL_DIM[12]; k_sw = GL_DIM[13]; 
C_st = GL_DIM[14]; C_sw = GL_DIM[15];


 /* Create a matrix for the return argument */
 // Return is zdot 
  plhs[0] = mxCreateDoubleMatrix(ms, ns, mxREAL);
  
  /* Dereference arguments */
  xplus = mxGetPr(plhs[0]);
  
  /* call the computational routine */
  //Note: x = z and xprime = zdot 
  heelstrike(xplus, x);
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
double A[nmat2][nmat2], B[nmat2][nmat2];

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
double A[nmat2][nmat2], B[nmat2][nmat2], sum;

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
double A[nmat2][nmat2], b[nmat2], sum;

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
	double A[nmat2][nmat2]; //Is the transpose
	
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
	double A[nmat2][nmat2];
	
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
double A[nmat2][nmat2]; //We are going to first assemble the transpose

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
double Ainv[nmat2][nmat2];

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