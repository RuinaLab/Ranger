// WITH LU DECOMPOSITION 
// Called by onestepMEXm and has monodromy matrix in it.
// piecewise linear curent for hip and ankle, Current = Constant1 + Constant2*t.
// Copied from singleMEXm.cpp from 20091019Zop
#include "vfieldfunc.h"
#include "matrix.h"
#include <math.h>
#include "mex.h"
#define nmat2 14
#define NMAT 18 //should be atleast nmat+1, here nmat = 6
#define PI 3.141592653589793

//static double s, r, b;	// These will be initialized by vfieldfuncinit before the integration begins

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

//************************************************
//
// THIS FUNCTION IS ONLY FOR USE WITH DOP853 
//
//***********************************************

//GLOBAL variables...
static double M, m, I, l, c, w, r, d, g, k, gam, Jm, k_st, k_sw, C_st, C_sw;
static double R, V0, I0, G_h, G_a, K, I_MAX, e_r, mu_h, C1_h, C0_h, mu_st, C1_st, C0_st, C1m_st, C0m_st, ElectricalW, eps, eps_f;
static double h0, h1, a0, a1, b0, b1;
static double coefs[100];


void vfieldfunc(unsigned n,  double t, double* y,  mxArray* params, double* f)
{

		double* paramsarray = mxGetPr(params);
        double GL_DIM[16], GL_MOT[19];
        double q1, q2, q3, q4, q5, q6, u1, u2, u3, u4, u5, u6;
        double Th, Ta, Tb, Thip, Ih, Ia, Ib, Vh, Va, Vb, Td_a, Td_b, Tatemp, Tbtemp, Tf_st, Tf_sw;
        double DE, E, val1, val2, val3; 
        double fac1, fac2, fac3, Ph, Pa, Pb;
        double xh, vx_h, yh, vy_h, xhddot, yhddot, ud1, ud2, ud3, ud4, ud5, ud6;
        double M11, M12, M13, M14, M15, M16, M17, M18;
        double M21, M22, M23, M24, M25, M26, M27, M28;
        double M31, M32, M33, M34, M35, M36, M37, M38;
        double M41, M42, M43, M44, M45, M46, M47, M48;
        double M51, M52, M53, M54, M55, M56, M57, M58;
        double M61, M62, M63, M64, M65, M66, M67, M68;
        double M71, M72, M73, M74, M75, M76, M77, M78;
        double M81, M82, M83, M84, M85, M86, M87, M88;
        double RHS1, RHS2, RHS3, RHS4, RHS5, RHS6, RHS7, RHS8;
        
        //double M11, M12, M13, M21, M22, M23, M31, M32, M33, RHS1, RHS2, RHS3;
        int  i, j;
        //int nmat = 14; //nmat should be equal to state space else error 12/10/2009
        double AA[nmat2][nmat2], bb[nmat2], AAinv[nmat2][nmat2], udot[nmat2];
        //double dbdx[nmat][nmat], dAAdx[nmat][nmat], dAdx[nmat][nmat][nmat];
        //double PXvec[nmat*nmat], dPXdtvec[nmat*nmat], PX[nmat][nmat], dPXdt[nmat][nmat]; 
   
        // Retrieve parameters
        //parms = [GL_DIM GL_MOT coefs];
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
        for (i=0;i<=5;i++)
        {
            coefs[i] = paramsarray[j];
            j = j + 1;
        }
    h0 = coefs[0]; h1 = coefs[1];
    a0 = coefs[2]; a1 = coefs[3];
    b0 = coefs[4]; b1 = coefs[5];
    //mexPrintf("%f \t %f \n", b0, b1);    
           
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


    //mexPrintf("sum is = %f\n", val);
    //mexPrintf("Hello");
    //s = mxGetScalar(mxGetField(params,0,"s"));
    //r = mxGetScalar(mxGetField(params,0,"r"));
    //b = mxGetScalar(mxGetField(params,0,"b"));
    
    Ih = h0 + h1*t;
    Ia = a0 + a1*t;
    Ib = b0 + b1*t;

    Vh = Ih*(R+V0/sqrt(Ih*Ih+I0*I0)) + G_h*K*u3;
    Va = Ia*(R+V0/sqrt(Ia*Ia+I0*I0)) + G_a*K*u6;
    Vb = Ib*(R+V0/sqrt(Ib*Ib+I0*I0)) + G_a*K*u5;

    Th = G_h*K*Ih*(1-mu_h*tanh(eps_f*Ih*u3))-C1_h*u3-C0_h*tanh(eps_f*u3);
    Ta = G_a*K*Ia*(1-mu_st*tanh(eps_f*Ia*u6))-C1m_st*u6-C0m_st*tanh(eps_f*u6);
    Tb = G_a*K*Ib*(1-mu_st*tanh(eps_f*Ib*u5))-C1m_st*u5-C0m_st*tanh(eps_f*u5);

    Ph = Vh*Ih;
    Pa = Va*Ia;
    Pb = Vb*Ib;

    // Square root smoothing //
    // Always positive //
    //eps = 0.1;
    fac1 = sqrt(Ph*Ph+eps*eps);
    fac2 = sqrt(Pa*Pa+eps*eps);
    fac3 = sqrt(Pb*Pb+eps*eps);
    val1 = (1+e_r)*((Ph-fac1)/2) + fac1;
    val2 = (1+e_r)*((Pa-fac2)/2) + fac2;
    val3 = (1+e_r)*((Pb-fac3)/2) + fac3;

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
    Td_a = -C_st*u1;//-C_st*u2;//
    Td_b = -C_sw*(u1+u2-u3-u4);//-C_sw*u4;//
    Tf_st = -C1_st*u2-C0_st*tanh(eps_f*u2);
    Tf_sw = -C1_st*u4-C0_st*tanh(eps_f*u4);
    Tatemp = k_st*(q6-q2);
    Tbtemp = k_sw*(q5-q4);
    
    M11 = 2*m*r*c*cos(-q3+q1+q2)-2*m*r*w*sin(-q3+q1+q2)+4*m*r*d*cos(q1)-2*m*r*w*sin(q1+q2)+2*m*r*c*cos(q1+q2)+2*M*r*d*cos(q1)-2*M*r*l*cos(q1+q2)-4*m*r*l*cos(q1+q2)+2*m*r*r+M*r*r+M*l*l-2*m*l*c+2*m*c*c+2*m*d*d+2*m*l*l+M*d*d+2*m*w*w-2*m*l*c*cos(q3)-2*m*l*w*sin(q3)+2*m*d*c*cos(-q3+q2)-2*m*d*w*sin(-q3+q2)-4*m*d*l*cos(q2)+2*m*d*c*cos(q2)-2*m*d*w*sin(q2)-2*M*d*l*cos(q2)+2*I;
    M12 = 2*m*w*w+2*m*c*c+M*l*l+2*m*l*l-2*m*l*c+m*r*c*cos(-q3+q1+q2)-m*r*w*sin(-q3+q1+q2)-m*r*w*sin(q1+q2)+m*r*c*cos(q1+q2)-M*r*l*cos(q1+q2)-2*m*r*l*cos(q1+q2)-m*d*w*sin(-q3+q2)+m*d*c*cos(-q3+q2)-2*m*l*w*sin(q3)-2*m*l*c*cos(q3)+m*d*c*cos(q2)-m*d*w*sin(q2)-2*m*d*l*cos(q2)-M*d*l*cos(q2)+2*I;
    M13 = 0;
    M14 = -m*c*c-m*w*w-m*d*c*cos(-q3+q2)+m*d*w*sin(-q3+q2)+m*l*c*cos(q3)+m*l*w*sin(q3)-m*r*c*cos(-q3+q1+q2)+m*r*w*sin(-q3+q1+q2)-I;
    M15 = 0;
    M16 = 0;
    M17 = -l*cos(q1+q2)+d*cos(q1)-d*cos(-q3+q1+q2-q4)+l*cos(-q3+q1+q2);
    M18 = -l*sin(q1+q2)+d*sin(q1)-d*sin(-q3+q1+q2-q4)+l*sin(-q3+q1+q2);
    M21 = 2*m*c*c-M*l*d*cos(q2)-M*l*cos(q1+q2)*r-2*m*l*cos(q1+q2)*r+m*c*d*cos(-q3+q2)-2*m*l*w*sin(q3)-m*w*d*sin(-q3+q2)-2*m*l*c*cos(q3)-2*m*l*c+2*m*w*w+m*r*c*cos(-q3+q1+q2)-m*r*w*sin(-q3+q1+q2)+m*c*cos(q1+q2)*r-m*w*sin(q1+q2)*r-2*m*l*d*cos(q2)+2*I-m*w*d*sin(q2)+M*l*l+2*m*l*l+m*c*d*cos(q2);
    M22 = 2*m*w*w+2*m*l*l+2*m*c*c+M*l*l-2*m*l*c-2*m*l*w*sin(q3)-2*m*l*c*cos(q3)+2*I;
    M23 = 0;
    M24 = m*l*c*cos(q3)+m*l*w*sin(q3)-m*w*w-m*c*c-I;
    M25 = 0;
    M26 = 0;
    M27 = -l*cos(q1+q2)-r-d*cos(-q3+q1+q2-q4)+l*cos(-q3+q1+q2);
    M28 = -l*sin(q1+q2)-d*sin(-q3+q1+q2-q4)+l*sin(-q3+q1+q2);
    M31 = 0;
    M32 = 0;
    M33 = 1;
    M34 = 0;
    M35 = 0;
    M36 = 0;
    M37 = 0;
    M38 = 0;
    M41 = -m*w*w-m*c*c-m*c*d*cos(-q3+q2)+m*c*l*cos(q3)+m*w*d*sin(-q3+q2)-I+m*w*l*sin(q3)-m*c*cos(-q3+q1+q2)*r+m*w*sin(-q3+q1+q2)*r;
    M42 = m*c*l*cos(q3)+m*w*l*sin(q3)-m*w*w-m*c*c-I;
    M43 = 0;
    M44 = m*w*w+m*c*c+I;
    M45 = 0;
    M46 = 0;
    M47 = r+d*cos(-q3+q1+q2-q4)-l*cos(-q3+q1+q2);
    M48 = d*sin(-q3+q1+q2-q4)-l*sin(-q3+q1+q2);
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
    M71 = d*cos(-q3+q1+q2-q4)+l*cos(q1+q2)-l*cos(-q3+q1+q2)-d*cos(q1);
    M72 = r+d*cos(-q3+q1+q2-q4)+l*cos(q1+q2)-l*cos(-q3+q1+q2);
    M73 = 0;
    M74 = -r-d*cos(-q3+q1+q2-q4)+l*cos(-q3+q1+q2);
    M75 = -r-d*cos(-q3+q1+q2-q4);
    M76 = 0;
    M77 = 0;
    M78 = 0;
    M81 = l*sin(q1+q2)-l*sin(-q3+q1+q2)-d*sin(q1)+d*sin(-q3+q1+q2-q4);
    M82 = l*sin(q1+q2)-l*sin(-q3+q1+q2)+d*sin(-q3+q1+q2-q4);
    M83 = 0;
    M84 = -d*sin(-q3+q1+q2-q4)+l*sin(-q3+q1+q2);
    M85 = -d*sin(-q3+q1+q2-q4);
    M86 = 0;
    M87 = 0;
    M88 = 0;
    RHS1 = m*g*((d*sin(q1)-l*sin(q1+q2)+c*sin(q1+q2)+w*cos(q1+q2))*cos(gam)-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(q1+q2)-w*sin(q1+q2))*sin(gam))+m*g*((d*sin(q1)-l*sin(q1+q2)+c*sin(-q3+q1+q2)+w*cos(-q3+q1+q2))*cos(gam)-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(-q3+q1+q2)-w*sin(-q3+q1+q2))*sin(gam))+M*g*((-l*sin(q1+q2)+d*sin(q1))*cos(gam)-(r+d*cos(q1)-l*cos(q1+q2))*sin(gam))-m*((-d*sin(q1)+l*sin(q1+q2)-c*sin(q1+q2)-w*cos(q1+q2))*(((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2-(u1+u2)*(u1+u2)*(c*cos(q1+q2)-w*sin(q1+q2)))-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(q1+q2)-w*sin(q1+q2))*(((-l*sin(q1+q2)+d*sin(q1))*u1-l*sin(q1+q2)*u2)*u1+(-l*sin(q1+q2)*u1-l*sin(q1+q2)*u2)*u2-(u1+u2)*(u1+u2)*(-c*sin(q1+q2)-w*cos(q1+q2))))-m*((-d*sin(q1)+l*sin(q1+q2)-c*sin(-q3+q1+q2)-w*cos(-q3+q1+q2))*(((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2-(-u3+u1+u2)*(-u3+u1+u2)*(c*cos(-q3+q1+q2)-w*sin(-q3+q1+q2)))-(r+d*cos(q1)-l*cos(q1+q2)+c*cos(-q3+q1+q2)-w*sin(-q3+q1+q2))*(((-l*sin(q1+q2)+d*sin(q1))*u1-l*sin(q1+q2)*u2)*u1+(-l*sin(q1+q2)*u1-l*sin(q1+q2)*u2)*u2-(-u3+u1+u2)*(-u3+u1+u2)*(-c*sin(-q3+q1+q2)-w*cos(-q3+q1+q2))))-M*((l*sin(q1+q2)-d*sin(q1))*(((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2)-(r+d*cos(q1)-l*cos(q1+q2))*(((-l*sin(q1+q2)+d*sin(q1))*u1-l*sin(q1+q2)*u2)*u1+(-l*sin(q1+q2)*u1-l*sin(q1+q2)*u2)*u2));
    RHS2 = -2*m*g*l*sin(-gam+q1+q2)+m*g*c*sin(-gam+q1+q2)+m*g*w*cos(-gam+q1+q2)+m*g*c*sin(-gam-q3+q1+q2)+m*g*w*cos(-gam-q3+q1+q2)-M*g*l*sin(-gam+q1+q2)+2*m*u1*u1*d*l*sin(q2)-m*u1*u1*d*c*sin(q2)-m*u1*u1*d*w*cos(q2)+m*l*u3*u3*c*sin(q3)-2*m*l*u3*u2*c*sin(q3)-m*l*u3*u3*w*cos(q3)+2*m*l*u3*u1*w*cos(q3)-2*m*l*u3*u1*c*sin(q3)+2*m*l*u3*u2*w*cos(q3)-m*c*u1*u1*d*sin(-q3+q2)-m*w*u1*u1*d*cos(-q3+q2)+M*l*u1*u1*d*sin(q2);
    RHS3 = 0;
    RHS4 = -m*g*c*sin(-gam-q3+q1+q2)-m*g*w*cos(-gam-q3+q1+q2)-m*w*l*u2*u2*cos(q3)+2*m*c*u1*l*u2*sin(q3)+m*c*l*u1*u1*sin(q3)-m*w*l*u1*u1*cos(q3)-2*m*w*u1*l*u2*cos(q3)+m*w*u1*u1*d*cos(-q3+q2)+m*c*u1*u1*d*sin(-q3+q2)+m*c*l*u2*u2*sin(q3);
    RHS5 = 0;
    RHS6 = 0;
    RHS7 = (-d*sin(q1)-l*sin(-q3+q1+q2)+d*sin(-q3+q1+q2-q4)+l*sin(q1+q2))*u1*u1+(2*u2*sin(-q3+q1+q2-q4)*d-2*l*u2*sin(-q3+q1+q2)+2*u3*l*sin(-q3+q1+q2)+2*l*sin(q1+q2)*u2-2*u4*sin(-q3+q1+q2-q4)*d-2*u3*sin(-q3+q1+q2-q4)*d)*u1-2*u2*d*sin(-q3+q1+q2-q4)*u4+d*sin(-q3+q1+q2-q4)*u2*u2-2*u2*d*sin(-q3+q1+q2-q4)*u3+2*u2*u3*l*sin(-q3+q1+q2)-u3*u3*l*sin(-q3+q1+q2)+d*sin(-q3+q1+q2-q4)*u4*u4-u2*u2*l*sin(-q3+q1+q2)+d*sin(-q3+q1+q2-q4)*u3*u3+2*u3*d*sin(-q3+q1+q2-q4)*u4+l*sin(q1+q2)*u2*u2;
    RHS8 = (-l*cos(q1+q2)-d*cos(-q3+q1+q2-q4)+l*cos(-q3+q1+q2)+d*cos(q1))*u1*u1+(2*d*cos(-q3+q1+q2-q4)*u3+2*d*cos(-q3+q1+q2-q4)*u4-2*u3*l*cos(-q3+q1+q2)-2*d*cos(-q3+q1+q2-q4)*u2+2*u2*l*cos(-q3+q1+q2)-2*l*cos(q1+q2)*u2)*u1+2*u2*d*cos(-q3+q1+q2-q4)*u4-d*cos(-q3+q1+q2-q4)*u2*u2-l*cos(q1+q2)*u2*u2+2*u2*d*cos(-q3+q1+q2-q4)*u3+u3*u3*l*cos(-q3+q1+q2)-d*cos(-q3+q1+q2-q4)*u4*u4+u2*u2*l*cos(-q3+q1+q2)-d*cos(-q3+q1+q2-q4)*u3*u3-2*u3*d*cos(-q3+q1+q2-q4)*u4-2*u2*u3*l*cos(-q3+q1+q2);


    RHS1 = RHS1 + Td_a + Td_b;
    RHS2 = RHS2 + Tatemp + Td_b + Tf_st;
    RHS3 = RHS3 + (Ta-Tatemp)/Jm; 

	// ################################
	// RHS4 = RHS4 + Thip + Th + Td_b;    // Pranav --  Error
	RHS4 = RHS4 + Thip + Th - Td_b;    //Javad Version

	// #####################################
	// RHS5 = RHS5 + Tbtemp + Td_b + Tf_sw;   // Pranav -- Error
	RHS5 = RHS5 + Tbtemp - Td_b - Tf_sw;     //Javad Version

    RHS6 = RHS6 + (Tb-Tbtemp)/Jm;


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
    AA[1][12] = M17;
    AA[1][13] = M18;
    AA[3][1] = M21;
    AA[3][3] = M22;
    AA[3][5] = M23;
    AA[3][7] = M24;
    AA[3][9] = M25;
    AA[3][11] = M26;
    AA[3][12] = M27;
    AA[3][13] = M28;
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
    AA[7][12] = M47;
    AA[7][13] = M48;
    AA[9][1] = M51;
    AA[9][3] = M52;
    AA[9][5] = M53;
    AA[9][7] = M54;
    AA[9][9] = M55;
    AA[9][11] = M56;
    AA[9][12] = M57;
    AA[9][13] = M58;
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

    bb[0] = u1;
    bb[1] = RHS1;
    bb[2] = u2;
    bb[3] = RHS2;
    bb[4] = u6;
    bb[5] = RHS3;
    bb[6] = u3;
    bb[7] = RHS4;
    bb[8] = u4;
    bb[9] = RHS5;
    bb[10] = u5;
    bb[11] = RHS6;
    bb[12] = RHS7;
    bb[13] = RHS8;


    //This inverse matches with matlab inverse to 15 significant digits. 
    inverse(nmat2,nmat2,&AA[0][0],&AAinv[0][0]);

    multiplyMAT2VEC(nmat2,nmat2,nmat2,&AAinv[0][0],&bb[0],&udot[0]);
      
    
    ud1 = udot[1];
    ud2 = udot[3];
    ud6 = udot[5];
    ud3 = udot[7];
    ud4 = udot[9];
    ud5 = udot[11];

	//	//Px[0] = udot[12];
	//	Py[0]= udot[13];
	//	//##### the last term should be -Py[0] ####################################
	//	//Ry[0] = m*(2*((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+2*(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2+2*(l*sin(q1+q2)-d*sin(q1))*ud1+2*l*sin(q1+q2)*ud2-(u1+u2)*(u1+u2)*(c*cos(q1+q2)-w*sin(q1+q2))+(ud1+ud2)*(-c*sin(q1+q2)-w*cos(q1+q2))-(-u3+u1+u2)*(-u3+u1+u2)*(c*cos(-q3+q1+q2)-w*sin(-q3+q1+q2))+(-ud3+ud1+ud2)*(-c*sin(-q3+q1+q2)-w*cos(-q3+q1+q2)))+M*(((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2+(l*sin(q1+q2)-d*sin(q1))*ud1+l*sin(q1+q2)*ud2)+(2*m+M)*g+Py[0];
	//	Ry[0] = m*(2*((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+2*(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2+2*(l*sin(q1+q2)-d*sin(q1))*ud1+2*l*sin(q1+q2)*ud2-(u1+u2)*(u1+u2)*(c*cos(q1+q2)-w*sin(q1+q2))+(ud1+ud2)*(-c*sin(q1+q2)-w*cos(q1+q2))-(-u3+u1+u2)*(-u3+u1+u2)*(c*cos(-q3+q1+q2)-w*sin(-q3+q1+q2))+(-ud3+ud1+ud2)*(-c*sin(-q3+q1+q2)-w*cos(-q3+q1+q2)))+M*(((l*cos(q1+q2)-d*cos(q1))*u1+l*cos(q1+q2)*u2)*u1+(l*cos(q1+q2)*u1+l*cos(q1+q2)*u2)*u2+(l*sin(q1+q2)-d*sin(q1))*ud1+l*sin(q1+q2)*ud2)+(2*m+M)*g-Py[0];

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

/*void eventfunc (unsigned ndims, double t, double *y, mxArray* params,
					   int ievent, double &value, int &isterminal, int &direction)
{
    
    double q1, q2, q3, q4;
    
    q1    = y[0]   ;  
    q2    = y[2]   ;
    q3    = y[4]   ;    
    q4    = y[6]   ;   
      
    //value = t-0.55;
    //isterminal = 1;
    //direction = 1;
    
    
	switch (ievent)
	{
	case 0:
        value = -l*cos(q1+q2)+d*cos(q1)+l*cos(-q3+q1+q2)-d*cos(q1+q2-q3-q4);    
        if (q3<-0.1)
            isterminal=1; //Ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
        else
            isterminal=0;
        direction=-1; // The t_final can be approached by any direction is indicated by this
		break;
	} 
}*/




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