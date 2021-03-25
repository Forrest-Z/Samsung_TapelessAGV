#ifndef _MATH_UTIL_H
#define _MATH_UTIL_H

#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <stdio.h>

#pragma warning(disable : 4067)

#ifdef __cplusplus
extern "C"
{
#endif

///////////////////////////////////////////////////////////////////////////////
// random utility function
///////////////////////////////////////////////////////////////////////////////
double unit_rand();
int rand_perm(int n);

///////////////////////////////////////////////////////////////////////////////
//Motion Utility Functions
///////////////////////////////////////////////////////////////////////////////
extern double cos_values[361];
extern double sin_values[361];

extern double ir_sin_values[101];
extern double ir_cos_values[101];

extern double urg_sin_values[769];
extern double urg_cos_values[769];

#ifndef PAR_D2R
#define PAR_D2R			0.017453292//(PI/180)
#endif

#ifndef to_rad(n)
#define to_rad(n) ( ((double) n) * PAR_D2R)
#endif

#ifndef to_deg(n)
#define to_deg(n) ( ((double) n) / PAR_D2R)
#endif

#ifndef PI
#define PI 3.141592653589793
#endif

#ifndef PI_2
#define PI_2 6.283185307179586
#endif

///////////////////////////////////////////////////////////////////////////////
// angle functions
///////////////////////////////////////////////////////////////////////////////
double math_heading_diff(double refAngleRad, double nxtAngleRad);
void   math_local_del_pose_rad(double refX, double refY, double refThRad, double nxtX, double nxtY, double nxtThRad, double * pDelX, double *pDelY, double *pDelThRad);
void   math_local_del_pose_deg(double refX, double refY, double refThDeg, double nxtX, double nxtY, double nxtThDeg, double * pDelX, double *pDelY, double *pDelThDeg);

///////////////////////////////////////////////////////////////////////////////
// motion functions
///////////////////////////////////////////////////////////////////////////////
double motion_normalize_heading_rad(double theta_rad);
double motion_normalize_heading_deg(double deg);
double motion_compute_heading_diff(
	double targetPoseX, double targetPoseY, double targetPoseTh,
	double curPoseX, double curPoseY, double curTh_rad, 
	double delPoseX, double delPoseY, double deltaThetaRad);
	
double motion_compute_next_pose
(
	double x_mm, 
	double y_mm, 
	double th_deg, 
	double time_step_sec,
	double v_tr_m_sec,
	double v_rot_deg_sec, 
	double *px_mm, 
	double *py_mm, 
	double *pth_deg_mm
);

//return value unit is mm/sec
double compute_velocity_to_goal(double goal_x, double goal_y, double prev_x, double prev_y, double cur_x, double cur_y, double del_time /*sec*/); //unit is mm

///////////////////////////////////////////////////////////////////////////////
// math utility function
///////////////////////////////////////////////////////////////////////////////
int    round_i(double num);
double round_d(double num);
float  round_f(float num);
///////////////////////////////////////////////////////////////////////////////
// time functions
///////////////////////////////////////////////////////////////////////////////
double time_milli_sec();

///////////////////////////////////////////////////////////////////////////////
// vector functions
///////////////////////////////////////////////////////////////////////////////
double vec_norm     (double * a, int n); //a must be a column vector
double vec_norm_l1  (double * a, int n); //a must be a column vector
double vec_norm_l2  (double * a, int n); //a must be a column vector
double vec_norm_ln  (double * a, int n); //a must be a column vector
double vec_norm_linf(double * a, int n); //a must be a column vector

double vec_angle(double * a, double * b, int n);

///////////////////////////////////////////////////////////////////////////////
// matrix functions
///////////////////////////////////////////////////////////////////////////////
int mat_copy(double * A, double * B, int m, int n);

int mat_add	(double * a, double * b, int m, int n, double * ret);
int mat_subtract(double * a, double * b, int m, int n, double * ret);
int mat_multiply(double * a, double * b, int m, int p, int n, double * ret);

double mat_determinent (double * a, int n);
double mat_trace       (double * a, int n);
double mat_abs_diff_sum(double * a, double * b, int m, int n);
double mat_abs_diff_max(double * a, double * b, int m, int n);
double mat_diff_det    (double * a, double * b, int m, int n);

int mat_transpose (double * a, int m, int n, double * ret);
int mat_covariance(double * a, double * b, int n, double * ret);
int mat_inverse   (double * a, int n, double * AInverse);

///////////////////////////////////////////////////////////////////////////////
// probability functions
///////////////////////////////////////////////////////////////////////////////
//x,mu is column vector
int    prob_pdf_gauss_by_vector(double * mu, double * cov, int dim, double * x, double * prob_value);
double prob_pdf_gauss(double mu, double sigma, double x);
double prob_pdf_gauss_by_one(double mu, double sigma, double multipler, double x);
double prob_pdf_exp(double lamda, double x);
double prob_pdf_exp_by_one(double lamda, double multiplier, double x);

///////////////////////////////////////////////////////////////////////////////
// stochastic functions
///////////////////////////////////////////////////////////////////////////////
//int    stoc_mahalanobis(double * x, double * mu, double * cov, 

///////////////////////////////////////////////////////////////////////////////
// probability functions
///////////////////////////////////////////////////////////////////////////////
double rand_uniform   (long * seed);
double rand_unifrnd   (double a, double b, long * seed);
double rand_exprnd    (double mu, long *seed);
double rand_gamrnd    (int n, double lambda, long * seed);
double rand_gauss     (double mean, double std_dev);
double rand_gauss_conf(double mean, double std_dev, double conf);

///////////////////////////////////////////////////////////////////////////////
// Markov functions
///////////////////////////////////////////////////////////////////////////////
int markov_nstep_prob	   (int n, double * a, int dim, double * ret);
int markov_limit_prob	   (double * a, int dim, double * limit_prob);
int markov_nstep_limit_prob(int n, double * a, int dim, double * limit_prob);
int markov_reward	   (double * reward, double * phi, int dim, double * phi_reward);

int markov_conv_prob	   (double * a, int dim, double * ret, double epsilon, int max_iter);
int markov_conv_limit_prob (double * a, int dim, double * limit_prob, double epsilon, int max_iter);

#ifdef __cplusplus
}
#endif
#endif