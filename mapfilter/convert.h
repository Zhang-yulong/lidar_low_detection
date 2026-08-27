#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>

#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */
#ifndef PI
#define PI          3.1415926535897932  /* pi */
#endif
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */
extern double *mat(int n, int m);
extern int *imat(int n, int m);
extern double *zeros(int n, int m);
extern double *eye(int n);
extern double dot(const double *a, const double *b, int n);
extern double norm(const double *a, int n);
extern void cross3(const double *a, const double *b, double *c);
extern int normv3(const double *a, double *b);
extern void matcpy(double *A, const double *B, int n, int m);
extern void matmul(const char *tr, int n, int k, int m, double alpha,
        const double *A, const double *B, double beta, double *C);
extern void ecef2pos(const double *r, double *pos);
extern void pos2ecef(const double *pos, double *r);
extern void xyz2enu(const double *pos, double *E);
extern void ecef2enu(const double *pos, const double *r, double *e);
extern void enu2ecef(const double *pos, const double *e, double *r);
extern void covenu(const double *pos, const double *P, double *Q);
extern void PointEnu2Deg( double *point,  double *enu, double *deg);
extern void PointDeg2Enu( double *point,  double *deg, double *enu);
