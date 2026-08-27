#include"convert.h"
/* new matrix ------------------------------------------------------------------
* allocate memory of matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double *mat(int n, int m)
{
    double *p;

    if (n <= 0 || m <= 0) return NULL;
    if (!(p = (double *)malloc(sizeof(double)*n*m))) {
        //fatalerr("matrix memory allocation error: n=%d,m=%d\n", n, m);
    }
    return p;
}
/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern int *imat(int n, int m)
{
    int *p;

    if (n <= 0 || m <= 0) return NULL;
    if (!(p = (int *)malloc(sizeof(int)*n*m))) {
        //fatalerr("integer matrix memory allocation error: n=%d,m=%d\n", n, m);
    }
    return p;
}
/* zero matrix -----------------------------------------------------------------
* generate new zero matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double *zeros(int n, int m)
{
    double *p;

#if NOCALLOC
    if ((p = mat(n, m))) for (n = n * m - 1; n >= 0; n--) p[n] = 0.0;
#else
    if (n <= 0 || m <= 0) return NULL;
    if (!(p = (double *)calloc(sizeof(double), n*m))) {
        //fatalerr("matrix memory allocation error: n=%d,m=%d\n", n, m);
    }
#endif
    return p;
}
/* identity matrix -------------------------------------------------------------
* generate new identity matrix
* args   : int    n         I   number of rows and columns of matrix
* return : matrix pointer (if n<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double *eye(int n)
{
    double *p;
    int i;

    if ((p = zeros(n, n))) for (i = 0; i < n; i++) p[i + i * n] = 1.0;
    return p;
}
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot(const double *a, const double *b, int n)
{
    double c = 0.0;

    while (--n >= 0) c += a[n] * b[n];
    return c;
}
/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
extern double norm(const double *a, int n)
{
    return sqrt(dot(a, a, n));
}
/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
extern void cross3(const double *a, const double *b, double *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}
/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int normv3(const double *a, double *b)
{
    double r;
    if ((r = norm(a, 3)) <= 0.0) return 0;
    b[0] = a[0] / r;
    b[1] = a[1] / r;
    b[2] = a[2] / r;
    return 1;
}
/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
extern void matcpy(double *A, const double *B, int n, int m)
{
    memcpy(A, B, sizeof(double)*n*m);
}
/* multiply matrix -----------------------------------------------------------*/
extern void matmul(const char *tr, int n, int k, int m, double alpha,
    const double *A, const double *B, double beta, double *C)
{
    double d;
    int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

    for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
        d = 0.0;
        switch (f) {
        case 1: for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m]; break;
        case 2: for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k]; break;
        case 3: for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m]; break;
        case 4: for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k]; break;
        }
        if (beta == 0.0) C[i + j * n] = alpha * d; else C[i + j * n] = alpha * d + beta * C[i + j * n];
    }
}
/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void ecef2pos(const double *r, double *pos)
{
    double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

    for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
        zk = z;
        sinp = z / sqrt(r2 + z * z);
        v = RE_WGS84 / sqrt(1.0 - e2 * sinp*sinp);
        z = r[2] + v * e2*sinp;
    }
    pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
    pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
    pos[2] = sqrt(r2 + z * z) - v;
}
/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void pos2ecef(const double *pos, double *r)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp*sinp);

    r[0] = (v + pos[2])*cosp*cosl;
    r[1] = (v + pos[2])*cosp*sinl;
    r[2] = (v*(1.0 - e2) + pos[2])*sinp;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double *pos, double *E)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

    E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
    E[1] = -sinp * cosl; E[4] = -sinp * sinl; E[7] = cosp;
    E[2] = cosp * cosl;  E[5] = cosp * sinl;  E[8] = sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];

    xyz2enu(pos, E);
    matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}
/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
extern void enu2ecef(const double *pos, const double *e, double *r)
{
    double E[9];

    xyz2enu(pos, E);
    matmul("TN", 3, 1, 3, 1.0, E, e, 0.0, r);
}
/* transform covariance to local tangental coordinate --------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *P        I   covariance in ecef coordinate
*          double *Q        O   covariance in local tangental coordinate
* return : none
*-----------------------------------------------------------------------------*/
extern void covenu(const double *pos, const double *P, double *Q)
{
    double E[9], EP[9];

    xyz2enu(pos, E);
    matmul("NN", 3, 3, 3, 1.0, E, P, 0.0, EP);
    matmul("NT", 3, 3, 3, 1.0, EP, E, 0.0, Q);
}

/*---------- lat/lon Deg  TO ENU ----------------------
输入：
  const double *point   point[3]  原点经纬度坐标    纬度/经度/高度（单位度）
  double *deg         deg[3]      待转换经纬度坐标  纬度/经度/高度(单位度）
输出
     const double *enu     enu[3]    待转换enu值  enu坐标系(单位米）
 */
extern void PointDeg2Enu( double *point,  double *deg, double *enu)
{
    double pointR[3];
    double pointecef[3];
    double degR[3];
    double len[3];
    double degecef[3];
    for (int i = 0; i < 3; i++)
    {
        pointR[i] = 0.0;
        pointecef[i] = 0.0;
        degR[i] = 0.0;
        len[i] = 0.0;
        degecef[i] = 0.0;
    }
    pointR[0] = point[0] * D2R;
    pointR[1] = point[1] * D2R;
    pointR[2] = point[2];

    degR[0] = deg[0] * D2R;
    degR[1] = deg[1] * D2R;
    degR[2] = deg[2];

    pos2ecef(pointR, pointecef);
    pos2ecef(degR, degecef);

    len[0] = degecef[0] - pointecef[0];
    len[1] = degecef[1] - pointecef[1];
    len[2] = degecef[2] - pointecef[2];
    ecef2enu(pointR, len, enu);
}


/*----------ENU TO lat/lon Deg----------------------
输入：
  const double *point   point[3]  原点经纬度坐标    纬度/经度/高度（单位度）
  const double *enu     enu[3]    待转换enu值  enu坐标系(单位米）
输出
  double *deg         deg[3]      转换后经纬度坐标          纬度/经度/高度(单位度）
 */
extern void PointEnu2Deg( double *point,  double *enu, double *deg)
{
    double pointR[3];
    double pointecef[3];
    double ecef[3];
    double pointDeg[3];
    for (int i = 0; i < 3; i++)
    {
        pointR[i] = 0.0;
        pointecef[i] = 0.0;
        ecef[i] = 0.0;
        pointDeg[i] = 0.0;
    }
    pointR[0] = point[0] * D2R;
    pointR[1] = point[1] * D2R;
    pointR[2] = point[2];
    enu2ecef(pointR, enu, pointecef);
    pos2ecef(pointR, ecef);
    pointecef[0] = ecef[0] + pointecef[0];
    pointecef[1] = ecef[1] + pointecef[1];
    pointecef[2] = ecef[2] + pointecef[2];
    ecef2pos(pointecef, pointDeg);
    deg[0] = pointDeg[0] / (D2R);
    deg[1] = pointDeg[1] / (D2R);
    deg[2] = pointDeg[2];
}
