/****************************************************************************
Copyright (c), 2019, www.echiev.com Co., Ltd.
File name:  readmap.cpp
Version:    V3.0
Date:       2019/01/15
Description:读取高精地图并存储，提供高精地图信息给其他模块使用

History:
 1.Date:    2019/01/15 
            Create file
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
#include <math.h>
//#include <time.h>
//#include <sys/time.h>

#include <set>
#include "read_hdmap.h"
#include "coordinate_convert_tool.h"


/* 全局锚点经纬度（地图文件头读取，供外部参考） */
double g_dLon = 0.0;
double g_dLat = 0.0;

#define JUNCTION_NUM           800											/* junction的个数 */
#define ROAD_NUM               3000                                           /* road 的个数 */
#define SECTION_NUM			   1000											/* section的个数 */
#define LANE_NUM               1500                                           /* lane 的个数 */
#define CONNECT_NUM       	   2000
#define LANE_SAMPLE            2000
#define SECTION_SAMPLE		   2000
#define ROAD_SAMPLE            600
#define JUNCTION_SAMPLE        600													
#define STATION_NUM			   200					                        /* 站点数量 */
#define STOP_LINE_COUNT        1000                                            /* 停止线数量 */
#define CROSSWALK_COUNT        1000 											/* 斑马线数量 */
#define SIHNAL_COUNT           2000                                            /* 信号灯数量 */
#define CLEAR_AREA_COUNT       1000                                            /* 禁停区数量 */
#define TRAFFICSIGN_COUNT      1000 											/* 路标数量 */  
#define pi                     3.1415926


#define ESC_START     "\033["
#define ESC_END       "\033[0m"
#define COLOR_FATAL   "31;40;5m"
#define COLOR_ALERT   "31;40;1m"
#define COLOR_CRIT    "31;40;1m"
#define COLOR_ERROR   "31;40;1m"
#define COLOR_WARN    "33;40;1m"
#define COLOR_NOTICE  "34;40;1m"
#define COLOR_INFO    "32;40;1m"
#define COLOR_DEBUG   "36;40;1m"
#define COLOR_TRACE   "37;40;1m"

/******************************* 宏定义 ***********************************/
#define filename(x) strrchr(x,'/')?strrchr(x,'/')+1:x

//#define TRACE_INFO(format, args...) (printf( ESC_START COLOR_INFO "[INFO]:%s -%s() line:%-d " format ESC_END,  filename(__FILE__), __FUNCTION__ , __LINE__, ##args))
//#define TRACE_DEBUG(format, args...) (printf( ESC_START COLOR_DEBUG "[DEBUG]:%s -%s() line:-%d " format ESC_END,  filename(__FILE__), __FUNCTION__ , __LINE__, ##args))
//#define TRACE_WARN(format, args...) (printf( ESC_START COLOR_WARN "[WARN]:%s -%s() line:-%d " format ESC_END,  filename(__FILE__), __FUNCTION__ , __LINE__, ##args))
//#define TRACE_ERROR(format, args...) (printf( ESC_START COLOR_ERROR "[ERROR]:%s -%s() line:-%d " format ESC_END, filename(__FILE__), __FUNCTION__ , __LINE__, ##args))
 
/******************************* 全局变量定义 ***********************************/

//STR_MAP *g_pstMap               = NULL;				 

//std::string WGS84 = "+proj=longlat +datum=WGS84 +no_defs";
//std::string TMERC = "+proj=tmerc +lat_0=22.6656342253927n  +lon_0=114.054429823102 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
//CoordinateConvertTool *convert_coordiante = new CoordinateConvertTool(WGS84, TMERC);

float distance(const float x1, const float y1 , const float x2, const float y2)
{
	return sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
}

void rotateAsPoint(const float x1, const float y1 , const float x2, const float y2, const float theta, float &x, float &y)
{
	float r = distance(x1, y1, x2, y2);
	float theta2 = atan2f(y2 - y1, x2 - x1);

    x = x1 + r*cosf(theta2 + theta);
    y = y1 + r*sinf(theta2 + theta);
}

int read_hdmap::ReadMapFile(char *pchMapDir, STR_MAP *pstMap)
{	
	double dLon      			= 0.0;
	double dLat      			= 0.0;
	int    i    				= 0;
	int    j					= 0;	
	STR_POINT2F tmpstrLocation[1] = {0};
	int ret = -1;

	if (NULL == pchMapDir)
	{
        //printf("Map Dir error\n");
		return ret;
	}
	 
	FILE *pFd  = NULL;
    pFd = fopen(pchMapDir, "rb");
	if (NULL == pFd)
	{
        //printf("Open Map file failed\n");
		return ret;
	}
	
	if (!feof(pFd))
	{
		ret = fread(pstMap->version, sizeof(char) * MAP_VER_LEN, 1, pFd);
		ret = fread(&dLon, sizeof(double), 1, pFd);
		ret = fread(&dLat, sizeof(double), 1, pFd);
        //TRACE_INFO("verison:%s\n", pstMap->version);
        //TRACE_INFO("dLon = %lf, dLat = %lf\n", dLon, dLat);

        pstMap->strAnchorLonLat.dLat = dLat;
        pstMap->strAnchorLonLat.dLon = dLon;

        g_dLat = dLat;
        g_dLon = dLon;

        std::string WGS84 = "+proj=longlat +datum=WGS84 +no_defs";
        //std::string TMERC = "+proj=tmerc +lat_0=22.6656342253927n  +lon_0=114.054429823102 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";

        char str[20];
        sprintf(str, "%.13lf", dLon);
        std::string strLon = str;
        sprintf(str, "%.13lf", dLat);
        std::string strLat = str;

        std::string TMERC = "+proj=tmerc +lat_0="+strLat+"n  +lon_0=" + strLon  +" +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
        //convert_coordiante = new CoordinateConvertTool(WGS84, TMERC);
		ret = fread(&(pstMap->iCountStation), sizeof(int), 1, pFd);
		//qDebug()<<pstMap->iCountStation<<"pstMap->iCountStation";
        //////TRACE_INFO("iCountStation:%d\n", pstMap->iCountStation);
		for (i = 0; i < pstMap->iCountStation; i++)
		{
			ret = fread(&(pstMap->pstrStation[i].iId), sizeof(int), 1, pFd);
            ////TRACE_INFO("pstrStation->iId:%d\n", pstMap->pstrStation->iId);
            //qDebug()<<pstMap->pstrStation[i].iId<<"pstMap->id";
			ret = fread(&tmpstrLocation, sizeof(STR_POINT2F), 1, pFd);
			memcpy(&(pstMap->pstrStation[i].strLocation), tmpstrLocation, sizeof(tmpstrLocation));			
            ////TRACE_INFO("strLocation.fX:%f strLocation.fY:%f\n", pstMap->pstrStation->strLocation.fX, pstMap->pstrStation->strLocation.fY);
            //qDebug()<<tmpstrLocation->fX<<tmpstrLocation->fY<<"tmpstrLocation.fX";
			//qDebug()<<pstMap->pstrStation->strLocation.fX<<pstMap->pstrStation->strLocation.fY<<"pstMap->pstrStation->strLocation.fX";
			ret = fread(&(pstMap->pstrStation[i].strPolygon.iCount), sizeof(int), 1, pFd);
            ////TRACE_INFO("pstMap->pstrStation->strPolygon.iCount:%d\n", pstMap->pstrStation->strPolygon.iCount);
			for (j = 0; j < pstMap->pstrStation[i].strPolygon.iCount; j++)
			{
				ret = fread(&(pstMap->pstrStation[i].strPolygon.pstrPoint[j].fX), sizeof(float), 1, pFd);
                ////TRACE_INFO("pstMap->pstrStation->strPolygon.pstrPoint->fX:%f\n", pstMap->pstrStation[i].strPolygon.pstrPoint[j].fX);
				ret = fread(&(pstMap->pstrStation[i].strPolygon.pstrPoint[j].fY), sizeof(float), 1, pFd);
                ////TRACE_INFO("pstMap->pstrStation->strPolygon.pstrPoint->fY:%f\n", pstMap->pstrStation[i].strPolygon.pstrPoint[j].fY);
			}
			
			ret = fread(&(pstMap->pstrStation->iCount), sizeof(int), 1, pFd);
            //TRACE_INFO("pstMap->pstrStation->iCount:%d\n", pstMap->pstrStation->iCount);
			for (j = 0; j < pstMap->pstrStation->iCount; j++)
			{
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].fTheta), sizeof(float), 1, pFd);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace->fTheta:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].fTheta);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[0].fX), sizeof(float), 1, pFd);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[0].fY), sizeof(float), 1, pFd);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[0].fX:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[0].fX);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[0].fY:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[0].fY);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[1].fX), sizeof(float), 1, pFd);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[1].fY), sizeof(float), 1, pFd);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[1].fX:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[1].fX);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[1].fY:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[1].fY);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[2].fX), sizeof(float), 1, pFd);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[2].fY), sizeof(float), 1, pFd);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[2].fX:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[2].fX);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[2].fY:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[2].fY);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[3].fX), sizeof(float), 1, pFd);
				ret = fread(&(pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[3].fY), sizeof(float), 1, pFd);
				////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[3].fX:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[3].fX);
                ////TRACE_INFO("pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[3].fY:%f\n", pstMap->pstrStation[i].pstrParkingSpace[j].pstrPoint[3].fY);
			}
		}

		/* Lane */
		ret = fread(&(pstMap->iCountLane), sizeof(int), 1, pFd);
        //TRACE_INFO("iCountLane:%d\n", pstMap->iCountLane);
        //qDebug()<<pstMap->iCountLane<<"pstMap->iCountLane";
		STR_POINT5f_LANE_CENTER_POINT strCenter[LANE_SAMPLE]= {0};
		int iCountAraay[10] = {0};
		for (i = 0; i < pstMap->iCountLane; i++)
        {
			ret = fread(&(pstMap->pstrLane[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].iCountCenter), sizeof(int), 1, pFd);
            //qDebug()<<pstMap->pstrLane[i].iId<<"pstMap->pstrLane[i].iId"<<pstMap->pstrLane[i].iCountCenter;
            ////TRACE_INFO("id:%d iCountCenter:%d\n", pstMap->pstrLane[i].iId, pstMap->pstrLane[i].iCountCenter);
            
			memset(strCenter, 0, sizeof(strCenter));
			for (j = 0; j < pstMap->pstrLane[i].iCountCenter; j++)
			{
				ret = fread(&(strCenter[j]), sizeof(STR_POINT5f_LANE_CENTER_POINT), 1, pFd);
                ////TRACE_INFO("strCenter.fX%f strCenter.fY%f strCenter.fS %f strCenter.fTheta %f strCenter.fKa %f \n", strCenter[j].fX, strCenter[j].fY, strCenter[j].fS, strCenter[j].fTheta, strCenter[j].fKa);
                ////TRACE_INFO("strCenter.fX:%f strCenter.fY:%f \n", strCenter[j].fX, strCenter[j].fY);
                //qDebug()<<strCenter[j].fTheta<<i + 1;
			}
            
			memcpy(pstMap->pstrLane[i].pstrCenter, strCenter, sizeof(STR_POINT5f_LANE_CENTER_POINT) * pstMap->pstrLane[i].iCountCenter);
            ////TRACE_INFO("fX:%f fY:%f\n", (pstMap->pstrLane[i].pstrCenter[0].fX), (pstMap->pstrLane[i].pstrCenter[0].fY));

			ret = fread(&(pstMap->pstrLane[i].fLen), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].fWidth), sizeof(float), 1, pFd);
            ////TRACE_INFO("fLen:%f fWidth:%f\n", pstMap->pstrLane[i].fLen, pstMap->pstrLane[i].fWidth);
			ret = fread(&(pstMap->pstrLane[i].emType), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].emTurn), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].emDirection), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].iLimitSpeed), sizeof(int), 1, pFd);
			//qDebug()<<pstMap->pstrLane[i].iLimitSpeed<<pstMap->pstrLane[i].fWidth;
            ////TRACE_INFO("emType:%d emTurn:%d emDirection:%d iLimitSpeed:%d\n", pstMap->pstrLane[i].emType, pstMap->pstrLane[i].emTurn, pstMap->pstrLane[i].emDirection, pstMap->pstrLane[i].iLimitSpeed);
			ret = fread(&(pstMap->pstrLane[i].iSationId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].iReverseId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].iLeftId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrLane[i].iRightId), sizeof(int), 1, pFd);
            ////TRACE_INFO("iSationId:%d iReverseId:%d iLeftId:%d iRightId:%d\n", pstMap->pstrLane[i].iSationId, pstMap->pstrLane[i].iReverseId, pstMap->pstrLane[i].iLeftId, pstMap->pstrLane[i].iRightId);
			ret = fread(&(pstMap->pstrLane[i].iCountIncome), sizeof(int), 1, pFd);
            ////TRACE_INFO("iCountIncome:%d\n", pstMap->pstrLane[i].iCountIncome);
            
			for (j = 0; j < pstMap->pstrLane[i].iCountIncome; j++)
			{

				ret = fread(&(iCountAraay[j]), sizeof(int), 1, pFd);
                //////TRACE_INFO("iCountAraay:%d\n", iCountAraay[j]);
                //qDebug()<< iCountAraay[j]<<i + 1<<"in";
			}
			memcpy(pstMap->pstrLane[i].piIncomeLaneId, iCountAraay, sizeof(int) * pstMap->pstrLane[i].iCountIncome);
            ////TRACE_INFO("piIncomeLaneId:%d\n", pstMap->pstrLane[i].piIncomeLaneId);
			//qDebug()<<"1112";
			//qDebug()<< pstMap->pstrLane[i].piIncomeLaneId<<i + 1<<"in";
			ret = fread(&(pstMap->pstrLane[i].iCountOutgo), sizeof(int), 1, pFd);
            ////TRACE_INFO("iCountOutgo:%d\n", pstMap->pstrLane[i].iCountOutgo);
			for (j = 0; j < pstMap->pstrLane[i].iCountOutgo; j++)
			{
				ret = fread(&(iCountAraay[j]), sizeof(int), 1, pFd);
                ////TRACE_INFO("iCountAraay:%d\n", iCountAraay[j]);
                //qDebug()<< iCountAraay[j]<<i + 1<<"out";
			}
			//qDebug()<<"1113";
			memcpy(pstMap->pstrLane[i].piOutgoLaneId, iCountAraay, sizeof(int) * pstMap->pstrLane[i].iCountOutgo);
            ////TRACE_INFO("piOutgoLaneId:%d\n", pstMap->pstrLane[i].piOutgoLaneId);
            //qDebug()<< pstMap->pstrLane[i].piOutgoLaneId<<i + 1;
		}

		/* stopline */
		ret = fread(&(pstMap->iCountStopLine), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountStopLine:%d\n", pstMap->iCountStopLine);
		for (i = 0; i < pstMap->iCountStopLine; i++)
		{
			ret = fread(&(pstMap->pstrStopLine[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrStopLine[i].pstrPoint[0].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrStopLine[i].pstrPoint[0].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrStopLine[i].pstrPoint[1].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrStopLine[i].pstrPoint[1].fY), sizeof(float), 1, pFd);
            ////TRACE_INFO("fX[0]:%f fY[0]%f fX[1]:%f fY[1]%f \n", pstMap->pstrStopLine[i].pstrPoint[0].fX, pstMap->pstrStopLine[i].pstrPoint[0].fY, pstMap->pstrStopLine[i].pstrPoint[1].fX, pstMap->pstrStopLine[i].pstrPoint[1].fY);
		}

		/* crosswalk */
		ret = fread(&(pstMap->iCountCrosswalk), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountCrosswalk:%d\n", pstMap->iCountCrosswalk);
		for (i = 0; i < pstMap->iCountCrosswalk; i++)
		{
			ret = fread(&(pstMap->pstrCrosswalk[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[0].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[0].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[1].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[1].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[2].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[2].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[3].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrCrosswalk[i].pstrPoint[3].fY), sizeof(float), 1, pFd);
            ////TRACE_INFO("fX[0]:%f fY[0]%f fX[1]:%f fY[1]%f \n", pstMap->pstrCrosswalk[i].pstrPoint[0].fX, pstMap->pstrCrosswalk[i].pstrPoint[0].fY, pstMap->pstrCrosswalk[i].pstrPoint[1].fX, pstMap->pstrCrosswalk[i].pstrPoint[1].fY);
            ////TRACE_INFO("fX[2]:%f fY[2]%f fX[3]:%f fY[3]%f \n", pstMap->pstrCrosswalk[i].pstrPoint[2].fX, pstMap->pstrCrosswalk[i].pstrPoint[2].fY, pstMap->pstrCrosswalk[i].pstrPoint[3].fX, pstMap->pstrCrosswalk[i].pstrPoint[3].fY);
		}

		/* Signal */
		//STR_POINT3F
		ret = fread(&(pstMap->iCountSignal), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountSignal:%d\n", pstMap->iCountSignal);
		for (i = 0; i < pstMap->iCountSignal; i++)
		{
			ret = fread(&(pstMap->pstrSignal[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSignal[i].strLocation.fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrSignal[i].strLocation.fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrSignal[i].strLocation.fZ), sizeof(float), 1, pFd);
            ////TRACE_INFO("Signal iId:%d fX: %f fY: %f fZ: %f\n", pstMap->pstrSignal[i].iId, pstMap->pstrSignal[i].strLocation.fX, pstMap->pstrSignal[i].strLocation.fY, pstMap->pstrSignal[i].strLocation.fZ);

			ret = fread(&(pstMap->pstrSignal[i].iCountSubSignal), sizeof(int), 1, pFd);
            ////TRACE_INFO("iCountSubSignal:%d\n", pstMap->pstrSignal[i].iCountSubSignal);
			for (j = 0; j < pstMap->pstrSignal[i].iCountSubSignal; j++)
			{
				ret = fread(&(pstMap->pstrSignal[i].pstrSubSignal[j].emShape), sizeof(int), 1, pFd);
				ret = fread(&(pstMap->pstrSignal[i].pstrSubSignal[j].emArrow), sizeof(int), 1, pFd);
                //TRACE_INFO("SubSignal emShape:%d SubSignal emArrow:%d\n", pstMap->pstrSignal[i].pstrSubSignal[j].emShape, pstMap->pstrSignal[i].pstrSubSignal[j].emArrow);
			}
		}
		/* ClearArea */
		ret = fread(&(pstMap->iCountClearArea), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountClearArea:%d\n", pstMap->iCountClearArea);
		for (i = 0; i < pstMap->iCountClearArea; i++)
		{
			ret = fread(&(pstMap->pstrClearArea[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[0].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[0].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[1].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[1].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[2].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[2].fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[3].fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrClearArea[i].pstrPoint[3].fY), sizeof(float), 1, pFd);
            ////TRACE_INFO("fX[0]:%f fY[0]%f fX[1]:%f fY[1]%f \n", pstMap->pstrClearArea[i].pstrPoint[0].fX, pstMap->pstrClearArea[i].pstrPoint[0].fY, pstMap->pstrClearArea[i].pstrPoint[1].fX, pstMap->pstrClearArea[i].pstrPoint[1].fY);
            ////TRACE_INFO("fX[2]:%f fY[2]%f fX[3]:%f fY[3]%f \n", pstMap->pstrClearArea[i].pstrPoint[2].fX, pstMap->pstrClearArea[i].pstrPoint[2].fY, pstMap->pstrClearArea[i].pstrPoint[3].fX, pstMap->pstrClearArea[i].pstrPoint[3].fY);
		}

		/* TrafficSign */
		ret = fread(&(pstMap->iCountTrafficSign), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountTrafficSign:%d\n", pstMap->iCountTrafficSign);
		for (i = 0; i < pstMap->iCountTrafficSign; i++)
		{
			ret = fread(&(pstMap->pstrTrafficSign[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrTrafficSign[i].strLocation.fX), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrTrafficSign[i].strLocation.fY), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrTrafficSign[i].strLocation.fZ), sizeof(float), 1, pFd);
			ret = fread(&(pstMap->pstrTrafficSign[i].iCount), sizeof(int), 1, pFd);
            ////TRACE_INFO("TrafficSign iId:%d fX: %f fY: %f fZ: %f\n", pstMap->pstrTrafficSign[i].iId, pstMap->pstrTrafficSign[i].strLocation.fX, pstMap->pstrTrafficSign[i].strLocation.fY, pstMap->pstrTrafficSign[i].strLocation.fZ);

			for (j = 0; j < pstMap->pstrTrafficSign[i].iCount; j++)
			{
				ret = fread(&(pstMap->pstrTrafficSign[i].pemType[j]), sizeof(int), 1, pFd);	
                ////TRACE_INFO("pemType:%d \n", pstMap->pstrTrafficSign[i].pemType[j]);
			}
		}

		/* Section */
		STR_POINT2F strLef[SECTION_SAMPLE] = {0};
		STR_POINT2F strRight[SECTION_SAMPLE] = {0};
		int iSLaneCount[10] = {0};
		int iSectionICount[10] = {0};
		int iSectionOCount[10] = {0};
		ret = fread(&(pstMap->iCountSection), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountSection:%d\n", pstMap->iCountSection);
        //qDebug()<<pstMap->iCountSection<<"pstMap->iCountSection";
		for (i = 0; i < pstMap->iCountSection; i++)
		{
			ret = fread(&(pstMap->pstrSection[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iCountLane), sizeof(int), 1, pFd);
            //////TRACE_INFO("Section iId:%d iCountLane:%d\n", pstMap->pstrSection[i].iId, pstMap->pstrSection[i].iCountLane);
			for (j = 0; j < pstMap->pstrSection[i].iCountLane; j++)
			{
				ret = fread(&(iSLaneCount[j]), sizeof(int), 1, pFd);
			}
			memcpy(pstMap->pstrSection[i].piLaneId, iSLaneCount, sizeof(int) * pstMap->pstrSection[i].iCountLane);
            //////TRACE_INFO("piLaneId:%d\n", pstMap->pstrSection[i].piLaneId);

			ret = fread(&(pstMap->pstrSection[i].iCountLeftRight), sizeof(int), 1, pFd);
            //////TRACE_INFO("iCountLeftRight: %d\n", pstMap->pstrSection[i].iCountLeftRight);
             //qDebug()<<pstMap->pstrSection[i].iCountLeftRight;
			for (j = 0; j < pstMap->pstrSection[i].iCountLeftRight; j++)
			{
				ret = fread(&(strLef[j]), sizeof(STR_POINT2F), 1, pFd);
                ////TRACE_INFO("strLef.fX %f strLef.fY %f \n", strLef[j].fX, strLef[j].fY);
                //qDebug()<<strLef[j].fX<<strLef[j].fY<<"strLef.fY";
			}
			memcpy(pstMap->pstrSection[i].pstrLeft, strLef, sizeof(STR_POINT2F) * pstMap->pstrSection[i].iCountLeftRight);

			for (j = 0; j < pstMap->pstrSection[i].iCountLeftRight; j++)
			{
				ret = fread(&(strRight[j]), sizeof(STR_POINT2F), 1, pFd);
				//qDebug()<<strRight[j].fX<<strRight[j].fY<<"strRight.fY";
                ////TRACE_INFO("strRight.fY %f strRight.fY %f\n", strRight[j].fX, strRight[j].fY);
			}
			memcpy(pstMap->pstrSection[i].pstrRight, strRight, sizeof(STR_POINT2F) * pstMap->pstrSection[i].iCountLeftRight);

			ret = fread(&(pstMap->pstrSection[i].emLeftMedian), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].emRightMedian), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iStopLineId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iCrosswalkId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iSignalId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iClearAreaId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iLeftSectionId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iRightSectionId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iReverseId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrSection[i].iCountTrafficSign), sizeof(int), 1, pFd);
			for (j = 0; j < pstMap->pstrSection[i].iCountTrafficSign; j++)
			{
				ret = fread(&(pstMap->pstrSection[i].piTrafficSignId[j]), sizeof(int), 1, pFd);
			}

			ret = fread(&(pstMap->pstrSection[i].iCountIncome), sizeof(int), 1, pFd);
			for (j = 0; j < pstMap->pstrSection[i].iCountIncome; j++)
			{
				ret = fread(&(iSectionICount[j]), sizeof(int), 1, pFd);
			}
			memcpy(pstMap->pstrSection[i].piIncomeSectionId, iSectionICount, sizeof(int) * pstMap->pstrSection[i].iCountIncome);
            //////TRACE_INFO("piIncomeSectionId:%d\n", pstMap->pstrSection[i].piIncomeSectionId);

			ret = fread(&(pstMap->pstrSection[i].iCountOutgo), sizeof(int), 1, pFd);
			for (j = 0; j < pstMap->pstrSection[i].iCountOutgo; j++)
			{
				ret = fread(&(iSectionOCount[j]), sizeof(int), 1, pFd);
			}
			memcpy(pstMap->pstrSection[i].piOutgoSectionId, iSectionOCount, sizeof(int) * pstMap->pstrSection[i].iCountOutgo);
            //////TRACE_INFO("piOutgoSectionId:%d\n", pstMap->pstrSection[i].piOutgoSectionId);
		}

		/* road */
		int iRoadSecitonCount[10] = {0};
		int iRoadICount[10] = {0};
		int iRoadOCount[10] = {0};
		ret = fread(&(pstMap->iCountRoad), sizeof(int), 1, pFd);
        //////TRACE_INFO("iCountRoad:%d\n", pstMap->iCountRoad);
        //qDebug()<<pstMap->iCountRoad<<"pstMap->iCountRoad";
		for (i = 0; i < pstMap->iCountRoad; i++)
		{
			ret = fread(&(pstMap->pstrRoad[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrRoad[i].iReverseId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrRoad[i].iCountSection), sizeof(int), 1, pFd);
			//qDebug()<<pstMap->pstrRoad[i].iId<<pstMap->pstrRoad[i].iCountSection;
			for (j = 0; j < pstMap->pstrRoad[i].iCountSection; j++)
			{
				ret = fread(&(iRoadSecitonCount[j]), sizeof(int), 1, pFd);
			}
			memcpy(pstMap->pstrRoad[i].piSectionId, iRoadSecitonCount, sizeof(int) * pstMap->pstrRoad[i].iCountSection);
            //////TRACE_INFO("piSectionId:%d\n", pstMap->pstrRoad[i].piSectionId);
            //myDebug();
			ret = fread(&(pstMap->pstrRoad[i].iCountIncome), sizeof(int), 1, pFd);
			for (j = 0; j < pstMap->pstrRoad[i].iCountIncome; j++)
			{
				ret = fread(&(iRoadICount[j]), sizeof(int), 1, pFd);
			}
			memcpy(pstMap->pstrRoad[i].piIncomeRoadId, iRoadICount, sizeof(int) * pstMap->pstrRoad[i].iCountIncome);
            //////TRACE_INFO("piIncomeRoadId:%d\n", pstMap->pstrRoad[i].piIncomeRoadId);
            //myDebug();
			ret = fread(&(pstMap->pstrRoad[i].iCountOutgo), sizeof(int), 1, pFd);
			for (j = 0; j < pstMap->pstrRoad[i].iCountOutgo; j++)
			{
				ret = fread(&(iRoadOCount[j]), sizeof(int), 1, pFd);
			}
			memcpy(pstMap->pstrRoad[i].piOutgoRoadId, iRoadOCount, sizeof(int) * pstMap->pstrRoad[i].iCountOutgo);
            //////TRACE_INFO("iCountOutgo:%d\n", pstMap->pstrRoad[i].piOutgoRoadId);
            //myDebug();
			ret = fread(&(pstMap->pstrRoad[i].iOutgoJunctionId), sizeof(int), 1, pFd);
            //////TRACE_INFO("iOutgoJunctionId:%d\n", pstMap->pstrRoad[i].iOutgoJunctionId);
		}

		/* junction*/
		int iJuncitonICount[10] = {0};
		int iJuncitonOCount[10] = {0};
		int iJuncitonCCount[10] = {0};
		ret = fread(&(pstMap->iCountJunction), sizeof(int), 1, pFd);
        ////TRACE_INFO("iCountJunction:%d\n", pstMap->iCountJunction);
        //qDebug()<<pstMap->iCountJunction<<"pstMap->iCountJunction";
		for (i = 0; i < pstMap->iCountJunction; i++)
		{
			ret = fread(&(pstMap->pstrJunction[i].iId), sizeof(int), 1, pFd);
			ret = fread(&(pstMap->pstrJunction[i].strPloygon.iCount), sizeof(int), 1, pFd);
            ////TRACE_INFO("iId:%d iCount:%d\n", pstMap->pstrJunction[i].iId, pstMap->pstrJunction[i].strPloygon.iCount);
			for (j = 0; j < pstMap->pstrJunction[i].strPloygon.iCount; j++)
			{
				ret = fread(&(pstMap->pstrJunction[i].strPloygon.pstrPoint[j].fX), sizeof(float), 1, pFd);
				ret = fread(&(pstMap->pstrJunction[i].strPloygon.pstrPoint[j].fY), sizeof(float), 1, pFd);
                ////TRACE_INFO("fX:%f fY:%f\n", pstMap->pstrJunction[i].strPloygon.pstrPoint[j].fX, pstMap->pstrJunction[i].strPloygon.pstrPoint[j].fY);
			}

			ret = fread(&(pstMap->pstrJunction[i].iCountIncome), sizeof(int), 1, pFd);
            ////TRACE_INFO("iCountIncome:%d\n", pstMap->pstrJunction[i].iCountIncome);
			for (j = 0; j < pstMap->pstrJunction[i].iCountIncome; j++)
			{
				ret = fread(&(iJuncitonICount[j]), sizeof(int), 1, pFd);
                ////TRACE_INFO("iJuncitonICount:%d\n", iJuncitonICount[j]);
			}
            //qDebug()<<"pstMap->pstrJunction[i].iCountIncome"<<pstMap->pstrJunction[i].iCountIncome;
			memcpy(pstMap->pstrJunction[i].piIncomeRoadId, iJuncitonICount, sizeof(int) * pstMap->pstrJunction[i].iCountIncome);
            ////TRACE_INFO("piIncomeRoadId:%d\n", pstMap->pstrJunction[i].piIncomeRoadId[0]);

			ret = fread(&(pstMap->pstrJunction[i].iCountOutgo), sizeof(int), 1, pFd);
            ////TRACE_INFO("iCountOutgo:%d\n", pstMap->pstrJunction[i].iCountOutgo);
			for (j = 0; j < pstMap->pstrJunction[i].iCountOutgo; j++)
			{
				ret = fread(&(iJuncitonOCount[j]), sizeof(int), 1, pFd);
                ////TRACE_INFO("iJuncitonOCount:%d\n", iJuncitonOCount[j]);
			}
            //qDebug()<<"pstMap->pstrJunction[i].iCountOutgo"<<pstMap->pstrJunction[i].iCountOutgo;
			memcpy(pstMap->pstrJunction[i].piOutgoRoadId, iJuncitonOCount, sizeof(int) * pstMap->pstrJunction[i].iCountOutgo);
            ////TRACE_INFO("piOutgoRoadId:%d\n", pstMap->pstrJunction[i].piOutgoRoadId);

			ret = fread(&(pstMap->pstrJunction[i].iCountConnect), sizeof(int), 1, pFd);
            ////TRACE_INFO("iCountConnect:%d\n", pstMap->pstrJunction[i].iCountConnect);
			for (j = 0; j < pstMap->pstrJunction[i].iCountConnect; j++)
			{
				ret = fread(&(iJuncitonCCount[j]), sizeof(int), 1, pFd);
                ////TRACE_INFO("iJuncitonCCount:%d\n", iJuncitonCCount[j]);
			}
            //qDebug()<<"pstMap->pstrJunction[i].iCountConnect"<<pstMap->pstrJunction[i].iCountConnect;
			memcpy(pstMap->pstrJunction[i].piConnectRoadId, iJuncitonCCount, sizeof(int) * pstMap->pstrJunction[i].iCountConnect);
            ////TRACE_INFO("piConnectRoadId:%d\n", pstMap->pstrJunction[i].piConnectRoadId);
            //qDebug()<<"pstMap->iCountJunction1111";
		}		
	}
    //qDebug()<<"readmap success";
	fclose(pFd);
	pFd = NULL;
	return ret;
}

#if 0
STR_ID_ARRAY read_hdmap::GetLaneIdByPoint(STR_POINT2F strXY)
{
	STR_ID_ARRAY ret;
	int i 										  = 0;
	int j 										  = 0;
	float fTmpDis 								  = 0.0;
    float fMinDis 								  = 3.4e+38F;
	STR_POINT5f_LANE_CENTER_POINT strCenter[2000]  = {0};
    int dDistThresh = 8;
    int IterativeStep = 16;
	ret.iCount = 0;
    //ret.piID = new int[10];
    int idx = -1;
	//ret.piID = (int*)malloc(sizeof(int) * 10);
    ////TRACE_INFO("strXY.fX:%f strXY.fY:%f \n", strXY.fX, strXY.fY);
    ////TRACE_INFO("CountLane:%d\n", g_pstMap->iCountLane);
	#if 0
    for (i = 0; i < g_pstMap->iCountLane; i++)
    {
        if (((int)(g_pstMap->pstrLane[i].iCountCenter / IterativeStep)) < 3)
        {
            IterativeStep = (int)(IterativeStep / 8.0);
            dDistThresh = dDistThresh / 8.0;
        }

        memcpy(strCenter, g_pstMap->pstrLane[i].pstrCenter, sizeof(STR_POINT5f_LANE_CENTER_POINT) * g_pstMap->pstrLane[i].iCountCenter);
        j = 0;
        while(1)
        {
            if (IterativeStep < 1)
            {

                dDistThresh = 8;
                IterativeStep = 16;
                break;
            }

            if (j >= g_pstMap->pstrLane[i].iCountCenter)
            {
                IterativeStep = IterativeStep / 2.0;
            }
            else
            {
                fTmpDis = sqrt((strXY.fX - strCenter[j].fX) * (strXY.fX - strCenter[j].fX)
                         + (strXY.fY - strCenter[j].fY) * (strXY.fY - strCenter[j].fY));

                if (fTmpDis < dDistThresh)
                {
                    if (fTmpDis < 0.5)
                    {
                        ret.iCount += 1;
                        memcpy(&(ret.piID[ + ret.iCount - 1]), &(g_pstMap->pstrLane[i].iId), sizeof(int));
                        printf("fTmpDis:%f lanid:%d \n", fTmpDis, g_pstMap->pstrLane[i].iId);
                        dDistThresh = 8;
                        IterativeStep = 16;
                        break;
                    }
                    else
                    {
                        IterativeStep = int(IterativeStep / 2.0);
                        dDistThresh = dDistThresh / 2.0;
                        j = 0;
                    }
                }
                else
                {
                    j += IterativeStep;
                }
            }
        }
    }
	#endif 

    #if 1
	for (i = 0; i < g_pstMap->iCountLane; i++)
	{
		memset(strCenter, 0, sizeof(STR_POINT5f_LANE_CENTER_POINT) * g_pstMap->pstrLane[i].iCountCenter);
		memcpy(strCenter, g_pstMap->pstrLane[i].pstrCenter, sizeof(STR_POINT5f_LANE_CENTER_POINT) * g_pstMap->pstrLane[i].iCountCenter);
        ////TRACE_INFO("iCountCenter:%d \n", g_pstMap->pstrLane[i].iCountCenter);
		for (j = 0; j < g_pstMap->pstrLane[i].iCountCenter; j++)
		{
            ////TRACE_INFO("pstMap.fX:%f pstMap.fY:%f \n", &(pstMap->pstrLane[i].pstrCenter[j]).fX, &(pstMap->pstrLane[i].pstrCenter[j]).fY);
            ////TRACE_INFO("strCenter.fX%f strCenter.fY%f \n", strCenter[j].fX, strCenter[j].fY);
			fTmpDis = sqrt((strXY.fX - strCenter[j].fX) * (strXY.fX - strCenter[j].fX) 
				         + (strXY.fY - strCenter[j].fY) * (strXY.fY - strCenter[j].fY));			
			if (fTmpDis <= fMinDis)
			{
				fMinDis = fTmpDis;
				idx = i;
				//ret.iCount += 1;
				//ret.piID = &(g_pstMap->pstrLane[i].iId);
				//tmpid[ret.iCount - 1] = (g_pstMap->pstrLane[i].iId);
				//memcpy((ret.piID[ + ret.iCount - 1]), &(g_pstMap->pstrLane[i].iId), sizeof(int));
				//memcpy((ret.piID), &tmpid, sizeof(tmpid));
                ////TRACE_INFO("fTmpDis:%f \n", fTmpDis);
                ////TRACE_INFO("ret.iCount:%d tmpid: %d  ret.piID:%d ret.piID2:%d\n", ret.iCount, tmpid[ret.iCount - 1], *(ret.piID), *(ret.piID + 1));
			}
		}
        ////TRACE_INFO("i:====================%d CountCenter:====%d \n", i, g_pstMap->pstrLane[i].iCountCenter);

	}

	if (idx != -1)
    {
        ret.iCount += 1;
        ret.piID[ret.iCount - 1] = g_pstMap->pstrLane[idx].iId;
    }

    #endif


	return ret;
}

#endif

STR_ID_ARRAY read_hdmap::GetLaneIdByPoint(STR_POINT2F strXY)
{
    STR_ID_ARRAY ret;
    int i 										  = 0;
    int j 										  = 0;
    float fTmpDis 								  = 0.0;
    float fMinDis 								  = 3.4e+38F;

    ret.iCount = 0;
    int idx = -1;

	std::set<int> idxs;

	
	
    for (i = 0; i < g_pstMap->iCountLane; i++)
    {
		//cout<<"lane "<<i<<" center count "<<g_pstMap->pstrLane[i].iCountCenter<<endl;
        for (j = 0; j < g_pstMap->pstrLane[i].iCountCenter; j++)
        {
			

			float px = g_pstMap->pstrLane[i].pstrCenter[j].fX;
			float py = g_pstMap->pstrLane[i].pstrCenter[j].fY;

			fTmpDis = hypotf(strXY.fX - px, strXY.fY - py);

			if(fTmpDis <= 18)//18)
			{
				idxs.insert(i);
				//cout<<"getlaneidbypoint laneIDS:"<<g_pstMap->pstrLane[i].iId<<endl;
			}

			//find the nearest lane i
            if (fTmpDis - fMinDis< 0.0001) // 浮点型不能用=
            {
                fMinDis = fTmpDis;
                idx = i;
            }
			
        }
        //TRACE_INFO("i:====================%d CountCenter:====%d \n", i, g_pstMap->pstrLane[i].iCountCenter);

    }

	
	/*
    if (idx != -1)
    {
        ret.iCount += 1;
        ret.piID[ret.iCount - 1] = g_pstMap->pstrLane[idx].iId;
    }
	*/

    if (idxs.size() > 0)
    {
		std::set<int>::iterator iter = idxs.begin();
		for( ; iter != idxs.end(); iter++)
		{
			int idx = *iter;
			ret.iCount += 1;
			ret.piID[ret.iCount - 1] = g_pstMap->pstrLane[idx].iId;
		}

    }


    return ret;
}


int read_hdmap::GetSectionIdByLaneId(int iLaneId)
{
	int ret 		= 0;
	int i 			= 0;
	int j 			= 0;
	int iLaneid[10] = {0};

	for (i = 0; i < g_pstMap->iCountSection; i++)
	{
		memcpy(iLaneid, g_pstMap->pstrSection[i].piLaneId, sizeof(int) * g_pstMap->pstrSection[i].iCountLane);
		for (j = 0; j < g_pstMap->pstrSection[i].iCountLane; j ++)
		{
			if (iLaneId == iLaneid[j])
			{
				ret  = g_pstMap->pstrSection[i].iId;
			}
		}
	}
    //TRACE_INFO("Secitonid:%d \n", ret);
	
	return ret;
}

int read_hdmap::GetRoadIdBySectionId(int iSectionId)
{
	int ret 				= 0;
	int i 					= 0;
	int j 					= 0;
	int itmpSectionid[20] 	= {0};
	for (i = 0; i < g_pstMap->iCountRoad; i++)
	{
		memcpy(itmpSectionid, g_pstMap->pstrRoad[i].piSectionId, sizeof(int) * g_pstMap->pstrRoad[i].iCountSection);
		for (j = 0; j < g_pstMap->pstrRoad[i].iCountSection; j ++)
		{
			if (iSectionId == itmpSectionid[j])
			{
				ret = g_pstMap->pstrRoad[i].iId;
			}
		}
	}
    //TRACE_INFO("roadid:%d \n", ret);
	return ret;
}

int read_hdmap::GetJunctionByRoadId(int iRoadId)
{
	int ret 			= 0;
	int i 				= 0;
	int j 				= 0;
	int itmpRoadid[10] 	= {0};
	for (i = 0; i < g_pstMap->iCountJunction; i++)
	{
		memcpy(itmpRoadid, g_pstMap->pstrJunction[i].piConnectRoadId, sizeof(int) * g_pstMap->pstrJunction[i].iCountConnect);

		for (j = 0; j < g_pstMap->pstrJunction[i].iCountConnect; j ++)
		{
            ////TRACE_INFO("itmpRoadid:%d \n", itmpRoadid[j]);
			if (iRoadId == itmpRoadid[j])
			{
				ret = g_pstMap->pstrJunction[i].iId;
			}
		}
	}
    //TRACE_INFO("Juncitonid:%d \n", ret);
	return ret;
}

STR_POINT2F_ARRAY * read_hdmap::GetLBoundaryByLaneId(int iLaneId)
{
	STR_POINT2F_ARRAY *pLBoundary;
	int index = -1;
	float fwidth = 0.0;
    float frontfx[LANE_SAMPLE] = {0.0};
    float frontfy[LANE_SAMPLE] = {0.0};
	STR_POINT2F strfxfy[LANE_SAMPLE] = {0.0};
	STR_POINT5f_LANE_CENTER_POINT strcent[LANE_SAMPLE];

    pLBoundary = new STR_POINT2F_ARRAY();
    pLBoundary->pstrPoint = new STR_POINT2F[LANE_SAMPLE];
    memset(strcent, 0.0, sizeof(strcent));

    index = iLaneId - 1;
	fwidth = g_pstMap->pstrLane[index].fWidth;
	pLBoundary->iCount = g_pstMap->pstrLane[index].iCountCenter;

	if (pLBoundary->iCount <= 0)
	{
		strfxfy->fX = 0.0;
		strfxfy->fY = 0.0;
		memcpy(pLBoundary->pstrPoint, strfxfy, sizeof(STR_POINT2F));
		return pLBoundary;
	}
	for (int i = 0; i < pLBoundary->iCount; i++)
	{
		strcent[i].fX = g_pstMap->pstrLane[index].pstrCenter[i].fX;
		strcent[i].fY = g_pstMap->pstrLane[index].pstrCenter[i].fY;
        strcent[i].fTheta = 90 - g_pstMap->pstrLane[index].pstrCenter[i].fTheta;
		
        frontfx[i] = strcent[i].fX + fwidth/2*cosf(strcent[i].fTheta*pi/180);
        frontfy[i] = strcent[i].fY + fwidth/2*sinf(strcent[i].fTheta*pi/180);
        rotateAsPoint(strcent[i].fX, strcent[i].fY, frontfx[i], frontfy[i], pi/2, strfxfy[i].fX, strfxfy[i].fY);
	}
	memcpy(pLBoundary->pstrPoint, strfxfy, sizeof(STR_POINT2F) * (pLBoundary->iCount));
    //for (int i = 0; i < pLBoundary->iCount; i++)
    //{
        //printf("fTheta:%f \n", g_pstMap->pstrLane[index].pstrCenter[i].fTheta);
    //}
    //TRACE_INFO("icentercount:%d\n", pLBoundary->iCount);
	
	return pLBoundary;
}

STR_POINT2F_ARRAY * read_hdmap::GetRBoundaryByLaneId(int iLaneId)
{
	STR_POINT2F_ARRAY *pRBoundary;
	int index = -1;
	float fwidth = 0.0;
	float frontfx[LANE_SAMPLE] = {0.0};
	float frontfy[LANE_SAMPLE] = {0.0};
	STR_POINT2F strfxfy[LANE_SAMPLE] = {0.0};
	STR_POINT5f_LANE_CENTER_POINT strcent[LANE_SAMPLE];

	pRBoundary = new STR_POINT2F_ARRAY();
	pRBoundary->pstrPoint = new STR_POINT2F[LANE_SAMPLE];
	memset(strcent, 0.0, sizeof(strcent));

	index = iLaneId - 1;
	fwidth = g_pstMap->pstrLane[index].fWidth;
    pRBoundary->iCount = g_pstMap->pstrLane[index].iCountCenter;

	if (pRBoundary->iCount <= 0)
	{
		strfxfy->fX = 0.0;
		strfxfy->fY = 0.0;
		memcpy(pRBoundary->pstrPoint, strfxfy, sizeof(STR_POINT2F));
		return pRBoundary;
	}
	for (int i = 0; i < pRBoundary->iCount; i++)
	{
		strcent[i].fX = g_pstMap->pstrLane[index].pstrCenter[i].fX;
		strcent[i].fY = g_pstMap->pstrLane[index].pstrCenter[i].fY;
		strcent[i].fTheta = 90 - g_pstMap->pstrLane[index].pstrCenter[i].fTheta;
		
        frontfx[i] = strcent[i].fX + fwidth/2*cosf(strcent[i].fTheta*pi/180);
        frontfy[i] = strcent[i].fY + fwidth/2*sinf(strcent[i].fTheta*pi/180);
        rotateAsPoint(strcent[i].fX, strcent[i].fY, frontfx[i], frontfy[i], -pi/2, strfxfy[i].fX, strfxfy[i].fY);
	}
	memcpy(pRBoundary->pstrPoint, strfxfy, sizeof(STR_POINT2F) * (pRBoundary->iCount));
	//for (int i = 0; i < pRBoundary->iCount; i++)
	//{
    //	//TRACE_INFO("i:%d fx:%f fy:%f \n", i, pRBoundary->pstrPoint[i].fX, pRBoundary->pstrPoint[i].fY);
	//}
    //TRACE_INFO("icentercount:%d\n", pRBoundary->iCount);
	
	return pRBoundary;
}

STR_POINT2F_ARRAY * read_hdmap::GetLBoundaryBySectionId(int iSectionId)
{
	STR_POINT2F_ARRAY *pLBoundary;
	STR_POINT2F strLeft[SECTION_SAMPLE] = {0};
	int itmpCount = 0;

	//pLBoundary = (STR_POINT2F_ARRAY*)malloc(sizeof(STR_POINT2F_ARRAY) * 1000);
	pLBoundary = new STR_POINT2F_ARRAY();
	pLBoundary->pstrPoint = new STR_POINT2F[SECTION_SAMPLE];
	memcpy(strLeft, g_pstMap->pstrSection[iSectionId].pstrLeft, sizeof(STR_POINT2F) * g_pstMap->pstrSection[iSectionId].iCountLeftRight);
	memcpy(&itmpCount, &(g_pstMap->pstrSection[iSectionId].iCountLeftRight), sizeof(int));
    //TRACE_INFO("itmpCount: %d \n", itmpCount);
	//for (int i = 0; i < g_pstMap->pstrSection[iSectionId].iCountLeftRight; i++)
	//{
    //	//TRACE_INFO("strLef.fX: %f strLef.fY: %f \n", strLeft[i].fX, strLeft[i].fY);
	//}

	memcpy(&(pLBoundary->iCount), &itmpCount, sizeof(int));
    //TRACE_INFO("pLBoundary->iCount: %d \n", pLBoundary->iCount);
	memcpy(pLBoundary->pstrPoint, strLeft, sizeof(strLeft));

	return pLBoundary;
}

STR_POINT2F_ARRAY * read_hdmap::GetRBoundaryBySectionId(int iSectionId)
{
	STR_POINT2F_ARRAY *pRBoundary;
	
	STR_POINT2F strRight[SECTION_SAMPLE] = {0};
	int itmpCount = 0;

	//pRBoundary = (STR_POINT2F_ARRAY*)malloc(sizeof(STR_POINT2F_ARRAY) * 1000);
	pRBoundary = new STR_POINT2F_ARRAY();
	pRBoundary->pstrPoint = new STR_POINT2F[SECTION_SAMPLE];
	
	memcpy(strRight, g_pstMap->pstrSection[iSectionId].pstrRight, sizeof(STR_POINT2F) * g_pstMap->pstrSection[iSectionId].iCountLeftRight);
	memcpy(&itmpCount, &(g_pstMap->pstrSection[iSectionId].iCountLeftRight), sizeof(int));
    //TRACE_INFO("itmpCount: %d \n", itmpCount);
    //for (int i = 0; i < g_pstMap->pstrSection[iSectionId].iCountLeftRight; i++)
    //{
        ////TRACE_INFO("strRight.fX: %f strRight.fY: %f \n", strRight[i].fX, strRight[i].fY);
    //}

	memcpy(&(pRBoundary->iCount), &itmpCount, sizeof(int));
    ////TRACE_INFO("pRBoundary->iCount: %d \n", pRBoundary->iCount);
	memcpy(pRBoundary->pstrPoint, strRight, sizeof(strRight));
	
	return pRBoundary;
}

STR_POINT2F_ARRAY * read_hdmap::GetLBoundaryByRoadId(int iRoadId)
{
	STR_POINT2F_ARRAY *pLBoundary;
	int index = 0;
    int isection[20];
	int icounsection = 0;
	int itmpcount = 0;
	STR_POINT2F strLeft[10000] = {0};

	index = iRoadId - 1;
	icounsection = g_pstMap->pstrRoad[index].iCountSection;
	//1.得到road里的sectionid列表
	//2.用section的边界组成road的边界
	memcpy(isection, g_pstMap->pstrRoad[index].piSectionId, sizeof(int) * icounsection);
	for (int i = 0; i < icounsection; i++)
	{
        //TRACE_INFO("iCountSection:%d isectionid %d\n", icounsection, isection[i]);
		memcpy(strLeft + itmpcount, g_pstMap->pstrSection[isection[i] -1].pstrLeft, sizeof(STR_POINT2F) * g_pstMap->pstrSection[isection[i]-1].iCountLeftRight);			
		itmpcount += g_pstMap->pstrSection[isection[i] -1].iCountLeftRight;		
	}
    ////TRACE_INFO("itmpcount%d \n", itmpcount);
	//for (int j = 0; j < itmpcount; j++)
	//{
        ////TRACE_INFO("j:%d fx:%f fy:%f \n", j, strLeft[j].fX, strLeft[j].fY);
	//}
    //myDebug();
	pLBoundary = new STR_POINT2F_ARRAY();
	pLBoundary->pstrPoint = new STR_POINT2F[10000];
	pLBoundary->iCount = itmpcount;
    //TRACE_INFO("pLBoundary->iCount: %d \n", pLBoundary->iCount);
	
	memcpy(pLBoundary->pstrPoint, strLeft, sizeof(STR_POINT2F) * itmpcount);
	//for (int i = 0 ; i < itmpcount; i++)
	//{
        ////TRACE_INFO("i:%d fx:%f fy:%f \n", i, pLBoundary->pstrPoint[i].fX, pLBoundary->pstrPoint[i].fY);
        ////TRACE_INFO("i:%d fx:%f fy:%f \n", i, strLeft[i].fX, strLeft[i].fY);
	//}
    //myDebug();
	return pLBoundary;
}

STR_POINT2F_ARRAY * read_hdmap::GetRBoundaryByRoadId(int iRoadId)
{
	STR_POINT2F_ARRAY *pRBoundary;
	int index = 0;
    int isection[20];
	int icounsection = 0;
	int itmpcount = 0;
	STR_POINT2F strRight[10000] = {0};

	index = iRoadId - 1;
	icounsection = g_pstMap->pstrRoad[index].iCountSection;
	//1.得到road里的sectionid列表
	//2.用section的边界组成road的边界
    //myDebug();
	memcpy(isection, g_pstMap->pstrRoad[index].piSectionId, sizeof(int) * icounsection);
	for (int i = 0; i < icounsection; i++)
	{
        //TRACE_INFO("iCountSection:%d isectionid %d\n", icounsection, isection[i]);
		memcpy(strRight + itmpcount, g_pstMap->pstrSection[isection[i] -1].pstrRight, sizeof(STR_POINT2F) * g_pstMap->pstrSection[isection[i]-1].iCountLeftRight);			
		itmpcount += g_pstMap->pstrSection[isection[i] -1].iCountLeftRight;		
	}
    ////TRACE_INFO("itmpcount%d \n", itmpcount);
	//for (int j = 0; j < itmpcount; j++)
	//{
        ////TRACE_INFO("j:%d fx:%f fy:%f \n", j, strRight[j].fX, strRight[j].fY);
	//}
    //myDebug();
	pRBoundary = new STR_POINT2F_ARRAY();
	pRBoundary->pstrPoint = new STR_POINT2F[10000];
	pRBoundary->iCount = itmpcount;
    //TRACE_INFO("pRBoundary->iCount: %d \n", pRBoundary->iCount);
    //myDebug();
	memcpy(pRBoundary->pstrPoint, strRight, sizeof(STR_POINT2F) * itmpcount);
	//for (int i = 0 ; i < itmpcount; i++)
	//{
        ////TRACE_INFO("i:%d fx:%f fy:%f \n", i, pRBoundary->pstrPoint[i].fX, pRBoundary->pstrPoint[i].fY);
        ////TRACE_INFO("i:%d fx:%f fy:%f \n", i, strRight[i].fX, strRight[i].fY);
	//}
    //myDebug();
	return pRBoundary;
}

STR_POINT2F read_hdmap::LonLat2XY(STR_LONLAT strLonLat, STR_LONLAT strAnchor)
{
//	float fX    	= 0.0;
//	float fY    	= 0.0;
//	double dLon 	= strLonLat.dLon;
//	double dLat 	= strLonLat.dLat;
	STR_POINT2F strXY;

	memset(&strXY, 0, sizeof(strXY));
//	convert_coordiante->ConvertLonLat2Cartesian(dLon, dLat, &fX, &fY);
//	strXY.fX = fX;
//	strXY.fY = fY;
//    ////TRACE_INFO("strXY.fX:%.12lf\nstrXY.fY:%.12lf \n", strXY.fX, strXY.fY);


    double anchorpoint[3]   = {0.0};
    double latlonpoint[3]   = {0.0};
    double xypoint[3]       = {0.0};
    anchorpoint[0]          = strAnchor.dLat;
    anchorpoint[1]          = strAnchor.dLon;
    anchorpoint[2]          = 0.0;//高度
    latlonpoint[0]          = strLonLat.dLat;
    latlonpoint[1]          = strLonLat.dLon;
    latlonpoint[2]          = 0.0;
//	qDebug()<<QString("%1").arg(strLonLat.dLat, 0, 'g', 14)<<QString("%2").arg(strLonLat.dLon, 0, 'g', 15)<<"befor";
	PointDeg2Enu(anchorpoint, latlonpoint, xypoint);
	strXY.fX = xypoint[0];
	strXY.fY = xypoint[1];
//	qDebug()<<QString("%1").arg(xypoint[0], 0, 'g', 14)<<QString("%2").arg(xypoint[1], 0, 'g', 15)<<"after";
	
	return strXY;
}

STR_LONLAT read_hdmap::XY2LonLat(STR_POINT2F strXY, STR_LONLAT strAnchor)
{
	STR_LONLAT strLonLat;
//	double dLon 	= 0.0;
//	double dLat     = 0.0;
//	float  fX       = strXY.fX;
//	float  fY		= strXY.fY;

	memset(&strLonLat, 0, sizeof(strLonLat));
//	convert_coordiante->ConvertCartesian2LonLat(fX, fY, &dLon, &dLat);
//	strLonLat.dLon = dLon;
//	strLonLat.dLat = dLat;
//    ////TRACE_INFO("strLonLat.dLon:%.12lf\nstrLonLat.dLat:%.12lf \n", strLonLat.dLon, strLonLat.dLat);

    double anchorpoint[3]   = {0.0};
    double xypoint[3]       = {0.0};
    double latlonpoint[3]   = {0.0};
    anchorpoint[0]          = strAnchor.dLat;
    anchorpoint[1]          = strAnchor.dLon;
    anchorpoint[2]          = 0.0;//高度
    xypoint[0]              = strXY.fX;
    xypoint[1]              = strXY.fY;
    xypoint[2]              = 0.0;
    PointEnu2Deg(anchorpoint, xypoint, latlonpoint);
    strLonLat.dLat = latlonpoint[0];
    strLonLat.dLon = latlonpoint[1];

	return strLonLat;
}

read_hdmap::read_hdmap()
{
	MallocMap();
	// // 路径参数化：不再在构造函数中硬编码 MAPDIR 自动加载，
	// // 由调用方通过 load(mapDir) 显式加载（失败可返回错误码）
	// char MapDir[DIR_LEN_MAX] = 	MAPDIR;
	// InitMap(MapDir);
}

/* 路径参数化加载：分配内存 + 解析地图文件 */
int read_hdmap::load(const char *mapDir)
{
	if (NULL == mapDir)
	{
		return -1;
	}
	if (NULL == g_pstMap)
	{
		MallocMap();
	}
	InitMap((char *)mapDir);
	// 通过关键元素计数判断是否加载成功
	if (NULL != g_pstMap && g_pstMap->iCountLane > 0)
	{
		return 0;
	}
	return -1;
}

read_hdmap::~read_hdmap()
{
	FreeMap();
}

int read_hdmap::MallocMap()
{
	int ret = 0;
    int icount = 0;
	int istaiontnum = 0;
	int ilanenum = 0;
	int isignalnum = 0;
	int iTrafficsign = 0;
	int isectionnum = 0;
	int iroadnum = 0;
	int ijunctionnum = 0;
	
	g_pstMap = new STR_MAP();
	if (NULL == g_pstMap)
	{
        //TRACE_ERROR("g_pstMap new memory fail!!!");
		ret  = -1;
		return ret;
	}
	icount += sizeof(STR_MAP);
    //TRACE_INFO("STR_MAP iCount %d\n", icount);
	
	g_pstMap->pstrStation = new STR_STATION[STATION_NUM];
	if (NULL == g_pstMap->pstrStation)
	{
        //TRACE_ERROR("g_pstMap->pstrStation new memory fail!!!");
		ret  = -1;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_STATION[STATION_NUM]);
    //TRACE_INFO("icount: %d g_pstMap->pstrStation : %d\n", icount, sizeof(STR_STATION[STATION_NUM]));

	for (istaiontnum = 0; istaiontnum < STATION_NUM; istaiontnum++)
	{
		g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint = new STR_POINT2F;
		if (NULL == g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint)
		{
            //TRACE_ERROR("g_pstMap->pstrStation->strPolygon.pstrPoint new memory fail!!!");
			ret  = -1;
			delete g_pstMap->pstrStation;
			delete g_pstMap;
		
			return ret;
		}
		icount += sizeof(STR_POINT2F);
        //TRACE_INFO("icount: %d g_pstMap->pstrStation->strPolygon.pstrPoint %d\n", icount, sizeof(STR_POINT2F));
		
		g_pstMap->pstrStation[istaiontnum].pstrParkingSpace = new STR_PARKING_SPACE;
		if (NULL == g_pstMap->pstrStation[istaiontnum].pstrParkingSpace)
		{
            //TRACE_ERROR("g_pstMap->pstrStation->pstrParkingSpace new memory fail!!!");
			ret  = -1;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_PARKING_SPACE);
        //TRACE_INFO("icount: %d g_pstMap->pstrStation->pstrParkingSpace %d\n", icount, sizeof(STR_PARKING_SPACE));
		
	}
			
	g_pstMap->pstrLane = new STR_LANE[LANE_NUM];
	if (NULL == g_pstMap->pstrLane)
	{
        //TRACE_ERROR(" g_pstMap->pstrLane new memory fail!!!");
		ret  = -1;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_LANE[LANE_NUM]);
    //TRACE_INFO("icount: %d g_pstMap->pstrLane %d\n", icount, sizeof(STR_LANE[LANE_NUM]));

	for (ilanenum = 0; ilanenum < LANE_NUM; ilanenum++)
	{
		g_pstMap->pstrLane[ilanenum].pstrCenter = new STR_POINT5f_LANE_CENTER_POINT[LANE_SAMPLE];
		if (NULL == g_pstMap->pstrLane[ilanenum].pstrCenter)
		{
            //TRACE_ERROR("g_pstMap->pstrLane->pstrCenter new memory fail!!!");
			ret  = -1;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_POINT5f_LANE_CENTER_POINT[LANE_SAMPLE]);
        //TRACE_INFO("icount: %d g_pstMap->pstrLane->pstrCenter %d\n", icount, sizeof(STR_POINT5f_LANE_CENTER_POINT[LANE_SAMPLE]));
		g_pstMap->pstrLane[ilanenum].piIncomeLaneId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrLane[ilanenum].piIncomeLaneId)
		{
            //TRACE_ERROR("g_pstMap->pstrLane->piIncomeLaneId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO(" icount: %d g_pstMap->pstrLane->piIncomeLaneId %d\n", icount, sizeof(int[CONNECT_NUM]));
		g_pstMap->pstrLane[ilanenum].piOutgoLaneId =  new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrLane[ilanenum].piOutgoLaneId)
		{
            //TRACE_ERROR("g_pstMap->pstrLane->piOutgoLaneId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("g_pstMap->pstrLane->piOutgoLaneId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
	}
		
	g_pstMap->pstrStopLine = new STR_STOPLINE[STOP_LINE_COUNT];
	if (NULL == g_pstMap->pstrStopLine)
	{
        //TRACE_ERROR("line:%d g_pstMap->pstrStopLine new memory fail!!!");
		ret  = -1;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_STOPLINE[STOP_LINE_COUNT]);
    //TRACE_INFO("icount: %d g_pstMap->pstrStopLine %d\n", icount, sizeof(STR_STOPLINE[STOP_LINE_COUNT]));

	g_pstMap->pstrCrosswalk = new STR_CROSSWALK[CROSSWALK_COUNT];
	if (NULL == g_pstMap->pstrCrosswalk)
	{
        //TRACE_ERROR("g_pstMap->pstrCrosswalk new memory fail!!!");
		ret  = -1;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_CROSSWALK[CROSSWALK_COUNT]);
    //TRACE_INFO("icount: %d g_pstMap->pstrCrosswalk %d\n", icount, sizeof(STR_CROSSWALK[CROSSWALK_COUNT]));

	g_pstMap->pstrSignal = new STR_SIGNAL[SIHNAL_COUNT];
	if (NULL == g_pstMap->pstrSignal)
	{
        //TRACE_ERROR("g_pstMap->pstrSignal new memory fail!!!");
		ret  = -1;
		delete []g_pstMap->pstrCrosswalk;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_SIGNAL[SIHNAL_COUNT]);
    //TRACE_INFO("icount: %d g_pstMap->pstrSignal %d\n", icount, sizeof(STR_SIGNAL[SIHNAL_COUNT]));

	for (isignalnum = 0 ; isignalnum < SIHNAL_COUNT; isignalnum++)
	{
		g_pstMap->pstrSignal[isignalnum].pstrSubSignal = new STR_SUB_SIGNAL;
		if (NULL == g_pstMap->pstrSignal->pstrSubSignal)
		{
            //TRACE_ERROR("g_pstMap->pstrSignal->pstrSubSignal new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_SUB_SIGNAL);
        //TRACE_INFO("icount: %d g_pstMap->pstrSignal->pstrSubSignal %d\n", icount, sizeof(STR_SUB_SIGNAL));
	}
	
	g_pstMap->pstrClearArea = new STR_CLEAR_AREA[CLEAR_AREA_COUNT];
	if (NULL == g_pstMap->pstrClearArea)
	{
        //TRACE_ERROR("g_pstMap->pstrClearArea new memory fail!!!");
		ret  = -1;
		delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
		delete []g_pstMap->pstrSignal;
		delete []g_pstMap->pstrCrosswalk;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_CLEAR_AREA[CLEAR_AREA_COUNT]);
    //TRACE_INFO("icount: %d g_pstMap->pstrClearArea %d\n", icount, sizeof(STR_CLEAR_AREA[CLEAR_AREA_COUNT]));

	g_pstMap->pstrTrafficSign = new STR_TRAFFIC_SIGN[TRAFFICSIGN_COUNT];
	if (NULL == g_pstMap->pstrTrafficSign)
	{
        //TRACE_ERROR("g_pstMap->pstrTrafficSign new memory fail!!!");
		ret  = -1;
		delete g_pstMap->pstrClearArea;
		delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
		delete []g_pstMap->pstrSignal;
		delete []g_pstMap->pstrCrosswalk;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_TRAFFIC_SIGN[TRAFFICSIGN_COUNT]);
    //TRACE_INFO("icount: %d g_pstMap->pstrTrafficSign %d\n", icount, sizeof(STR_TRAFFIC_SIGN[TRAFFICSIGN_COUNT]));

	for (iTrafficsign = 0; iTrafficsign < TRAFFICSIGN_COUNT; iTrafficsign++)
	{
		g_pstMap->pstrTrafficSign[iTrafficsign].pemType = new EM_TRAFFIC_SIGN_TYPE;
		if (NULL == g_pstMap->pstrTrafficSign->pemType)
		{
            //TRACE_ERROR("g_pstMap->pstrTrafficSign->pemType new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_TRAFFIC_SIGN[TRAFFICSIGN_COUNT]);
        //TRACE_INFO("icount: %d g_pstMap->pstrTrafficSign->pemType %d\n", icount, sizeof(EM_TRAFFIC_SIGN_TYPE));
		
	}
	
	g_pstMap->pstrSection = new STR_SECTION[SECTION_NUM];
	if (NULL == g_pstMap->pstrSection)
	{
        //TRACE_ERROR("g_pstMap->pstrSection new memory fail!!!");
		ret  = -1;
		delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
		delete []g_pstMap->pstrTrafficSign;
		delete g_pstMap->pstrClearArea;
		delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
		delete []g_pstMap->pstrSignal;
		delete []g_pstMap->pstrCrosswalk;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_SECTION[SECTION_NUM]);
    //TRACE_INFO("icount: %d g_pstMap->pstrSection %d\n", icount, sizeof(STR_SECTION[SECTION_NUM]));
	
	for (isectionnum = 0 ; isectionnum < SECTION_NUM; isectionnum++)
	{
		g_pstMap->pstrSection[isectionnum].piLaneId = new int[LANE_NUM];
		if (NULL == g_pstMap->pstrSection[isectionnum].piLaneId)
		{
            //TRACE_ERROR("_pstMap->pstrSection->piLaneId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[LANE_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrSection->piLaneId %d\n", icount, sizeof(int[LANE_NUM]));

		g_pstMap->pstrSection[isectionnum].pstrLeft = new STR_POINT2F[SECTION_SAMPLE];
		if (NULL == g_pstMap->pstrSection[isectionnum].pstrLeft)
		{
            //TRACE_ERROR("g_pstMap->pstrSection->pstrLeft new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_POINT2F[SECTION_SAMPLE]);
        //TRACE_INFO("icount: %d g_pstMap->pstrSection->pstrLeft %d\n", icount, sizeof(STR_POINT2F[SECTION_SAMPLE]));
		
		g_pstMap->pstrSection[isectionnum].pstrRight = new STR_POINT2F[SECTION_SAMPLE];
		if (NULL == g_pstMap->pstrSection[isectionnum].pstrRight)
		{
            //TRACE_ERROR("g_pstMap->pstrSection->pstrRight new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_POINT2F[SECTION_SAMPLE]);
        //TRACE_INFO("icount: %d g_pstMap->pstrSection->pstrRight %d\n", icount, sizeof(STR_POINT2F[SECTION_SAMPLE]));
		
		g_pstMap->pstrSection[isectionnum].piTrafficSignId = new int;
		if (NULL == g_pstMap->pstrSection[isectionnum].piTrafficSignId)
		{
            //TRACE_ERROR("g_pstMap->pstrSection->piTrafficSignId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int);
        //TRACE_INFO("icount: %d g_pstMap->pstrSection->piTrafficSignId %d\n", icount, sizeof(int));
		
		g_pstMap->pstrSection[isectionnum].piIncomeSectionId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrSection[isectionnum].piIncomeSectionId)
		{
            //TRACE_ERROR("g_pstMap->pstrSection->piIncomeSectionId new memory fail!!!");
			ret  = -1;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrSection->piIncomeSectionId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
		g_pstMap->pstrSection[isectionnum].piOutgoSectionId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrSection[isectionnum].piOutgoSectionId)
		{
            //TRACE_ERROR("g_pstMap->pstrSection->piOutgoSectionId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrSection->piOutgoSectionId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
	}
		
	g_pstMap->pstrRoad = new STR_ROAD[ROAD_NUM];
	if (NULL == g_pstMap->pstrRoad)
	{
        //TRACE_ERROR("g_pstMap->pstrRoad new memory fail!!!");
		ret  = -1;
		delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
		delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
		delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
		delete []g_pstMap->pstrSection[isectionnum].pstrRight;
		delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
		delete []g_pstMap->pstrSection[isectionnum].piLaneId;
		delete []g_pstMap->pstrSection;
		delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
		delete []g_pstMap->pstrTrafficSign;
		delete g_pstMap->pstrClearArea;
		delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
		delete []g_pstMap->pstrSignal;
		delete []g_pstMap->pstrCrosswalk;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_ROAD[ROAD_NUM]);
    //TRACE_INFO("icount: %d g_pstMap->pstrRoad %d\n", icount, sizeof(STR_ROAD[ROAD_NUM]));
	for (iroadnum  = 0; iroadnum < ROAD_NUM; iroadnum++ )
	{
		g_pstMap->pstrRoad[iroadnum].piSectionId = new int[SECTION_NUM];
		if (NULL == g_pstMap->pstrRoad[iroadnum].piSectionId)
		{
            //TRACE_ERROR("g_pstMap->pstrRoad->piSectionId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[SECTION_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrRoad->piSectionId %d\n", icount, sizeof(int[SECTION_NUM]));

		g_pstMap->pstrRoad[iroadnum].piIncomeRoadId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrRoad[iroadnum].piIncomeRoadId)
		{
            //TRACE_ERROR("g_pstMap->pstrRoad->piIncomeRoadId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrRoad->piIncomeRoadId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
		g_pstMap->pstrRoad[iroadnum].piOutgoRoadId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrRoad[iroadnum].piOutgoRoadId)
		{
            //TRACE_ERROR("g_pstMap->pstrRoad->piOutgoRoadId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrRoad[iroadnum].piIncomeRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrRoad->piOutgoRoadId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
	}
	
	g_pstMap->pstrJunction = new STR_JUNCTION[JUNCTION_NUM];
	if (NULL == g_pstMap->pstrJunction)
	{
        //TRACE_ERROR("g_pstMap->pstrJunction new memory fail!!!");
		ret  = -1;
		delete []g_pstMap->pstrRoad[iroadnum].piOutgoRoadId;
		delete []g_pstMap->pstrRoad[iroadnum].piIncomeRoadId;
		delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
		delete []g_pstMap->pstrRoad;
		delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
		delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
		delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
		delete []g_pstMap->pstrSection[isectionnum].pstrRight;
		delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
		delete []g_pstMap->pstrSection[isectionnum].piLaneId;
		delete []g_pstMap->pstrSection;
		delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
		delete []g_pstMap->pstrTrafficSign;
		delete g_pstMap->pstrClearArea;
		delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
		delete []g_pstMap->pstrSignal;
		delete []g_pstMap->pstrCrosswalk;
		delete []g_pstMap->pstrStopLine;
		delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
		delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
		delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
		delete g_pstMap->pstrLane;
		delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
		delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
		delete []g_pstMap->pstrStation;
		delete g_pstMap;
		
		return ret;
	}
	icount += sizeof(STR_JUNCTION[JUNCTION_NUM]);
    //TRACE_INFO("icount: %d g_pstMap->pstrJunction %d\n", icount, sizeof(STR_JUNCTION[JUNCTION_NUM]));
	for (ijunctionnum = 0 ; ijunctionnum < JUNCTION_NUM; ijunctionnum++)
	{
		g_pstMap->pstrJunction[ijunctionnum].strPloygon.pstrPoint =  new STR_POINT2F;
		if (NULL == g_pstMap->pstrJunction)
		{
            //TRACE_ERROR("g_pstMap->pstrJunction->strPloygon.pstrPoint new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrJunction;
			delete []g_pstMap->pstrRoad[iroadnum].piOutgoRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piIncomeRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(STR_POINT2F);
        //TRACE_INFO("icount: %d g_pstMap->pstrJunction->strPloygon.pstrPoint %d\n", icount, sizeof(STR_POINT2F));

		g_pstMap->pstrJunction[ijunctionnum].piIncomeRoadId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrJunction[ijunctionnum].piIncomeRoadId)
		{
            //TRACE_ERROR("g_pstMap->pstrJunction->piOutgoRoadId new memory fail!!!");
			ret  = -1;
			delete g_pstMap->pstrJunction[ijunctionnum].strPloygon.pstrPoint;
			delete []g_pstMap->pstrJunction;
			delete []g_pstMap->pstrRoad[iroadnum].piOutgoRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piIncomeRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrJunction->piOutgoRoadId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
		g_pstMap->pstrJunction[ijunctionnum].piOutgoRoadId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrJunction[ijunctionnum].piOutgoRoadId)
		{
            //TRACE_ERROR("g_pstMap->pstrJunction->piOutgoRoadId new memory fail!!!");
			ret  = -1;			
			delete g_pstMap->pstrJunction[ijunctionnum].strPloygon.pstrPoint;
			delete []g_pstMap->pstrJunction[ijunctionnum].piIncomeRoadId;
			delete []g_pstMap->pstrJunction;
			delete []g_pstMap->pstrRoad[iroadnum].piOutgoRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piIncomeRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrJunction->piOutgoRoadId %d\n", icount, sizeof(int[CONNECT_NUM]));
		
		g_pstMap->pstrJunction[ijunctionnum].piConnectRoadId = new int[CONNECT_NUM];
		if (NULL == g_pstMap->pstrJunction[ijunctionnum].piConnectRoadId)
		{
            //TRACE_ERROR("g_pstMap->pstrJunction->piConnectRoadId new memory fail!!!");
			ret  = -1;
			delete []g_pstMap->pstrJunction[ijunctionnum].piIncomeRoadId;
			delete []g_pstMap->pstrJunction[ijunctionnum].piOutgoRoadId;
			delete g_pstMap->pstrJunction[ijunctionnum].strPloygon.pstrPoint;
			delete []g_pstMap->pstrJunction;
			delete []g_pstMap->pstrRoad[iroadnum].piOutgoRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piIncomeRoadId;
			delete []g_pstMap->pstrRoad[iroadnum].piSectionId;
			delete []g_pstMap->pstrRoad;
			delete []g_pstMap->pstrSection[isectionnum].piOutgoSectionId;
			delete []g_pstMap->pstrSection[isectionnum].piIncomeSectionId;
			delete g_pstMap->pstrSection[isectionnum].piTrafficSignId;
			delete []g_pstMap->pstrSection[isectionnum].pstrRight;
			delete []g_pstMap->pstrSection[isectionnum].pstrLeft;
			delete []g_pstMap->pstrSection[isectionnum].piLaneId;
			delete []g_pstMap->pstrSection;
			delete g_pstMap->pstrTrafficSign[iTrafficsign].pemType;
			delete []g_pstMap->pstrTrafficSign;
			delete g_pstMap->pstrClearArea;
			delete g_pstMap->pstrSignal[isignalnum].pstrSubSignal;
			delete []g_pstMap->pstrSignal;
			delete []g_pstMap->pstrCrosswalk;
			delete []g_pstMap->pstrStopLine;
			delete []g_pstMap->pstrLane[ilanenum].piOutgoLaneId;
			delete []g_pstMap->pstrLane[ilanenum].piIncomeLaneId;
			delete []g_pstMap->pstrLane[ilanenum].pstrCenter;
			delete g_pstMap->pstrLane;
			delete g_pstMap->pstrStation[istaiontnum].pstrParkingSpace;
			delete g_pstMap->pstrStation[istaiontnum].strPolygon.pstrPoint;
			delete []g_pstMap->pstrStation;
			delete g_pstMap;
			
			return ret;
		}
		icount += sizeof(int[CONNECT_NUM]);
        //TRACE_INFO("icount: %d g_pstMap->pstrJunction->piConnectRoadId %d\n", icount, sizeof(int[CONNECT_NUM]));
	}
	return ret;

}

void read_hdmap::FreeMap()
{		
	delete g_pstMap->pstrStation->strPolygon.pstrPoint;
	delete g_pstMap->pstrStation->pstrParkingSpace;
	delete [] g_pstMap->pstrStation;
	
	delete [] g_pstMap->pstrLane->pstrCenter;
	delete [] g_pstMap->pstrLane->piIncomeLaneId;
	delete [] g_pstMap->pstrLane->piOutgoLaneId;
	delete [] g_pstMap->pstrLane;

	delete [] g_pstMap->pstrStopLine;	
	delete [] g_pstMap->pstrCrosswalk;
	delete g_pstMap->pstrSignal->pstrSubSignal;
	delete [] g_pstMap->pstrSignal;
	delete [] g_pstMap->pstrClearArea;
	delete g_pstMap->pstrTrafficSign->pemType;
	delete [] g_pstMap->pstrTrafficSign;

	delete [] g_pstMap->pstrSection->piLaneId;
	delete [] g_pstMap->pstrSection->pstrLeft;
	delete [] g_pstMap->pstrSection->pstrRight;
	delete g_pstMap->pstrSection->piTrafficSignId;
	delete [] g_pstMap->pstrSection->piIncomeSectionId;
	delete [] g_pstMap->pstrSection->piOutgoSectionId;
	delete []g_pstMap->pstrSection;

	delete [] g_pstMap->pstrRoad->piSectionId;
	delete g_pstMap->pstrRoad->piIncomeRoadId;
	delete [] g_pstMap->pstrRoad->piOutgoRoadId;
	delete [] g_pstMap->pstrRoad;

	delete g_pstMap->pstrJunction->strPloygon.pstrPoint;
	delete [] g_pstMap->pstrJunction->piOutgoRoadId;
	delete [] g_pstMap->pstrJunction->piConnectRoadId;
	delete [] g_pstMap->pstrJunction;

	delete g_pstMap;

}


void read_hdmap::InitMap(char *mapDir)
{
	int iRet                 = 	0;
	//struct timeval  tv = {0};  
    //struct timezone tz = {0};
	//unsigned long long starttime = 0;
	//unsigned long long endtime = 0;
	//char MapDir[DIR_LEN_MAX] = 	MAPDIR;
	//STR_MAP stMap;
	//STR_MAP *readmap = new STR_MAP;
	//memset(&g_pstMap, 0, sizeof(STR_MAP));

	//iRet = ReadMapFile(MapDir, &stMap);
	//MallocMap();

	//gettimeofday(&tv, &tz);
	//starttime = (unsigned long long)(tv.tv_sec);
	//starttime *= 1000;
	//starttime += (unsigned long long)(tv.tv_usec / 1000);
    ////TRACE_INFO("starttime:%lld \n", starttime);	
	iRet = ReadMapFile(mapDir, g_pstMap);
	if (-1 == iRet )
	{	
		//SET_LIDAR_ALM_MAP_LOAD_FAIL(1);
        //TRACE_ERROR("ReadMapFile failed \n");
		// 加载失败：由调用方通过返回值/计数判断，不在此告警
	}
	else
	{	
		// SET_LIDAR_ALM_MAP_LOAD_FAIL(0); //zyl:源文件有这个，但是嵌套了很多文件，编译麻烦
		// 加载成功
	}
	
	
	//memset(&tv, 0, sizeof(tv));
	//memset(&tz, 0, sizeof(tz));
    //gettimeofday(&tv, &tz);
	//endtime = (unsigned long long)(tv.tv_sec);
	//endtime *= 1000;
	//endtime += (unsigned long long)(tv.tv_usec / 1000);
    ////TRACE_INFO("endtime:%lld \n", endtime);
    ////TRACE_INFO("delta-time:%lld \n", (endtime - starttime));
    //iFredMap();
    
	return;
}

STR_MAP * read_hdmap::get_hdmap()
{
	return g_pstMap;
}

STR_ID_ARRAY getmap(const STR_MAP *stMap, STR_POINT2F strXY)
{
	read_hdmap rmp;
	STR_ID_ARRAY ilaneid;
	ilaneid = rmp.GetLaneIdByPoint(strXY);
	return ilaneid;
}

#if 0
/********************test code*******************************/
int main()
{
	////read_hdmap *read_map = new read_hdmap();
	STR_POINT2F lanexy;
	STR_ID_ARRAY strlaneid;
	read_hdmap rmp;
	struct timeval  tv = {0};  
    struct timezone tz = {0};
	unsigned long long starttime = 0;
	unsigned long long endtime = 0;
    
	lanexy.fX = -53.690636025;
	lanexy.fY = 15.635070611;

	//gettimeofday(&tv, &tz);
	//starttime = (unsigned long long)(tv.tv_sec);
	//starttime *= 1000;
	//starttime += (unsigned long long)(tv.tv_usec / 1000);
    ////TRACE_INFO("starttime:%lld \n", starttime);
	
	//ilaneid = getmap(read_map->get_hdmap(), lanexy);
	strlaneid = rmp.GetLaneIdByPoint(lanexy);
	
    //TRACE_INFO("icount %d ilaneid %d \n", strlaneid.iCount, *(strlaneid.piID));
	//memset(&tv, 0, sizeof(tv));
	//memset(&tz, 0, sizeof(tz));
   // gettimeofday(&tv, &tz);
	//endtime = (unsigned long long)(tv.tv_sec);
	//endtime *= 1000;
	//endtime += (unsigned long long)(tv.tv_usec / 1000);
    ////TRACE_INFO("endtime:%lld \n", endtime);
    ////TRACE_INFO("delta-time:%lld \n", (endtime - starttime));
	
    ////TRACE_INFO("verison:%s\n", rmp.get_hdmap()->version);
    ////TRACE_INFO("dLon = %lf, dLat = %lf\n", rmp.get_hdmap()->strAnchorLonLat.dLon, rmp.get_hdmap()->strAnchorLonLat.dLat);
    ////TRACE_INFO("iCountStation:%d\n", rmp.get_hdmap()->iCountStation);
    ////TRACE_INFO("strLocation.fX:%f strLocation.fY:%f\n", rmp.get_hdmap()->pstrStation->strLocation.fX, rmp.get_hdmap()->pstrStation->strLocation.fY);

	int isectionid = -1;
	int ilandid = 3;
	isectionid = rmp.GetSectionIdByLaneId(ilandid);
    //TRACE_INFO("isectionid %d \n", isectionid);

	STR_LONLAT strLonLat;
	STR_POINT2F strXY;
	strXY.fX = -53.503701769313;
	strXY.fY = 15.877660770956;
	strLonLat = rmp.XY2LonLat(strXY);
    //TRACE_INFO("strLonLat.Lon:%.12lf strLonLat.Lat:%.12lf\n",strLonLat.dLon, strLonLat.dLat);
	
}

#endif
/********************test code*******************************/
#if 0
int main()
{
	char MapDir[DIR_LEN_MAX] = 	MAPDIR;
	read_hdmap *readmap = new read_hdmap();

	readmap->InitMap(MapDir);
   
	/**********点找laneid testcode**********/
	STR_ID_ARRAY laneid;
	STR_POINT2F lanexy;
    
	lanexy.fX = -53.690636025;
	lanexy.fY = 15.635070611;
	laneid = GetLaneIdByPoint(lanexy);
    //TRACE_INFO("laneid.count %d laneid.id %d \n", laneid.iCount, *(laneid.piID));

	/**********laneid找section testcode**********/
    int tmpLaneid = 52;
    int iSectionid = 0;
	iSectionid = GetSectionIdByLaneId(tmpLaneid);
    //TRACE_INFO("iSectionid %d\n", iSectionid);

	/**********sectionid找road testcode**********/
	int tmepSecitonid = 23;
	int iRoadid = 0;
	iRoadid = GetRoadIdBySectionId(tmepSecitonid);
    //TRACE_INFO("iRoadid %d\n", iRoadid);

	/**********roadid找junction testcode**********/
	int tmepRoadid = 14;
	int iJucntionid = 0;
	iJucntionid = GetJunctionByRoadId(tmepRoadid);
    //TRACE_INFO("iJucntionid %d\n", iJucntionid);

	/**********sectionid找左边界线 testcode**********/
	int itmplsection = 3;
	STR_POINT2F_ARRAY *tmplboundary = NULL;

	tmplboundary = GetLBoundaryBySectionId(itmplsection);
	STR_POINT2F strtmpLeft[1000] = {0};
	memcpy(&strtmpLeft, &(tmplboundary->pstrPoint), sizeof(strtmpLeft));
	for (int l = 0; l < tmplboundary->iCount; l++)
	{
        ////TRACE_INFO("tmplboundary.fX: %f tmplboundary.fY: %f \n", strtmpLeft[l].fX, strtmpLeft[l].fY);
	}

	/**********sectionid找右边界线 testcode**********/
	int itmprsection = 4;
	STR_POINT2F_ARRAY *tmprboundary = NULL;

	tmprboundary = GetRBoundaryBySectionId(itmprsection);
	STR_POINT2F strtmpRight[1000] = {0};
	memcpy(&strtmpRight, &(tmprboundary->pstrPoint), sizeof(strtmpRight));

	for (int l = 0; l < tmprboundary->iCount; l++)
	{
        ////TRACE_INFO("tmprboundary.fX: %f tmprboundary.fY: %f \n", strtmpRight[l].fX, strtmpRight[l].fY);
	}


    ////TRACE_INFO("iCountLane number %d\n", g_pstMap.iCountLane);
    ////TRACE_INFO("Junction number %d\n", g_pstMap.iCountJunction);
		
	/**********roadid找左边界线 testcode**********/
	int iroadid = 1;
	STR_POINT2F_ARRAY * ptmplroad = NULL;
	ptmplroad = new STR_POINT2F_ARRAY();
	ptmplroad->pstrPoint = new STR_POINT2F[10000];
	//ptmplroad = (STR_POINT2F_ARRAY *)malloc(sizeof(STR_POINT2F_ARRAY) * 10000);
	ptmplroad = GetLBoundaryByRoadId(iroadid);
	for (int i = 0; i < ptmplroad->iCount; i++)
	{
        ////TRACE_INFO("i:%d fx:%f fy:%f \n", i, ptmplroad->pstrPoint[i].fX, ptmplroad->pstrPoint[i].fY);
	}
	
	/**********roadid找右边界线 testcode**********/
	STR_POINT2F_ARRAY * ptmprroad = NULL;
	ptmprroad = new STR_POINT2F_ARRAY();
	ptmprroad->pstrPoint = new STR_POINT2F[10000];
	ptmprroad = GetRBoundaryByRoadId(iroadid);
	for (int i = 0; i < ptmprroad->iCount; i++)
	{
        ////TRACE_INFO("i:%d fx:%f fy:%f \n", i, ptmprroad->pstrPoint[i].fX, ptmprroad->pstrPoint[i].fY);
	}
	
	/**********laneid找左边界线 testcode**********/
	int ilandid = 1;
	GetLBoundaryByLaneId(ilandid);

	/**********laneid找右边界线 testcode**********/
	GetRBoundaryByLaneId(ilandid);
	
	STR_LONLAT strLonLat;
	STR_POINT2F strXY;
	memset(&strLonLat, 0, sizeof(STR_LONLAT));
	memset(&strXY, 0, sizeof(STR_POINT2F));

	strLonLat.dLon = 114.053909223409;
	strLonLat.dLat = 22.665777603200;
	strXY = LonLat2XY(strLonLat);
    //TRACE_INFO("strXY.X:%.12lf\nstrXY.Y:%.12lf \n", strXY.fX, strXY.fY);

	memset(&strLonLat, 0, sizeof(STR_LONLAT));
	memset(&strXY, 0, sizeof(STR_POINT2F));
	strXY.fX =  -53.503701769313;
	strXY.fY =  15.877660770956;
	strLonLat = XY2LonLat(strXY);
    //TRACE_INFO("strLonLat.Lon:%.12lf\nstrLonLat.Lat:%.12lf\n",strLonLat.dLon, strLonLat.dLat);
	return 0;
}
#endif
/********************test code*******************************/
