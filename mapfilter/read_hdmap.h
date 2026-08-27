/******************************************************************
 *  文件名称：readmap.h
 *  简要描述：1.定义高精地图的数据格式
 *           2.定义高精地图的元素
 *           3.提供获取高精地图相关信息的接口
 *  修改历史：
 *  日期                      修改说明
 *  2019-01-16                创建文件
 *
 *
*******************************************************************/

#ifndef _READMAP_H_
#define _READMAP_H_

#include "convert.h"


//#pragma once
#include<map>
#include<deque>
#include<vector>
#include<iostream>
#pragma pack(push, 1)

/************************************************** 宏定义 ***********************************************/
#define MAPDIR                 "/etc/echiev/hdmap/hdmap.bin"   /* 高精地图文件存放路径 */
#define DIR_LEN_MAX            1024                                         /* 文件及路径最大长度 */
#define ANCHOR_LON             114.054429823102                             /* 经度 */
#define ANCHOR_LAT             22.6656342253927                             /* 纬度 */
#define MAP_VER_LEN            6                                            /* 地图版本号长度 */

typedef std::deque<int> DEQUE_INT;

typedef std::vector<int> VEC_INT;

/************************************************** 枚举定义 *********************************************/

#define MAX_PIDCOUNT  200
/************** 信号灯类型 ***************/
typedef enum _EM_SIGNAL_SHAPE_TYPE 
{
    UNKNOWN = 0,						   /* 未知 */
    HORIZONTAL = 1,						   /* 信号灯垂直排列 */
    VERTICAL = 2,					       /* 信号灯水平排列 */
    SINGLE = 3 							   /* 单个信号灯 */
}EM_SIGNAL_SHAPE_TYPE;

/************ 信号灯方向 *****************/
typedef enum _EM_SIGNAL_ARROW_TYPE 
{
    ARROW_UNKNOWN           = 0,            /* 未知 */
    CIRCLE                  = 1,            /* 圆圈 */
    ARROW_LEFT              = 2,            /* 向左箭头 */
    ARROW_FORWARD           = 3,            /* 向前进箭头 */
    ARROW_RIGHT             = 4,            /* 向右箭头 */
    ARROW_LEFT_AND_FORWARD  = 5,            /* 向左前进箭头 */
    ARROW_RIGHT_AND_FORWARD = 6,            /* 向右前进箭头 */
    ARROW_U_TURN            = 7,            /* U型箭头 */
    DIGITAL                 = 8             /* 数显 */
}EM_SIGNAL_ARROW_TYPE;

/************ 道路类型 *****************/
typedef enum _EM_LANE_TYPE 
{
    UNKNOWN_TYPE            = 0,            /* 未知 */
    CITY_DRIVING            = 1,            /* 城市道路 */
    BIKING                  = 2,            /* 自行车道 */
    SIDEWALK                = 3,            /* 人行道 */
    PARKING                 = 4,            /* 停车道 */
    SHOULDER                = 5             /* 路肩 */
}EM_LANE_TYPE;

typedef enum _EM_LANE_TURN 
{

    UNKNOWN_TURN            = 0,            /* 未知 */
    ALL                     = 1,            /* 直行 + 左转 + 右转合并道 */
    LEFT                    = 2,            /* 左转道 */
    FORWARD                 = 3,            /* 直行道 */
    RIGHT                   = 4,            /* 右转道 */
    LEFT_AND_FORWARD        = 5,            /* 左转 + 直行道 */
    RIGHT_AND_FORWARD       = 6,            /* 右转 + 直行道 */
    U_TURN                  = 7,            /* U型道 */
    LEFT_AND_RIGHT          = 8,            /* 左转 + 右转道 */
} EM_LANE_TURN;

typedef enum _EM_LANE_DIRECTION 
{
    FORWARD_DIRECTION       = 0,            /* Lane 上的箭头方向向前箭头 */
    BACKWARD_DIRECTION      = 1,			/* Lane 上的箭头方向向后箭头 */
    BIDIRECTION             = 2
}EM_LANE_DIRECTION;

/************ 隔离带类型 **************/
typedef enum
{
    UNKNOWN_MEDIAN          = 0,            /* 未知 */
    AVAILABLE_CROSS         = 1,            /* 不可穿越隔离带 */
    INVALID_CROSS           = 2,            /* 可穿越隔离带 */
    INVALID_CROSS_PERSON    = 3             /* 人可穿越车不可穿越 */

}EM_MEDIAN;

/************* 超车类型 ***************/
typedef enum _EM_TRAFFIC_SIGN_TYPE
{
    UNKNOWN_SIGN            = 0,            /* 未知 */
    OVERTAKE                = 1,            /* 允许超车 */
    NO_OVERTAKING           = 2,            /* 禁止超车 */
    MIN_SPEED               = 3,            /* 最低限速 */
    MAX_SPEED               = 4,            /* 最高限速 */
    MIX_MAX_SPEED           = 5,            /* 最高最低限速 */
    SLOW_DOWN_YIELD         = 6,            /* 减速让行 */
    STOP_YIELD              = 7,            /* 停车让行 */
    TRUNK_ROAD_AHEAD        = 8,            /* 干路先行 */
    GIVE_WAY                = 9,            /* 会车让行 */
    FRIST_MEETING           = 10            /* 会车先行 */
}EM_TRAFFIC_SIGN_TYPE;

/****************************************** 结构体定义 *********************************************/

typedef struct _STR_POINT2F
{
    float  fX;                              /* XY坐标中的X */
    float  fY;                              /* XY坐标中的Y */
}STR_POINT2F;

typedef struct _STR_POLYGON2
{
    int iCount;
    STR_POINT2F *pstrPoint;
}STR_POLYGON2, STR_POINT2F_ARRAY;

// typedef struct _STR_ID_ARRAY
// {
//     int iCount;
//     int *piID;
// }STR_ID_ARRAY;

typedef struct _STR_ID_ARRAY
{
    int iCount;
    int piID[MAX_PIDCOUNT];
}STR_ID_ARRAY;

typedef struct _STR_POINT3F
{
    float fX;                               /* XYZ坐标中的X */
    float fY;                               /* XYZ坐标中的Y */
    float fZ;                               /* XYZ坐标中的Z */
}STR_POINT3F;

typedef struct _STR_LONLAT
{
    double dLon;                            /* 经度 */
    double dLat;                            /* 纬度 */
}STR_LONLAT;

typedef struct _STR_SUB_SIGNAL 
{
	EM_SIGNAL_SHAPE_TYPE emShape;			/* 子信号灯的形状 */
	EM_SIGNAL_ARROW_TYPE emArrow;			/* 子信号灯的方向 */
}STR_SUB_SIGNAL;

/**************** 信号灯 *****************/
typedef struct _STR_SIGNAL 
{
	int             iId;					/* 信号灯的ID */
	STR_POINT3F 	strLocation;			/* 信号灯的位置 */
	int 			iCountSubSignal;		/* 信号灯的数量 */
	STR_SUB_SIGNAL 	*pstrSubSignal;         /* 信号灯数组 */
}STR_SIGNAL;

/*************** 斑马线 *****************/
typedef struct _STR_CROSSWALK
{
	int iId;								/* 人行道ID */
	STR_POINT2F pstrPoint[4];				/* 人行道区域 */
}STR_CROSSWALK, STR_CLEAR_AREA;

/************* Lane上的点信息 ***********/
typedef struct _STR_POINT5f_LANE_CENTER_POINT
{
	float fX;								/* 中心线点X坐标 */
	float fY;								/* 中心线点Y坐标 */
	float fS;								/* 中心线采样点与起点的累加距离 */
	float fTheta;							/* 中心线点航向 */
	float fKa;								/* 中心线点曲率 */
}STR_POINT5f_LANE_CENTER_POINT;

/*************** 停车场 *****************/
typedef struct _STR_PARKING_SPACE
{
	float fTheta;							/* 停车场的航向 */
	STR_POINT2F pstrPoint[4];				/* 停车场的位置 */
}STR_PARKING_SPACE;

/************* 停车场/站点 **************/
typedef struct _STR_PARKINGLOT_OR_STATION
{
	int iId;
	STR_POINT2F strLocation;
    STR_POLYGON2 strPolygon;
	int iCount;
	STR_PARKING_SPACE *pstrParkingSpace;
}STR_STATION;

/*************** 信号灯 *****************/
typedef struct _STR_TRAFFIC_SIGN
{
    int iId;
    STR_POINT3F strLocation;
    int iCount;
    EM_TRAFFIC_SIGN_TYPE *pemType;  
}STR_TRAFFIC_SIGN;

/*************** 停止线 *****************/
typedef struct _STR_STOPLINE
{
    int iId;
    STR_POINT2F pstrPoint[2];
}STR_STOPLINE;

/*************** Lane属性 **************/
typedef struct _STR_LANE
{
    int iId;                              /* Lane ID */
    int iCountCenter;			  /* Lane 采样点数量 */
    STR_POINT5f_LANE_CENTER_POINT *pstrCenter; /* 采样点信息 */

    float fLen;                           /* 道路中心线长 */
    float fWidth;                         /* 车道宽度 */

    EM_LANE_TYPE emType;		  /* Lane 所在的道路类型 */
    EM_LANE_TURN emTurn;		  /* Lane 所在道路的车道属性 */
    EM_LANE_DIRECTION emDirection;	  /* Lane 上的箭头标识 */
    int iLimitSpeed;                      /* 车道中心线限速 */ 

    int iSationId;                        /* 站点 ID*/
    int iReverseId;                       /* 反向ID */ 
	
    int iLeftId;                          /* 左方车道ID */
    int iRightId;                         /* 右方车道ID */	

    int iCountIncome;			 /* Lane的后方ID数量 */
    int *piIncomeLaneId;		 /* Lane的后方ID列表 */

    int iCountOutgo;			/* Lane的前方ID数量 */
    int *piOutgoLaneId;			/* Lane的前方ID列表 */
}STR_LANE;

/************** 道路片段结构 ***********/
typedef struct _STR_SECTION
{
    int         iId;          		/* Section ID */
    int 	iCountLane;		/* Lane数量 */
    int      	*piLaneId;              /* 组成道路片段的LaneID 列表 */

    int iCountLeftRight;		/* 左右边界线的数量 */
    STR_POINT2F *pstrLeft;        	/* 道路片段左边界线 */
    STR_POINT2F *pstrRight;       	/* 道路片段右边界线 */
	
    EM_MEDIAN   emLeftMedian;           /* 左隔离带 */
    EM_MEDIAN	emRightMedian;		/* 右隔离带 */

    int iStopLineId;			/* 停止线ID */
    int iCrosswalkId;			/* 人行道ID */
    int iSignalId;			/* 信号灯ID */
    int iClearAreaId;			/* 禁停区ID */

    int iLeftSectionId;                 /* 左边Section ID */
    int iRightSectionId;                /* 右边Section ID */
    int iReverseId;                     /* 反向ID */

    int iCountTrafficSign;	        /* 路标数量 */
    int *piTrafficSignId;               /* 路标ID */

    int iCountIncome;			/* Section的后方ID数量 */
    int *piIncomeSectionId;		/* Section的后方ID列表 */

    int iCountOutgo;			/* Section的前方ID数量*/
    int *piOutgoSectionId;		/* Section的前方ID列表*/
}STR_SECTION;

/***************** Road结构 *****************/
typedef struct _STR_ROAD
{
    int	iId;                   			/* Road ID */
    int iReverseId;                             /* 反向ID */

    int iCountSection;				/* Section的数量 */
    int *piSectionId;				/* Section的信息 */

    int iCountIncome;				/* 进入road的数量 */
    int *piIncomeRoadId;			/* 进入road的信息 */

    int iCountOutgo;				/* Road出去的数量 */
    int *piOutgoRoadId;				/* Road出去的信息 */

    int iOutgoJunctionId;			/* Road连接Juction的ID */
}STR_ROAD;

/***************** Junction 结构 ***********/
typedef struct _STR_JUNCTION
{
    int     	    iId;             	        /* JunctionID */
    STR_POLYGON2     strPloygon;	        /* Junction的信息 */

    int iCountIncome;				/* 进入Junction的Road 数量 */
    int *piIncomeRoadId;			/* 进入Junction的Road 信息 */

    int iCountOutgo;				/* Junction出去的Road 数量 */
    int *piOutgoRoadId;				/* Junction出去的Road 信息 */

    int iCountConnect;				/* Junction内连接的Road 数量 */
    int *piConnectRoadId;			/* Junction内连接的Road 信息 */

}STR_JUNCTION;

/***************** Map结构 *****************/
typedef struct _STR_MAP
{
    char        version[MAP_VER_LEN];       	/* 地图版本号 */
    STR_LONLAT  strAnchorLonLat;     		/* 坐标原点经纬度 */

    int iCountStation;				/* 站点数量 */
    STR_STATION *pstrStation;			/* 站点信息 */

    int iCountLane;				/* Lane数量 */
    STR_LANE *pstrLane;				/* Lane信息 */

    int iCountStopLine;				/* 停止线数量 */
    STR_STOPLINE    *pstrStopLine;              /* 停止线信息 */

    int iCountCrosswalk;			/* 斑马线数量 */
    STR_CROSSWALK   *pstrCrosswalk;		/* 斑马线信息 */

    int iCountSignal;				/* 信号灯数量 */
    STR_SIGNAL      *pstrSignal;		/* 信号灯信息 */

    int iCountClearArea;			/* 禁停区数量 */
    STR_CLEAR_AREA  *pstrClearArea;		/* 禁停区信息 */

    int iCountTrafficSign;			/* 路标数量 */
    STR_TRAFFIC_SIGN *pstrTrafficSign;		/* 路标信息 */

    int iCountSection;				/* Section数量 */
    STR_SECTION	*pstrSection;                   /* Section信息 */

    int iCountRoad;				/* Road数量 */
    STR_ROAD *pstrRoad;				/* Road信息 */

    int iCountJunction;				/* Junction数量 */
    STR_JUNCTION *pstrJunction;			/* Junction信息 */
}STR_MAP;


typedef struct _STR_POINT_ID_IN_MAP
{
    STR_ID_ARRAY piLaneId;
    STR_ID_ARRAY piSectionId; /* 当前点所在的车道所在的SectionId序列 */
    STR_ID_ARRAY piRoadId;    /* 当前点坐在的车道所在的Section所在的RoadId序列 */
    STR_ID_ARRAY piJunctionId;
}STR_POINT_ID_IN_MAP;

extern double g_dLon;
extern double g_dLat;



/**************************************** 函数接口 ***********************************/

float distance(const float x1, const float y1 , const float x2, const float y2);

void rotateAsPoint(const float x1, const float y1 , const float x2, const float y2, const float theta, float &x, float &y);



#ifdef __cplusplus
extern "C" {
#endif


class read_hdmap
{
private:
    STR_MAP *g_pstMap;
public:
    read_hdmap();
    ~read_hdmap();

    // 路径参数化加载：分配内存 + 解析地图文件
    // 返回 0 表示成功，-1 表示失败（路径无效/文件不存在/解析失败）
    int load(const char *mapDir);
    void InitMap(char *mapDir);
    STR_MAP * get_hdmap();
    //STR_MAP LoadMapFromFlie(char *mapDir);
    STR_ID_ARRAY GetLaneIdByPoint(STR_POINT2F strXY);
    int GetSectionIdByLaneId(int iLaneId);
    int GetRoadIdBySectionId(int iSectionId);
    int GetJunctionByRoadId(int iRoadId);


    STR_POINT2F_ARRAY *GetLBoundaryByLaneId(int iLaneId);
    STR_POINT2F_ARRAY *GetRBoundaryByLaneId(int iLaneId);
    STR_POINT2F_ARRAY *GetLBoundaryBySectionId(int iSectionId);
    STR_POINT2F_ARRAY *GetRBoundaryBySectionId(int iSectionId);
    STR_POINT2F_ARRAY *GetLBoundaryByRoadId(int iRoadId);
    STR_POINT2F_ARRAY *GetRBoundaryByRoadId(int iRoadId);
    STR_POLYGON2 GetRoIPolygonByPoint(STR_POINT3F strXYZ, float front, float after);
    STR_POINT2F LonLat2XY(STR_LONLAT strLonLat, STR_LONLAT strAnchor);
    STR_LONLAT XY2LonLat(STR_POINT2F strXY, STR_LONLAT strAnchor);
private:
    int ReadMapFile(char *pchMapDir ,STR_MAP * pstMap);
    int MallocMap();
    void FreeMap();
};



/**************************************************************************
Function Name : GetLaneByPoint
Description:    根据XYZ点获取Lane ID
InParam:        STR_POINT3F strXYZ XY坐标下的点
OutParam:       void
Return:         STR_ID_ARRAY：返回XYZ所在的Lane ID
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_ID_ARRAY read_hdmap::GetLaneIdByPoint(STR_POINT2F strXY);

/**************************************************************************
Function Name : GetSectionIdByLaneId
Description:    根据LaneId获取Section
InParam:        int iLaneId：Lane ID               
OutParam:       void
Return:         int：返回Lane ID 所在的Section
Caller:         提供给地图使用者调用

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//int GetSectionIdByLaneId(int iLaneId);

/**************************************************************************
Function Name : GetRoadIdBySectionId
Description:    根据Section ID获取Road
InParam:        int iSectionId：Section ID                
OutParam:       void
Return:         int：返回Section所在的Road
Caller:         提供给地图使用者调用

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//int GetRoadIdBySectionId(int iSectionId);

/**************************************************************************
Function Name : GetJunctionByRoadId
Description:    根据道路片段获取Road
InParam:        int iRoadId：Road ID                
OutParam:       void
Return:         int：返回Road ID所在的Junction
Caller:         提供给地图使用者调用

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//int GetJunctionByRoadId(int iRoadId);

/**************************************************************************
Function Name : GetLBoundaryByLaneId
Description:    根据Lane ID获取左边界
InParam:        int iLaneId：Lane ID                
OutParam:       void
Return:         STR_POINT2F_ARRAY ：返回所在Lane的左边界
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F_ARRAY *GetLBoundaryByLaneId(int iLaneId);

/**************************************************************************
Function Name : GetRBoundaryByLaneId
Description:    根据Lane ID获取右边界
InParam:        int iLaneId：Lane ID                
OutParam:       void
Return:         STR_POINT2F_ARRAY ：返回所在Lane的右边界
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F_ARRAY *GetRBoundaryByLaneId(int iLaneId);

/**************************************************************************
Function Name : GetLBoundaryBySectionId
Description:    根据Section ID获取左边界
InParam:        int iSectionId：Section ID                
OutParam:       void
Return:         STR_POINT2F_ARRAY ：返回所在Section的左边界
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F_ARRAY *GetLBoundaryBySectionId(int iSectionId);

/**************************************************************************
Function Name : GetRBoundaryBySectionId
Description:    根据Section ID获取右边界
InParam:        iint iSectionId：Section ID                 
OutParam:       void
Return:         STR_POINT2F_ARRAY ：返回所在Section的右边界
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F_ARRAY *GetRBoundaryBySectionId(int iSectionId);

/**************************************************************************
Function Name : GetLBoundaryByRoadId
Description:    根据Road ID获取左边界
InParam:        int iRoadId：Road ID                
OutParam:       void
Return:         STR_POINT2F_ARRAY ：返回所在Road的左边界
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F_ARRAY *GetLBoundaryByRoadId(int iRoadId);

/**************************************************************************
Function Name : GetRBoundaryByRoadId
Description:    根据Road ID获取右边界
InParam:        iint iRoadId：Road ID                 
OutParam:       void
Return:         STR_POINT2F_ARRAY ：返回所在Road的右边界
Caller:         提供给地图使用者调用,由调用者释放

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F_ARRAY *GetRBoundaryByRoadId(int iRoadId);

/**************************************************************************
Function Name : LonLat2XY
Description:    经纬度转XY
InParam:        STR_LONLAT strLonLat：经纬度              
OutParam:       STR_POINT2F strXY: XY坐标中的XY                
Return:         int：成功 0；失败-1
Caller:         提供给地图使用者调用

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_POINT2F LonLat2XY(STR_LONLAT strLonLat);

/**************************************************************************
Function Name : XY2LonLat
Description:    XY转经纬度
int Param:      STR_POINT2F strXY: XY坐标中的XY
OutParam:       void 
Return:         STR_LONLAT：返回转换后的经纬度
Caller:         提供给地图使用者调用

History:
  1.Date:   2019/01/16 
            Create function        
**************************************************************************/
//STR_LONLAT XY2LonLat(STR_POINT2F strXY);


//int iFreeMap(STR_MAP *pstMap);

//float distance(const float x1, const float y1 , const float x2, const float y2);

//void rotateAsPoint(const float x1, const float y1 , const float x2, const float y2, const float theta, float &x, float &y);

#ifdef __cplusplus
}
#endif

#pragma pack(pop)

#endif/*_READMAP_H_*/
