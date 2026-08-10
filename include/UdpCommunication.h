#pragma once
#ifndef UDP_COMMUNICATION_H
#define UDP_COMMUNICATION_H


#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>

#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <pthread.h>
#include <string.h>
#include <functional>
#include <list>

#include "struct_typedef.h"

#define SERVER_PORT 9700
#define BUFF_LEN 4096

namespace Lidar_Low_Detection
{
/*新加的相机置信度结构体*/
typedef struct _STR_CV_META_DATA{
	int iId;
	unsigned int typeID;
	float dealConfidence; //已经使用conf_scale处理过原始置信度
	int life_time;
	int age;
	
	STR_POINT3 strPoint3fCenter;
	STR_SIZE strSizeTarget;
	float fAngle;
	float fAngleRate;
	float fSpeed;
	STR_POLYGON strPolygonTarget;

}STR_CV_META_DATA;


class CvLaneDataManager{

public: 
	static CvLaneDataManager& GetInstance();  //静态成员函数属于类本身，所以它们可以在没有类对象的情况下被调用；单例全局访问：通过静态方法 getInstance() 获取全局唯一实例；

	void update(const std::vector<STR_CV_LANE_DATA> &new_data);

	const std::vector<STR_CV_LANE_DATA> WaitAndGetData();

private:
	//下面避免用户可能无意中创建多个实例，破坏单例
	CvLaneDataManager() = default; //私有默认构造函数；单例模式的核心是禁止外部直接创建类的实例，构造函数必须是 private 的
	CvLaneDataManager(const CvLaneDataManager&) = delete; // 单例模式禁用拷贝构造函数
    CvLaneDataManager& operator=(const CvLaneDataManager&) = delete; // 单例模式禁用拷贝赋值运算符

    std::vector<STR_CV_LANE_DATA> data_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool data_ready_ = false;

};


typedef  std::function<void(const std::vector<STR_CV_LANE_DATA>&)> CallbackFunc;
//chatgpt写的回调
class CvLaneDataCallback{
public:
	void receiveData(const std::vector<STR_CV_LANE_DATA>& frame);

	void registerCallback(CallbackFunc cb); // 多个模块可注册监听
private:
	std::mutex callback_mutex_;
	std::list<CallbackFunc> callbacks_;
	
};


void recv_thread();

}

#endif
