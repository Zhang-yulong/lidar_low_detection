#include "UdpCommunication.h"
#include <mutex>

namespace Lidar_Low_Detection
{


CvLaneDataManager& CvLaneDataManager::GetInstance() {
    /*1.内部定义了一个静态的LaneDataManager实例，并返回它的引用。
        静态局部变量在第一次调用时初始化，之后每次调用都返回同一个实例。
        这样确保了整个程序运行期间只有一个实例存在。
      2.使用静态方法可以方便各个线程获取同一个数据管理器实例*/
    static CvLaneDataManager instance;
    return instance;
}

void CvLaneDataManager::update(const std::vector<STR_CV_LANE_DATA>& new_data) {
    /*整个过程需要一次性加锁，无需中途解锁。
    std::lock_guard 的特点：
    1.自动加锁与释放：在构造时加锁，析构时（离开作用域时）自动释放锁。
    2.不可手动控制锁：不支持中途解锁或重新加锁，锁的生命周期严格绑定到作用域。
    */
    std::lock_guard<std::mutex> lock(mutex_);   //自动加锁
    data_ = new_data;                              //更新数据
    data_ready_ = true;                            //标记数据就绪
    cv_.notify_one();                               //唤醒一个等待线程
    // cv_.notify_all();    //唤醒所有等待线程
}

const std::vector<STR_CV_LANE_DATA> CvLaneDataManager::WaitAndGetData() {
    /*std::condition_variable::wait必须接收一个std::unique_lock<std::mutex>参数。
    wait在等待时会自动释放锁（避免死锁），并在被唤醒后重新加锁。
    */
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return data_ready_; });     //若 data_ready_为true，wait会释放锁并阻塞线程。
    // cv_.wait(lock, [this] { return !queue_.empty(); });  // 与cv_.notify_all();配套使用。 //每个线程获取数据前需再次判断队列是否为空（标准做法，防止“虚假唤醒”）

    auto result = data_;
    data_ready_ = false;
    return result;
}


void CvLaneDataCallback::receiveData(const std::vector<STR_CV_LANE_DATA>& frame){
    std::lock_guard<std::mutex> lock(callback_mutex_);
    for(auto cb : callbacks_ ){
        cb(frame);  //触发所有监听者
    }
}

void CvLaneDataCallback::registerCallback(CallbackFunc cb){
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.push_back(cb);
}


// 下版本： recv_thread --> CvLaneDataManager --> [处理线程] --> 结果队列 --> 主线程展示
void recv_thread(){

    int numbytes;

    struct sockaddr_in remote_addr;
    int remote_fd;

    if( (remote_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
        perror("create socket error\n");
        exit(0);
    }
    else{
        printf("create socket successful\n");
    }

    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET; //IPV4
    remote_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    remote_addr.sin_port = htons(SERVER_PORT); 

    socklen_t len = sizeof(struct sockaddr);

    int opt = 1;
    setsockopt( remote_fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt) );


    if( bind(remote_fd, (struct sockaddr*)&remote_addr, sizeof(remote_addr)) == -1){
        printf("bind socket error\n");
        exit(0);
    }
    else{
        printf("bind socket successful\n");
    }

    char recv_buff[BUFF_LEN] = {0};

    while(1){
        
        printf("--------------------\n");
        memset(recv_buff, 0, sizeof(recv_buff));
        numbytes = recvfrom(remote_fd, recv_buff, BUFF_LEN, 0, (struct sockaddr *)&remote_addr, &len);
        printf("receive bytes: %d \n", numbytes);

        if( numbytes == -1 ){
            perror("recive error\n");
            continue;
        }
        else if( numbytes == 0){
            printf("close connect\n");
        }
            
        // printf("go packet from %s \n", inet_ntoa(remote_addr.sin_addr));

        // int amount = numbytes / sizeof(STR_CV_LANE_DATA);
        // printf("sizeof(STR_CV_LANE_DATA): %d \n", sizeof(STR_CV_LANE_DATA));
        int amount = numbytes / 1639;
        
        printf("个数: %d\n", amount );

        STR_CV_LANE_DATA rec_array[amount];

        std::vector<STR_CV_LANE_DATA> tmp_vec;
        for(int idx = 0; idx < amount; idx++)
        {
            // memcpy(&rec_array[idx], recv_buff+idx*sizeof(STR_CV_LANE_DATA), sizeof(STR_CV_LANE_DATA));
            memcpy(&rec_array[idx], recv_buff+idx*1639, 1639);

            // udp_camera_target.push_back(rec_array[idx]);
            // printf("UDP接收总个数: %d, recv data: camera id = %d ; TargetType = %d ; X = %f ; Y = %f ; angle = %f ; camera_Speed = %f ; polygonTarget nums = %d\n",
            // 	amount, 
            //     rec_array[idx].iId,
            //     rec_array[idx].emTypeTarget, 
            //     rec_array[idx].strPoint3fCenter.fX, 
            // 	   rec_array[idx].strPoint3fCenter.fY, 
            //     (rec_array[idx].fAngle*180 / 3.14),
            //     rec_array[idx].fSpeed, 
            //     rec_array[idx].strPolygonTarget.byNums
            //     );

            // udp_meta_data.push_back(rec_array[idx]);
            // printf("UDP接收总个数: %d, life_time = %d ; recv data: camera id = %d ; TargetType = %d ; confidence = %f ; age = %d ; X = %f ; Y = %f ; \n",
            //     amount, 
            //     rec_array[idx].life_time,
            //     rec_array[idx].iId,
            //     rec_array[idx].typeID, 
            //     rec_array[idx].dealConfidence, 
            //     rec_array[idx].age,
            //     rec_array[idx].strPoint3fCenter.fX,
            //     rec_array[idx].strPoint3fCenter.fY,
            //     rec_array[idx].strSizeTarget,
            //     rec_array[idx].fAngle,
            //     rec_array[idx].fAngleRate,
            //     rec_array[idx].fSpeed,
            //     rec_array[idx].strPolygonTarget

            //     );

            tmp_vec.push_back(rec_array[idx]);

            for(int j = 0; j < rec_array[idx].iCounts; j++){
                printf("《 timeStamp: %lld, valid_lane_cnt: %d, StartPoint x: %f, y: %f; EndPoint x: %f, y:%f》\n",
        			
					rec_array[idx].ullTimestamp,
					rec_array[idx].iCounts,
					rec_array[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[0].fY,
					rec_array[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[0].fX,
                    rec_array[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[1].fY,
					rec_array[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[1].fX
					//根据数据来看，这里的fX和fY应该互换
        		);
            }

        }

        

        CvLaneDataManager::GetInstance().update(tmp_vec);
    }
    close(remote_fd);
}

}