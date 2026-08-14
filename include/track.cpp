#include "track.h"


namespace Lidar_Low_Detection
{

extern std::atomic<uint64_t> g_previousTimestamp;

uint64_t nextTrackID = 9999;


unsigned long long GetCurrentTimestamp()
{
    auto now =
        std::chrono::system_clock::now();

    auto ms =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            now.time_since_epoch());

    return ms.count();
}

std::string timestampToTimeString(unsigned long long timestamp_in_milliseconds) {
    // 将毫秒时间戳转换为秒和毫秒
    time_t seconds = timestamp_in_milliseconds / 1000;
    unsigned long long ms = timestamp_in_milliseconds % 1000;

    // struct tm* timeinfo;
    // timeinfo = localtime(&seconds); // 注意：localtime 是线程不安全的，在多线程中建议用 localtime_r

	// 1. 在栈上分配一个 tm 结构体，作为当前线程私有的缓冲区
    struct tm timeinfo_struct;
    
    // 2. 使用 POSIX 标准的 localtime_r，将结果写入我们自己的缓冲区
    // 注意：localtime_r 的参数顺序是 (时间戳指针, 目标结构体指针)
    localtime_r(&seconds, &timeinfo_struct); 


    std::stringstream ss;
    // ss << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S");
	ss << std::put_time(&timeinfo_struct, "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms;
    return ss.str();
}





KalmanFilter2D::KalmanFilter2D(float init_x, float init_y){
	state = {init_x, init_y, 0.0f, 0.0f};
	// 初始化协方差，位置不确定性小，速度不确定性大
	P = {1.0f, 1.0f, 10.0f, 10.0f};
	// 过程噪声，假设加速度有一定扰动
	Q = {0.01f, 0.01f, 0.1f, 0.1f};
	// 测量噪声，假设检测有一定误差
	R = {0.5f, 0.5f};
}


void KalmanFilter2D::predict(float dt) {
	// 状态转移
	state[0] += state[2] * dt; // pos_x = pos_x + vel_x * dt
	state[1] += state[3] * dt; // pos_y = pos_y + vel_y * dt
	// 速度保持不变

	// 协方差预测 (简化版)
	for (int i = 0; i < 4; ++i) {
		P[i] += Q[i];
	}
}


void KalmanFilter2D::update(float meas_x, float meas_y) {
	// 计算卡尔曼增益 (简化版，假设P和R是对角矩阵)
	std::array<float, 2> K; // 卡尔曼增益
	K[0] = P[0] / (P[0] + R[0]);
	K[1] = P[1] / (P[1] + R[1]);

	// 更新状态
	state[0] += K[0] * (meas_x - state[0]);
	state[1] += K[1] * (meas_y - state[1]);
	// 更新速度估计
	state[2] += K[0] * (meas_x - state[0]) / (P[0] > 0.001f ? P[0] : 0.001f); // 防止除零
	state[3] += K[1] * (meas_y - state[1]) / (P[1] > 0.001f ? P[1] : 0.001f);

	// 更新协方差
	P[0] = (1 - K[0]) * P[0];
	P[1] = (1 - K[1]) * P[1];
}


void SimpleTracker::update(const std::vector<TrackedObstacle>& detections, const unsigned long long &time)
{

	float delta_time = (time - g_previousTimestamp.load(std::memory_order_acquire)) /1000.0f;
	LOG_RAW("上一轮和当前时间差: %.3f\n", delta_time);

	// 1. 如果当前没有检测到任何目标，更新现有轨迹的状态（标记为丢失）
	if (detections.empty()) {
		

		for (auto& track : vtrackings) {
			
			//卡尔曼滤波
			// if (track.kf) {
            //     track.kf->predict(delta_time); // 预测位置
            //     // 用预测值更新轨迹的位置信息
            //     track.pos_x = track.kf->getPosX();
            //     track.pos_y = track.kf->getPosY();
            // }
			

			// // --- 漏检预测逻辑 x = x + vx * dt---
            // track.pos_x = track.pos_x + track.vx * delta_time;
            // track.pos_y = track.pos_y + track.vy * delta_time;

			
			track.lastSeen++;
		}
		// 添加删除长时间未出现的目标
		removeLostTargets();
		return;
	}

	//卡尔曼滤波预测
    // for (auto& track : vtrackings) {
    //     if (track.kf) {
    //         track.kf->predict();
    //     }
    // }

	//  为匹配计算一个“预测位置”，但不直接修改 track 的 pos_x/pos_y
    // 我们创建一个临时的位置列表用于计算代价矩阵
    // std::vector<std::pair<float, float>> predicted_positions;
    // for (const auto& track : vtrackings) {
    //     float pred_x = track.pos_x + track.vx * delta_time;
    //     float pred_y = track.pos_y + track.vy * delta_time;
    //     predicted_positions.push_back({pred_x, pred_y});
    // }




	// 2. 计算代价矩阵 (Cost Matrix)
	// 行是现有的 vtrackings，列是当前的 detections
	int rows = vtrackings.size();
	int cols = detections.size();
	std::vector<std::vector<float>> cost_matrix(rows, std::vector<float>(cols, 0.0f));

	// 填充代价矩阵：计算每个检测框与每个现有轨迹的距离
	for (int t = 0; t < rows; t++) {
		for (int d = 0; d < cols; d++) {

			// // 使用卡尔曼滤波预测的位置进行匹配
            // float pred_x = vtrackings[t].kf ? vtrackings[t].kf->getPosX() : vtrackings[t].pos_x;
            // float pred_y = vtrackings[t].kf ? vtrackings[t].kf->getPosY() : vtrackings[t].pos_y;
            // float dx = pred_x - detections[d].pos_x;
            // float dy = pred_y - detections[d].pos_y;


			// float dx = predicted_positions[t].first - detections[d].pos_x;
            // float dy = predicted_positions[t].second - detections[d].pos_y;

			float dx = vtrackings[t].pos_x - detections[d].pos_x;
			float dy = vtrackings[t].pos_y - detections[d].pos_y;
			float distance = sqrt(dx * dx + dy * dy);
			cost_matrix[t][d] = distance;
		}
	}

	// 3. 匹配逻辑 (简单的贪心匹配，为了效率)
	// 创建标记数组，记录哪些检测框已经被匹配了
	std::vector<bool> matched_detections(cols, false);
	std::vector<bool> matched_vtrackings(rows, false);

	// 遍历代价矩阵，寻找最佳匹配
	for (int i = 0; i < rows; i++)
	{
		float min_cost = match_threshold;
		int best_det_idx = -1;

		for (int j = 0; j < cols; j++) {
			// 寻找距离小于阈值且未被匹配的检测框
			if (!matched_detections[j] && cost_matrix[i][j] < min_cost) {
				min_cost = cost_matrix[i][j];
				best_det_idx = j;
			}
		}

		if (best_det_idx != -1)
		{
			
			// if (vtrackings[i].kf) {
            //     vtrackings[i].kf->update(detections[best_det_idx].pos_x, detections[best_det_idx].pos_y);
            // }
            // //用滤波后的平滑位置更新轨迹
            // vtrackings[i].pos_x = vtrackings[i].kf->getPosX();
            // vtrackings[i].pos_y = vtrackings[i].kf->getPosY();

			// // 1. 保存“上一帧的位置”
            // float old_x = vtrackings[i].pos_x;
            // float old_y = vtrackings[i].pos_y;

            // // 2. 更新位置 (直接使用检测值)
            // vtrackings[i].pos_x = detections[best_det_idx].pos_x;
            // vtrackings[i].pos_y = detections[best_det_idx].pos_y;
            
            // // 3. 更新速度：使用 (当前检测位置 - 上一帧位置) / dt
            // // 这正是您提出的、最直观的方法！
            // vtrackings[i].vx = (vtrackings[i].pos_x - old_x) / delta_time;
            // vtrackings[i].vy = (vtrackings[i].pos_y - old_y) / delta_time;



			vtrackings[i].pos_x = detections[best_det_idx].pos_x;
			vtrackings[i].pos_y = detections[best_det_idx].pos_y;
			vtrackings[i].pos_z = detections[best_det_idx].pos_z;
			
			vtrackings[i].depth = detections[best_det_idx].depth;
			vtrackings[i].width = detections[best_det_idx].width;
			vtrackings[i].height = detections[best_det_idx].height;

			vtrackings[i].Translation[0] = detections[best_det_idx].Translation[0];
			vtrackings[i].Translation[1] = detections[best_det_idx].Translation[1];
			vtrackings[i].Translation[2] = detections[best_det_idx].Translation[2];
			for(int r = 0; r < 9; r++){
				vtrackings[i].Rotation[r] = detections[best_det_idx].Rotation[r];
			}
			// 同步更新4个角点，否则corners保留旧帧数据，与新的center/depth/width不匹配
			for (int c = 0; c < 4; c++) {
				vtrackings[i].corners[c] = detections[best_det_idx].corners[c];
			}
			
			vtrackings[i].age++;
			vtrackings[i].lastSeen = 0; // 重置丢失计数
			matched_vtrackings[i] = true;
			matched_detections[best_det_idx] = true;

			
			LOG_RAW(" [与%d匹配上] 当前跟踪id = %d，distance = %.3f\n", detections[best_det_idx].cluster_id, vtrackings[i].id, cost_matrix[i][best_det_idx]);
			// LOG_RAW(" [与%d匹配] Track id = %d，distance = %.3f，Center(%.2f, %.2f, %.2f)，age=%d\n", 
			// 	detections[best_det_idx].cluster_id, vtrackings[i].id, cost_matrix[i][best_det_idx],
			// 	vtrackings[i].pos_x, vtrackings[i].pos_y, vtrackings[i].pos_z, vtrackings[i].age
			// );

		}
		else {
			// 未匹配上：目标可能暂时消失了
			vtrackings[i].lastSeen++;
			LOG_RAW("may丢失, id = %d, loss_count = %d \n", vtrackings[i].id, vtrackings[i].lastSeen);
		}
	}

	// 4. 处理未匹配的检测 (新目标)
	for (int k = 0; k < cols; k++) {
		if (!matched_detections[k]) {
			// 创建新轨迹
			TrackedObstacle new_track = detections[k];
			// TrackedObstacle new_track(detections[k].pos_x, detections[k].pos_y, 0.0 , 0.0); // 使用新构造函数

			new_track.cluster_id = detections[k].cluster_id;
			new_track.id = nextTrackID++;
			new_track.age = 0;
			new_track.lastSeen = 0;
			vtrackings.push_back(new_track);
			LOG_RAW("(待定)new_track id = %d, age =%d; (平移)[%.3f, %.3f, %.3f]\n", new_track.id, new_track.age,
				new_track.Translation[0], new_track.Translation[1], new_track.Translation[2]);
			// LOG_RAW("		(旋转)[%f, %f, %f, %f, %f, %f, %f, %f, %f]\n",
			// 	new_track.Rotation[0], new_track.Rotation[1], new_track.Rotation[2],
			// 	new_track.Rotation[3], new_track.Rotation[4], new_track.Rotation[5],
			// 	new_track.Rotation[6], new_track.Rotation[7], new_track.Rotation[8]);
		}
	}

	// 5. 清理长时间未出现的目标
	removeLostTargets();


	// 6. 防止nextTrackID溢出
	if (nextTrackID > 50000) {
		int min_id = 100000; 
		for(auto it = vtrackings.begin(); it != vtrackings.end(); ++it) {
			if(it->id < min_id) {
				min_id = it->id;
			}
		}
		if(min_id >= 50001) {
			nextTrackID = 9999;
		}
	}
}

void SimpleTracker::removeLostTargets(){
// 移除 lastSeen > 10 的目标（例如：连续10帧没看到就删掉）
	auto it = std::remove_if(vtrackings.begin(), vtrackings.end(),
		[](const TrackedObstacle& t) { 
			return t.lastSeen > 10; 
			// return t.lastSeen == 1; 
		}
	);
	vtrackings.erase(it, vtrackings.end());
}










// void TrackingThread()
// {



//         //--------------------------------
// 		// 跟踪器
// 		//--------------------------------
//         g_simpleTracker.update(vTransFrameMsg);

//         //--------------------------------
//         // 卡尔曼
//         //--------------------------------

//         //--------------------------------
//         // IOU匹配
//         //--------------------------------

//         //--------------------------------
//         // Track管理
//         //--------------------------------




}