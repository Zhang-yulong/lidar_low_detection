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




void SimpleTracker::hungarianAssignment(
    const std::vector<std::vector<float>>& cost_matrix,
    std::vector<int>& track_to_detection)
{
    const int rows = static_cast<int>(cost_matrix.size());

    if (rows == 0) {
        track_to_detection.clear();
        return;
    }

    const int cols = static_cast<int>(cost_matrix[0].size());

    if (cols == 0) {
        track_to_detection.assign(rows, -1);
        return;
    }

    // ------------------------------------------------------------
    // 构造成方阵
    // n = max(Tracks, Clusters)
    // ------------------------------------------------------------
    const int n = std::max(rows, cols);

    // dummy 匹配的代价
    //
    // 真实匹配如果 distance < match_threshold，
    // 那么真实匹配一定比 dummy 更有优势。
    //
    // 如果真实匹配已经超过 threshold，
    // 我们在 cost matrix 中会设置为 INF，
    // Hungarian 就会倾向于选择 dummy。
    const float DUMMY_COST = match_threshold;

    const float INF = 1e6f;

    std::vector<std::vector<float>> cost(
        n,
        std::vector<float>(n, DUMMY_COST)
    );

    // 拷贝真实代价
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cost[i][j] = cost_matrix[i][j];
        }
    }

    // ------------------------------------------------------------
    // Hungarian algorithm
    //
    // u / v: potentials
    // p:     当前 column 对应的 row
    // way:   增广路径
    // ------------------------------------------------------------
    std::vector<float> u(n + 1, 0.0f);
    std::vector<float> v(n + 1, 0.0f);

    std::vector<int> p(n + 1, 0);
    std::vector<int> way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {

        p[0] = i;

        int j0 = 0;

        std::vector<float> minv(n + 1, INF);
        std::vector<char> used(n + 1, false);

        do {
            used[j0] = true;

            int i0 = p[j0];
            float delta = INF;
            int j1 = 0;

            for (int j = 1; j <= n; ++j) {

                if (used[j])
                    continue;

                float cur =
                    cost[i0 - 1][j - 1]
                    - u[i0]
                    - v[j];

                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }

                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }

            for (int j = 0; j <= n; ++j) {

                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;

        } while (p[j0] != 0);

        // --------------------------------------------------------
        // 增广
        // --------------------------------------------------------
        do {
            int j1 = way[j0];

            p[j0] = p[j1];

            j0 = j1;

        } while (j0 != 0);
    }

    // ------------------------------------------------------------
    // p[column] = row
    //
    // 转换成：
    // track_to_detection[track] = detection
    // ------------------------------------------------------------
    track_to_detection.assign(rows, -1);

    for (int j = 1; j <= n; ++j) {

        int i = p[j];

        if (i <= 0)
            continue;

        int track_idx = i - 1;
        int detection_idx = j - 1;

        // 只有真实 Track + 真实 Detection 才算匹配
        if (track_idx < rows &&
            detection_idx < cols) {

            // 再做一次安全检查
            if (cost_matrix[track_idx][detection_idx]
                    < match_threshold) {

                track_to_detection[track_idx] =
                    detection_idx;
            }
        }
    }
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
			

			// 思考：如果漏检，还保留障碍物位置不动，那么给到下游的融合将雷达系下目标转到地图系下。
			// 		若车不动，那障碍物不会飘；!!!车动起来，障碍物默认还是在雷达系下位置与上一帧一样，在地图系看来，障碍物永远跟车跑
			// 		{那假如车速很快，2m/s或更快，障碍物是静止，t+1时刻发生漏检，t时刻假设它在（x=4,y=0）,速度用的是t-1时刻的位置与t时刻位置之差除时间，
			// 		似乎可以知道t+1时刻，可能它在（x=2,y=0）；我原本想着是不是要知道真实的车速，	}
			// // --- 漏检预测逻辑 x = x + vx * dt---
            track.pos_x = track.pos_x + track.vx * delta_time;
            track.pos_y = track.pos_y + track.vy * delta_time;
			for (int c = 0; c < 4; c++) {
				track.corners[c].x = track.corners[c].x + track.vx * delta_time;
				track.corners[c].y = track.corners[c].y + track.vy * delta_time;
			}

			
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
    std::vector<std::pair<float, float>> predicted_positions;
    for (const auto& track : vtrackings) {
        float pred_x = track.pos_x + track.vx * delta_time;
        float pred_y = track.pos_y + track.vy * delta_time;
        predicted_positions.push_back({pred_x, pred_y});
    }




	// 2. 计算代价矩阵 (Cost Matrix)
	// 行是现有的 vtrackings，列是当前的 detections
	int rows = vtrackings.size();
	int cols = detections.size();
	// std::vector<std::vector<float>> cost_matrix(rows, std::vector<float>(cols, 0.0f));
	const float INF = 1e6f;
	std::vector<std::vector<float>> cost_matrix(rows, std::vector<float>(cols, INF));

	// 填充代价矩阵：计算每个检测框与每个现有轨迹的距离
	for (int t = 0; t < rows; t++) {
		for (int d = 0; d < cols; d++) {

			// // 使用卡尔曼滤波预测的位置进行匹配
            // float pred_x = vtrackings[t].kf ? vtrackings[t].kf->getPosX() : vtrackings[t].pos_x;
            // float pred_y = vtrackings[t].kf ? vtrackings[t].kf->getPosY() : vtrackings[t].pos_y;
            // float dx = pred_x - detections[d].pos_x;
            // float dy = pred_y - detections[d].pos_y;


			float dx = predicted_positions[t].first - detections[d].pos_x;
            float dy = predicted_positions[t].second - detections[d].pos_y;

			// float dx = vtrackings[t].pos_x - detections[d].pos_x;
			// float dy = vtrackings[t].pos_y - detections[d].pos_y;
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

			// 1. 保存“上一帧的位置”
            float old_x = vtrackings[i].pos_x;
            float old_y = vtrackings[i].pos_y;

            // 2. 更新位置 (直接使用检测值)
            vtrackings[i].pos_x = detections[best_det_idx].pos_x;
            vtrackings[i].pos_y = detections[best_det_idx].pos_y;
            
            // 3. 更新速度：使用 (当前检测位置 - 上一帧位置) / dt
            // 这正是您提出的、最直观的方法！
            vtrackings[i].vx = (vtrackings[i].pos_x - old_x) / delta_time;
            vtrackings[i].vy = (vtrackings[i].pos_y - old_y) / delta_time;



			// vtrackings[i].pos_x = detections[best_det_idx].pos_x;
			// vtrackings[i].pos_y = detections[best_det_idx].pos_y;
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
			// TrackedObstacle new_track = detections[k];
			TrackedObstacle new_track(detections[k].pos_x, detections[k].pos_y, 0.0f , 0.0f); // 使用新构造函数

			// 构造函数只初始化 pos_x/pos_y/vx/vy/kf，需补拷 detections[k] 的几何字段
			new_track.pos_z   = detections[k].pos_z;
			new_track.depth   = detections[k].depth;
			new_track.width   = detections[k].width;
			new_track.height  = detections[k].height;
			for (int c = 0; c < 4; ++c) {
				new_track.corners[c] = detections[k].corners[c];
			}

			new_track.cluster_id = detections[k].cluster_id;
			new_track.id = nextTrackID++;
			new_track.age = 0;
			new_track.lastSeen = 0;
			vtrackings.push_back(new_track);
			LOG_RAW("(待定)new_track id = %d, age =%d; [%.3f, %.3f, %.3f]\n", new_track.id, new_track.age,
				new_track.pos_x, new_track.pos_y, new_track.pos_z);
			// LOG_RAW("(待定)new_track id = %d, age =%d; (平移)[%.3f, %.3f, %.3f]\n", new_track.id, new_track.age,
			// 	new_track.Translation[0], new_track.Translation[1], new_track.Translation[2]);
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
			return t.lastSeen > 5; 
			// return t.lastSeen == 1; 
		}
	);
	vtrackings.erase(it, vtrackings.end());
}




void SimpleTracker::update_V2(const std::vector<TrackedObstacle>& detections, const unsigned long long &time)
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
			

			// 思考：如果漏检，还保留障碍物位置不动，那么给到下游的融合将雷达系下目标转到地图系下。
			// 		若车不动，那障碍物不会飘；!!!车动起来，障碍物默认还是在雷达系下位置与上一帧一样，在地图系看来，障碍物永远跟车跑
			// 		{那假如车速很快，2m/s或更快，障碍物是静止，t+1时刻发生漏检，t时刻假设它在（x=4,y=0）,速度用的是t-1时刻的位置与t时刻位置之差除时间，
			// 		似乎可以知道t+1时刻，可能它在（x=2,y=0）；我原本想着是不是要知道真实的车速，	}
			// // --- 漏检预测逻辑 x = x + vx * dt---
            track.pos_x = track.pos_x + track.vx * delta_time;
            track.pos_y = track.pos_y + track.vy * delta_time;
			for (int c = 0; c < 4; c++) {
				track.corners[c].x = track.corners[c].x + track.vx * delta_time;
				track.corners[c].y = track.corners[c].y + track.vy * delta_time;
			}

			
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
    std::vector<std::pair<float, float>> predicted_positions;
    for (const auto& track : vtrackings) {
        float pred_x = track.pos_x + track.vx * delta_time;
        float pred_y = track.pos_y + track.vy * delta_time;
        predicted_positions.push_back({pred_x, pred_y});
    }




	// 2. 计算代价矩阵 (Cost Matrix)
	// 行是现有的 vtrackings，列是当前的 detections
	int rows = vtrackings.size();
	int cols = detections.size();
	// std::vector<std::vector<float>> cost_matrix(rows, std::vector<float>(cols, 0.0f));
	const float INF = 1e6f;

    std::vector<std::vector<float>>
        cost_matrix(
            rows,
            std::vector<float>(cols, INF)
        );


	// 填充代价矩阵：计算每个检测框与每个现有轨迹的距离
	for (int t = 0; t < rows; t++) {
		for (int d = 0; d < cols; d++) {

			// // 使用卡尔曼滤波预测的位置进行匹配
            // float pred_x = vtrackings[t].kf ? vtrackings[t].kf->getPosX() : vtrackings[t].pos_x;
            // float pred_y = vtrackings[t].kf ? vtrackings[t].kf->getPosY() : vtrackings[t].pos_y;
            // float dx = pred_x - detections[d].pos_x;
            // float dy = pred_y - detections[d].pos_y;


			float dx = predicted_positions[t].first - detections[d].pos_x;
            float dy = predicted_positions[t].second - detections[d].pos_y;

			// float dx = vtrackings[t].pos_x - detections[d].pos_x;
			// float dy = vtrackings[t].pos_y - detections[d].pos_y;
			float distance = sqrt(dx * dx + dy * dy);
			// cost_matrix[t][d] = distance;
		
		
			if(distance < match_threshold){
				cost_matrix[t][d] = distance;
			}
			else{
				cost_matrix[t][d] = INF;
			}
		}
	}


	// ============================================================
    //  Hungarian 全局匹配
    //
    // track_to_detection[t]：
    //
    // >= 0  : 匹配到的 Detection index
    // -1    : 没有匹配
    // ============================================================
    std::vector<int> track_to_detection;

    hungarianAssignment(cost_matrix, track_to_detection);
        
	// ============================================================
    // 5. 标记 Detection 是否已经被使用
    // ============================================================
    std::vector<bool>matched_detections(cols,false);

    for (int t = 0; t < rows; ++t) {

        int d = track_to_detection[t];

        // ========================================================
        // 5.1 Track 匹配成功
        // ========================================================
        if (d >= 0 && d < cols) {

            matched_detections[d] = true;

            // ----------------------------------------------------
            // 保存旧位置
            //
            // 这里的 old_x / old_y 是 Track 上一帧实际保存的位置。
            // ----------------------------------------------------
            float old_x = vtrackings[t].pos_x;
            float old_y = vtrackings[t].pos_y;

            // ----------------------------------------------------
            // 使用检测结果更新位置
            // ----------------------------------------------------
            vtrackings[t].pos_x = detections[d].pos_x;
            vtrackings[t].pos_y = detections[d].pos_y;
            vtrackings[t].pos_z = detections[d].pos_z;

            // ----------------------------------------------------
            // 更新速度
            //
            // 暂时继续使用你现在已经验证过的 vx / vy 方法。
            // ----------------------------------------------------
            vtrackings[t].vx = (vtrackings[t].pos_x - old_x) / delta_time;
            vtrackings[t].vy = (vtrackings[t].pos_y - old_y) / delta_time;

            // ----------------------------------------------------
            // 更新尺寸
            // ----------------------------------------------------
            vtrackings[t].depth = detections[d].depth;
            vtrackings[t].width = detections[d].width;
            vtrackings[t].height = detections[d].height;

            // ----------------------------------------------------
            // 更新 Translation
            // ----------------------------------------------------
            vtrackings[t].Translation[0] = detections[d].Translation[0];
            vtrackings[t].Translation[1] = detections[d].Translation[1];
            vtrackings[t].Translation[2] =detections[d].Translation[2];

            // ----------------------------------------------------
            // 更新 Rotation
            // ----------------------------------------------------
            for (int r = 0; r < 9; ++r) {
                vtrackings[t].Rotation[r] = detections[d].Rotation[r];
            }

            // ----------------------------------------------------
            // 更新 corners
            // ----------------------------------------------------
            for (int c = 0; c < 4; ++c) {
                vtrackings[t].corners[c] = detections[d].corners[c];
            }

            // ----------------------------------------------------
            // Track 状态
            // ----------------------------------------------------
            vtrackings[t].cluster_id =detections[d].cluster_id;
            vtrackings[t].age++;
            vtrackings[t].lastSeen = 0;

            // ----------------------------------------------------
            // 保持你原来的日志格式
            // ----------------------------------------------------
            LOG_RAW(
                " [与%d匹配上] 当前跟踪id = %d，distance = %.3f\n",
                detections[d].cluster_id,
                vtrackings[t].id,
                cost_matrix[t][d]
            );
        }
        else {
		// ========================================================
        // 5.2 Track 没有匹配
        // ========================================================
            // ----------------------------------------------------
            // 漏检：
            // 使用 vx / vy 预测位置。
            //
            // 这一点非常重要。
            //
            // 这样 Track 不是停在旧位置，
            // 而是继续向预测位置移动。
            // ----------------------------------------------------
            vtrackings[t].pos_x += vtrackings[t].vx * delta_time;
            vtrackings[t].pos_y += vtrackings[t].vy * delta_time;
			for (int c = 0; c < 4; c++) {
				// 注意：corners 是 Point2D(x,y)，没有 vx/vy 成员，这里应使用 track 自身的 vx/vy
				vtrackings[t].corners[c].x += vtrackings[t].vx * delta_time;
				vtrackings[t].corners[c].y += vtrackings[t].vy * delta_time;
			}

            vtrackings[t].lastSeen++;


            LOG_RAW("may丢失, id = %d, loss_count = %d \n",
				vtrackings[t].id, vtrackings[t].lastSeen );
        }
    }

	// ============================================================
    // 6. 处理没有被任何 Track 匹配的 Detection
    //
    // 这些 Detection 才真正创建新 Track。
    // ============================================================
   
	for (int d = 0; d < cols; d++) {
		if (matched_detections[d]) 
			continue;

		{
			// 创建新轨迹
			// TrackedObstacle new_track = detections[d];
			TrackedObstacle new_track(detections[d].pos_x, detections[d].pos_y, 0.0f , 0.0f); // 使用新构造函数

			// 构造函数只初始化了 pos_x/pos_y/vx/vy/kf，
			// 必须把 detections[d] 中 ConvertClustersToTrackedObstacles 初始化好的几何字段带过来，
			// 否则新 track 的 pos_z/depth/width/height/corners 会是未定义值（垃圾值）。
			// (Translation/Rotation 有默认成员初始化器=0，且 detections 里也是 0，无需再拷)
			new_track.pos_z   = detections[d].pos_z;
			new_track.depth   = detections[d].depth;
			new_track.width   = detections[d].width;
			new_track.height  = detections[d].height;
			for (int c = 0; c < 4; ++c) {
				new_track.corners[c] = detections[d].corners[c];
			}

			new_track.cluster_id = detections[d].cluster_id;
			new_track.id = nextTrackID++;
			new_track.age = 0;
			new_track.lastSeen = 0;
			vtrackings.push_back(new_track);
			LOG_RAW("(待定)new_track id = %d, age =%d; [%.3f, %.3f, %.3f]\n", new_track.id, new_track.age,
				new_track.pos_x, new_track.pos_y, new_track.pos_z);
			// LOG_RAW("(待定)new_track id = %d, age =%d; (平移)[%.3f, %.3f, %.3f]\n", new_track.id, new_track.age,
			// 	new_track.Translation[0], new_track.Translation[1], new_track.Translation[2]);
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


}