/*#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <ceres/ceres.h>
#include <cmath>

// 定义误差模型
struct ProjectileCostFunctor {
    ProjectileCostFunctor(double observed_x, double observed_y, double t, double x0, double y0)
        : observed_x_(observed_x), observed_y_(observed_y), t_(t), x0_(x0), y0_(y0) {}

    template <typename T>
    bool operator()(const T* const params, T* residuals) const {
        T v_x0 = params[0];  // v_x0 初始水平速度
        T v_y0 = params[1];  // v_y0 初始竖直速度
        T g = params[2];     // g 重力加速度
        T k = params[3];     // k 空气阻力系数
        
        T delta_t = T(t_);  // 当前时间
        
        // 弹丸的水平位置 (x)
        T x_pred = x0_ + v_x0 / k * (T(1.0) - ceres::exp(-k * delta_t));
        
        // 弹丸的竖直位置 (y)
        T y_pred = y0_ + (v_y0 + g / k) / k * (T(1.0) - ceres::exp(-k * delta_t)) - g / k * delta_t;
        
        // 计算水平和竖直位置的残差
        residuals[0] = x_pred - T(observed_x_);
        residuals[1] = y_pred - T(observed_y_);
        
        return true;
    }

    double observed_x_;
    double observed_y_;
    double t_;
    double x0_;  // 初始 x 坐标
    double y0_;  // 初始 y 坐标
};

// 智能初始值估计
void estimateInitialParams(double* params, 
                          const std::vector<std::pair<double, double>>& data, 
                          const std::vector<double>& times) 
{
    if (data.size() < 2) {
        // 如果数据不足，使用默认值
        params[0] = 499.0;  // 初始水平速度
        params[1] = 100.0;  // 初始竖直速度
        params[2] = 100.0;  // g 取 100
        params[3] = 0.1;    // k 取 0.1
        return;
    }
    
    // 使用前两个点估计初始速度
    double dx = data[1].first - data[0].first;
    double dy = data[1].second - data[0].second;
    double dt = times[1] - times[0];
    
    params[0] = dx / dt;  // v_x0 用差分估计
    params[1] = dy / dt;  // v_y0 用差分估计
    params[2] = 100.0;    // g 取 100
    params[3] = 0.1;      // k 取 0.1
    
    std::cout << "初始参数估计: v_x0=" << params[0] << ", v_y0=" << params[1] << std::endl;
}

int main() 
{
    // 打开视频文件
    std::string video_path = "/home/guojinxuan/my_cmake_project/opencv_project/resources/video.mp4";
    cv::VideoCapture cap(video_path);
    
    if (!cap.isOpened()) 
    {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // 用来存储每一帧的目标位置（x, y）和时间
    std::vector<double> times;
    std::vector<std::pair<double, double>> video_data;

    // 获取视频帧率（FPS）
    double fps = 60;
    
    cv::Mat frame, gray, blurred;

    // 开始读取视频帧
    int frame_index = 0;
    while (true) 
    {
        cap >> frame;
        if (frame.empty()) break;

        // 转换为灰度图像
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 使用高斯模糊减少噪音
        cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

        // 使用霍夫圆变换检测圆形
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1, blurred.rows / 2, 50, 20, 5, 50);
        // 如果检测到圆形
        if (!circles.empty()) 
        {
            // 选择第一个检测到的圆（可以根据需要改进为选择最合适的圆）
            cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int radius = cvRound(circles[0][2]);

            std::cout << "Frame " << frame_index << ": Circle center: (" 
                      << center.x << ", " << center.y << "), radius: " << radius << std::endl;

            // 在图像上绘制圆形
            cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 3);
            cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), 3);

            // 保存数据
            double time_stamp = frame_index / fps;
            times.push_back(time_stamp);
            video_data.push_back(std::make_pair(center.x, center.y));
        } 
        else 
        {
            std::cout << "Frame " << frame_index << ": No circles detected." << std::endl;
        }

        frame_index++;
    }

    // 释放视频资源
    cap.release();
    cv::destroyAllWindows();

    if (video_data.empty()) {
        std::cerr << "错误：没有检测到任何圆！" << std::endl;
        return -1;
    }

    // 保存坐标数据到 CSV 文件
    std::ofstream outFile("positions.csv");
    outFile << "time,x,y\n";
    for (size_t i = 0; i < times.size(); ++i) {
        outFile << times[i] << "," << video_data[i].first << "," << video_data[i].second << "\n";
    }
    outFile.close();

    std::cout << "检测到 " << video_data.size() << " 个位置点，已保存到 'positions.csv'" << std::endl;

    // 初始化 Ceres 问题
    ceres::Problem problem;

    // 初始值（智能估计初始值）
    double params[4];  // 包括 v_x0, v_y0, g, k
    estimateInitialParams(params, video_data, times);

    // 获取初始位置
    double x0 = video_data[0].first;
    double y0 = video_data[0].second;

    // 添加参数块
    problem.AddParameterBlock(params, 4);

    // 设置参数边界
    problem.SetParameterLowerBound(params, 2, 100.0);   // g 下限
    problem.SetParameterUpperBound(params, 2, 1000.0);  // g 上限
    problem.SetParameterLowerBound(params, 3, 0.01);   // k 下限
    problem.SetParameterUpperBound(params, 3, 1.0);    // k 上限

    // 使用鲁棒的损失函数来抑制异常值
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    // 添加残差块
    for (size_t i = 0; i < times.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ProjectileCostFunctor, 2, 4>(
                new ProjectileCostFunctor(video_data[i].first, video_data[i].second, times[i], x0, y0)
            );
        problem.AddResidualBlock(cost_function, loss_function, params);
    }

    // 配置 Ceres 求解器（在求解之前设置！）
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-8;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 输出求解结果
    std::cout << summary.FullReport() << std::endl;
    std::cout << "最优参数: ";
    std::cout << "v_x0 = " << params[0] << " px/s, v_y0 = " << params[1] << " px/s, ";
    std::cout << "g = " << params[2] << " px/s², k = " << params[3] << " 1/s" << std::endl;

    return 0;
}
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <ceres/ceres.h>
#include <cmath>
#include <algorithm>

// 修正后的误差模型
struct ProjectileCostFunctor {
    ProjectileCostFunctor(double observed_x, double observed_y, double t)
        : observed_x_(observed_x), observed_y_(observed_y), t_(t) {}

    template <typename T>
    bool operator()(const T* const params, T* residuals) const {
        // 参数: x0, y0, v_x0, v_y0, g, k
        T x0 = params[0];
        T y0 = params[1];
        T v_x0 = params[2];
        T v_y0 = params[3];
        T g = params[4];
        T k = params[5];
        
        T delta_t = T(t_);  // Δt = t - t0, 假设 t0 = 0
        
        // 弹丸的水平位置 (x)
        T x_pred = x0 + v_x0 / k * (T(1.0) - ceres::exp(-k * delta_t));
        
        // 弹丸的竖直位置 (y)
        T y_pred = y0 + (v_y0 + g / k) / k * (T(1.0) - ceres::exp(-k * delta_t)) - g / k * delta_t;
        
        // 计算残差
        residuals[0] = x_pred - T(observed_x_);
        residuals[1] = y_pred - T(observed_y_);
        
        return true;
    }

    double observed_x_;
    double observed_y_;
    double t_;
};

// 改进的初始参数估计
void estimateInitialParams(double* params, 
                          const std::vector<std::pair<double, double>>& data, 
                          const std::vector<double>& times) 
{
    if (data.size() < 3) {
        // 默认值
        params[0] = data[0].first;   // x0
        params[1] = data[0].second;  // y0
        params[2] = 300.0;           // v_x0
        params[3] = -100.0;          // v_y0 (通常向上为负)
        params[4] = 500.0;           // g
        params[5] = 0.05;            // k
        return;
    }
    
    // 使用第一个点作为初始位置估计
    params[0] = data[0].first;
    params[1] = data[0].second;
    
    // 使用前几个点进行速度估计
    double sum_vx = 0, sum_vy = 0;
    int count = 0;
    for (size_t i = 1; i < std::min(size_t(10), data.size()); ++i) {
        double dt = times[i] - times[i-1];
        if (dt > 0) {
            sum_vx += (data[i].first - data[i-1].first) / dt;
            sum_vy += (data[i].second - data[i-1].second) / dt;
            count++;
        }
    }
    
    if (count > 0) {
        params[2] = sum_vx / count;
        params[3] = sum_vy / count;
    } else {
        params[2] = 300.0;
        params[3] = -100.0;
    }
    
    // 基于轨迹特征估计 g 和 k
    if (data.size() >= 5) {
        // 估计最大高度点
        double min_y = data[0].second;
        for (const auto& point : data) {
            if (point.second < min_y) min_y = point.second;
        }
        
        // 如果轨迹有明显上升，g 应该较大；如果较平缓，g 较小
        double vertical_range = std::abs(data[0].second - min_y);
        if (vertical_range > 50) {
            params[4] = 800.0;  // 大抛物线，重力较大
        } else {
            params[4] = 300.0;  // 较平缓，重力较小
        }
        
        // 基于水平减速估计 k
        if (data.size() >= 10) {
            double early_speed = std::abs(data[4].first - data[0].first) / (times[4] - times[0]);
            double late_speed = std::abs(data[9].first - data[5].first) / (times[9] - times[5]);
            if (late_speed > 0 && early_speed > late_speed) {
                double speed_ratio = late_speed / early_speed;
                params[5] = 0.1 * (1.0 - speed_ratio);  // 减速越大，k越大
                params[5] = std::max(0.01, std::min(1.0, params[5]));
            } else {
                params[5] = 0.05;
            }
        } else {
            params[5] = 0.05;
        }
    } else {
        params[4] = 500.0;
        params[5] = 0.05;
    }
    
    std::cout << "初始参数估计: " << std::endl;
    std::cout << "x0=" << params[0] << ", y0=" << params[1] 
              << ", v_x0=" << params[2] << ", v_y0=" << params[3] 
              << ", g=" << params[4] << ", k=" << params[5] << std::endl;
}

// 运行单次优化
double runOptimization(double* params, 
                      const std::vector<std::pair<double, double>>& video_data,
                      const std::vector<double>& times,
                      bool verbose = false) 
{
    ceres::Problem problem;

    // 添加参数块
    problem.AddParameterBlock(params, 6);

    // 设置参数边界
    problem.SetParameterLowerBound(params, 4, 100.0);   // g 下限
    problem.SetParameterUpperBound(params, 4, 1000.0);  // g 上限
    problem.SetParameterLowerBound(params, 5, 0.01);    // k 下限
    problem.SetParameterUpperBound(params, 5, 1.0);     // k 上限

    // 使用鲁棒的损失函数
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    // 添加残差块
    for (size_t i = 0; i < times.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ProjectileCostFunctor, 2, 6>(
                new ProjectileCostFunctor(video_data[i].first, video_data[i].second, times[i])
            );
        problem.AddResidualBlock(cost_function, loss_function, params);
    }

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = verbose;
    options.max_num_iterations = 200;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-8;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (verbose) {
        std::cout << summary.BriefReport() << std::endl;
    }

    return summary.final_cost;
}

int main() 
{
    // 打开视频文件
    std::string video_path = "/home/guojinxuan/my_cmake_project/opencv_project/resources/video.mp4";
    cv::VideoCapture cap(video_path);
    
    if (!cap.isOpened()) 
    {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // 用来存储每一帧的目标位置（x, y）和时间
    std::vector<double> times;
    std::vector<std::pair<double, double>> video_data;

    // 获取视频帧率（FPS）
    double fps = 60;
    
    cv::Mat frame, gray, blurred;

    // 开始读取视频帧
    int frame_index = 0;
    while (true) 
    {
        cap >> frame;
        if (frame.empty()) break;

        // 转换为灰度图像
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 使用高斯模糊减少噪音
        cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

        // 使用霍夫圆变换检测圆形
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1, blurred.rows / 2, 50, 20, 5, 50);
        
        // 如果检测到圆形
        if (!circles.empty()) 
        {
            // 选择第一个检测到的圆
            cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int radius = cvRound(circles[0][2]);

            std::cout << "Frame " << frame_index << ": Circle center: (" 
                      << center.x << ", " << center.y << "), radius: " << radius << std::endl;

            // 在图像上绘制圆形
            cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 3);
            cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), 3);

            // 保存数据
            double time_stamp = frame_index / fps;
            times.push_back(time_stamp);
            video_data.push_back(std::make_pair(center.x, center.y));
        } 
        else 
        {
            std::cout << "Frame " << frame_index << ": No circles detected." << std::endl;
        }

        frame_index++;
    }

    // 释放视频资源
    cap.release();
    cv::destroyAllWindows();

    if (video_data.empty()) {
        std::cerr << "错误：没有检测到任何圆！" << std::endl;
        return -1;
    }

    // 保存坐标数据到 CSV 文件
    std::ofstream outFile("positions.csv");
    outFile << "time,x,y\n";
    for (size_t i = 0; i < times.size(); ++i) {
        outFile << times[i] << "," << video_data[i].first << "," << video_data[i].second << "\n";
    }
    outFile.close();

    std::cout << "检测到 " << video_data.size() << " 个位置点，已保存到 'positions.csv'" << std::endl;

    // 多起点优化策略
    std::vector<double> k_candidates = {0.01, 0.05, 0.1, 0.2, 0.5};
    std::vector<double> best_params(6);
    double best_cost = std::numeric_limits<double>::max();

    std::cout << "\n开始多起点优化..." << std::endl;
    
    for (size_t i = 0; i < k_candidates.size(); ++i) {
        std::cout << "\n尝试 k_init = " << k_candidates[i] << std::endl;
        
        double current_params[6];
        estimateInitialParams(current_params, video_data, times);
        current_params[5] = k_candidates[i];  // 设置不同的 k 初始值
        
        double cost = runOptimization(current_params, video_data, times, false);
        
        std::cout << "成本: " << cost << std::endl;
        std::cout << "结果: v_x0=" << current_params[2] << ", v_y0=" << current_params[3] 
                  << ", g=" << current_params[4] << ", k=" << current_params[5] << std::endl;
        
        if (cost < best_cost) {
            best_cost = cost;
            std::copy(current_params, current_params + 6, best_params.begin());
            std::cout << "← 新的最佳解" << std::endl;
        }
    }

    // 使用最佳参数进行最终优化
    std::cout << "\n进行最终优化..." << std::endl;
    double final_params[6];
    std::copy(best_params.begin(), best_params.end(), final_params);
    runOptimization(final_params, video_data, times, true);

    // 输出最终结果
    std::cout << "\n=== 最终拟合结果 ===" << std::endl;
    std::cout << "x0 = " << final_params[0] << " px" << std::endl;
    std::cout << "y0 = " << final_params[1] << " px" << std::endl;  
    std::cout << "v_x0 = " << final_params[2] << " px/s" << std::endl;
    std::cout << "v_y0 = " << final_params[3] << " px/s" << std::endl;
    std::cout << "g = " << final_params[4] << " px/s²" << std::endl;
    std::cout << "k = " << final_params[5] << " 1/s" << std::endl;

    // 计算初始速度大小和角度
    double v0 = sqrt(final_params[2] * final_params[2] + final_params[3] * final_params[3]);
    double angle = atan2(final_params[3], final_params[2]) * 180.0 / M_PI;
    std::cout << "初始速度大小: " << v0 << " px/s" << std::endl;
    std::cout << "发射角度: " << angle << " 度" << std::endl;

    // 计算拟合误差
    std::cout << "\n=== 拟合误差分析 ===" << std::endl;
    double total_error = 0;
    double max_error = 0;
    int max_error_idx = 0;
    
    for (size_t i = 0; i < times.size(); ++i) {
        double delta_t = times[i];
        double x_pred = final_params[0] + final_params[2] / final_params[5] * 
                       (1 - exp(-final_params[5] * delta_t));
        double y_pred = final_params[1] + (final_params[3] + final_params[4]/final_params[5])/final_params[5] * 
                       (1 - exp(-final_params[5] * delta_t)) - final_params[4]/final_params[5] * delta_t;
        
        double error = sqrt(pow(x_pred - video_data[i].first, 2) + 
                           pow(y_pred - video_data[i].second, 2));
        total_error += error;
        
        if (error > max_error) {
            max_error = error;
            max_error_idx = i;
        }
        
        std::cout << "点 " << i << " (t=" << times[i] << "s): " 
                  << "预测(" << x_pred << ", " << y_pred << "), "
                  << "实际(" << video_data[i].first << ", " << video_data[i].second << "), "
                  << "误差=" << error << " px" << std::endl;
    }
    
    double avg_error = total_error / times.size();
    std::cout << "\n平均误差: " << avg_error << " px" << std::endl;
    std::cout << "最大误差: " << max_error << " px (在点 " << max_error_idx << ")" << std::endl;
    
    // 检查参数是否在要求范围内
    std::cout << "\n=== 参数范围检查 ===" << std::endl;
    if (final_params[4] >= 100.0 && final_params[4] <= 1000.0) {
        std::cout << "✓ g 在要求范围内 [100, 1000]" << std::endl;
    } else {
        std::cout << "✗ g 超出要求范围: " << final_params[4] << std::endl;
    }
    
    if (final_params[5] >= 0.01 && final_params[5] <= 1.0) {
        std::cout << "✓ k 在要求范围内 [0.01, 1.0]" << std::endl;
    } else {
        std::cout << "✗ k 超出要求范围: " << final_params[5] << std::endl;
    }

    return 0;
}