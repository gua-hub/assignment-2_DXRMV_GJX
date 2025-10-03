/*#include <opencv2/opencv.hpp>
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

    return 0；
}*/
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

// 定义残差函数，来拟合小球的轨迹
struct ProjectileResidual {
    ProjectileResidual(double x, double y, double t)
        : x_(x), y_(y), t_(t) {}

    template <typename T>
    bool operator()(const T* const abgk, T* residual) const {
        // abgk[0] = x0, abgk[1] = y0, abgk[2] = v_x0, abgk[3] = v_y0, abgk[4] = g, abgk[5] = k
        T x0 = abgk[0];
        T y0 = abgk[1];
        T v_x0 = abgk[2];
        T v_y0 = abgk[3];
        T g = abgk[4];
        T k = abgk[5];

        T delta_t = T(t_);  // 将时间转换为 T 类型，假设 t_ 已经是 double 类型的时间戳

        // 水平位置 (x_pred)
        T x_pred = x0 + v_x0 / k * (T(1.0) - ceres::exp(-k * delta_t));

        // 竖直位置 (y_pred)
        T y_pred = y0 + (v_y0 + g / k) / k * (T(1.0) - ceres::exp(-k * delta_t)) - g / k * delta_t;

        // 计算残差，只有一个维度的残差
        residual[0] = x_pred - T(x_);
        residual[1] = y_pred - T(y_);

        return true;
    }

    double x_, y_, t_;
};

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

    // 初始猜测的参数
    double abgk[6] = {0.0, 0.0, 300.0, -100.0, 9.8, 0.1};  // 初始猜测的参数
    ceres::Problem problem;

    // 添加残差块
    for (size_t i = 0; i < times.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ProjectileResidual, 2, 6>(
                new ProjectileResidual(video_data[i].first, video_data[i].second, times[i])
            ),
            nullptr,  // 不使用损失函数
            abgk      // 优化的参数：x0, y0, v_x0, v_y0, g, k
        );
    }

    // 设置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 输出优化结果
    std::cout << summary.FullReport() << std::endl;
    std::cout << "优化后的参数：" << std::endl;
    std::cout << "x0 = " << abgk[0] << std::endl;
    std::cout << "y0 = " << abgk[1] << std::endl;
    std::cout << "v_x0 = " << abgk[2] << std::endl;
    std::cout << "v_y0 = " << abgk[3] << std::endl;
    std::cout << "g = " << abgk[4] << std::endl;
    std::cout << "k = " << abgk[5] << std::endl;

    return 0;
}
