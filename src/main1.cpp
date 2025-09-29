/*#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <ceres/ceres.h>
#include <cmath>

// 定义误差模型
struct ProjectileCostFunctor {
    ProjectileCostFunctor(double observed_x, double observed_y, double t)
        : observed_x_(observed_x), observed_y_(observed_y), t_(t) {}

    template <typename T>
    bool operator()(const T* const params, T* residuals) const {
        T x0 = params[0];
        T y0 = params[1];
        T v_x0 = params[2];
        T v_y0 = params[3];
        T g = params[4];
        T k = params[5];
        
        T delta_t = T(t_);  // 将 t_ 转换为 T 类型
        T x_pred = x0 + v_x0 / k * (T(1.0) - ceres::exp(-k * delta_t));
        T y_pred = y0 + (v_y0 + g/k) / k * (T(1.0) - ceres::exp(-k * delta_t)) - g/k * delta_t;

        residuals[0] = x_pred - T(observed_x_);
        residuals[1] = y_pred - T(observed_y_);
        
        return true;
    }

    double observed_x_;
    double observed_y_;
    double t_;
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
    std::vector<double> times;  // 存储时间戳
    std::vector<std::pair<double, double>> video_data;  // 存储 (x, y) 坐标

    // 获取视频帧率（FPS）
    double fps = cap.get(cv::CAP_PROP_FPS);  // 获取帧率
    std::cout << "FPS: " << fps << std::endl;

    cv::Mat frame, gray, blurred;

    // 开始读取视频帧
    int frame_index = 0;  // 帧索引，从0开始
    while (true) 
    {
        cap >> frame;
        if (frame.empty()) break;  // 如果没有帧则退出
        //imshow("original",frame);
        //cv::waitKey(0);

        // 转换为灰度图像
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        //imshow("gray",gray);
        //cv::waitKey(0);

        // 使用高斯模糊减少噪音
        cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);
        //imshow("blur",blurred);
        //cv::waitKey(0);

        // 使用霍夫圆变换检测圆形
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1, blurred.rows / 2, 50, 20, 5, 50);  // 调整最小和最大半径
        
        // 如果检测到圆形
        if (!circles.empty()) {
            std::cout << "Circles detected in this frame: " << circles.size() << std::endl;  // 调试输出
            for (size_t i = 0; i < circles.size(); i++) {
                // 获取圆心坐标 (x, y) 和半径 r
                cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);

                // 输出检测到的圆心坐标和半径
                std::cout << "Circle center: (" << center.x << ", " << center.y << "), radius: " << radius << std::endl;

                // 在图像上绘制圆形
                cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 3);
                cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), 3);

                // 将当前帧的时间戳和圆心坐标保存到 vector 中
                double time_stamp = frame_index / fps;  // 计算当前帧的时间戳
                times.push_back(time_stamp);  // 保存时间戳
                video_data.push_back(std::make_pair(center.x, center.y));  // 保存 (x, y) 坐标
            }
        } else {
            std::cout << "No circles detected in this frame." << std::endl;  // 如果没有检测到圆，输出信息
        }

        // 显示当前帧
        cv::imshow("Frame", frame);

        // 按 'q' 键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }

        frame_index++;  // 增加帧索引
    }

    // 释放视频资源
    cap.release();
    cv::destroyAllWindows();

    // 保存坐标数据到 CSV 文件
    std::ofstream outFile("positions.csv");
    outFile << "time,x,y\n";  // CSV 文件的表头
    for (size_t i = 0; i < times.size(); ++i) {
        outFile << times[i] << "," << video_data[i].first << "," << video_data[i].second << "\n";
    }
    outFile.close();

    std::cout << "Detected positions saved to 'positions.csv'" << std::endl;

    // 初始化 Ceres 问题
    ceres::Problem problem;
    
    // 初始值
    double params[6] = {0.0, 0.0, 10.0, 10.0, 9.81, 0.1};  // x0, y0, v_x0, v_y0, g, k

    // 确保 Ceres 知道这些参数，首先添加这些参数到问题中
    problem.AddParameterBlock(params, 6);

    // 添加数据（已获取视频数据，x, y 和对应的时间 t）
    for (size_t i = 0; i < times.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ProjectileCostFunctor, 2, 6>(
                new ProjectileCostFunctor(video_data[i].first, video_data[i].second, times[i])
            );
        problem.AddResidualBlock(cost_function, nullptr, params);
    }

    // 设置参数范围之前，确保参数块已经添加
    problem.SetParameterLowerBound(params, 4, 100.0); // g 的范围
    problem.SetParameterUpperBound(params, 4, 1000.0);
    problem.SetParameterLowerBound(params, 5, 0.01);  // k 的范围
    problem.SetParameterUpperBound(params, 5, 1.0);
    
    // 配置 Ceres 求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR; // 推荐的求解器类型
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    options.minimizer_progress_to_stdout = true;      // 显示优化进度
    options.max_num_iterations = 200;                 // 增加迭代次数确保收敛
    options.function_tolerance = 1e-8;                // 更严格的收敛条件
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-8;

    // 使用更鲁棒的损失函数来抑制异常值
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    // 在添加残差块时使用损失函数
    for (size_t i = 0; i < times.size(); ++i) 
    {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ProjectileCostFunctor, 2, 6>(
                new ProjectileCostFunctor(video_data[i].first, video_data[i].second, times[i])
            );
        problem.AddResidualBlock(cost_function, loss_function, params);
    }
    // 输出求解结果
    std::cout << summary.FullReport() << std::endl;
    std::cout << "Optimal parameters: ";
    std::cout << "x0 = " << params[0] << ", y0 = " << params[1] << ", ";
    std::cout << "v_x0 = " << params[2] << ", v_y0 = " << params[3] << ", ";
    std::cout << "g = " << params[4] << ", k = " << params[5] << std::endl;

    return 0;
}

void calculateAccuracy(const double* params, 
                     const std::vector<double>& times,
                     const std::vector<std::pair<double, double>>& data) 
{
    
    double total_error_x = 0, total_error_y = 0;
    double max_error = 0;
    
    std::cout << "\n=== 拟合精度分析 ===" << std::endl;
    
    for (size_t i = 0; i < times.size(); ++i) 
    {
        double t = times[i];
        double x_pred = params[0] + params[2] / params[5] * (1 - exp(-params[5] * t));
        double y_pred = params[1] + (params[3] + params[4]/params[5]) / params[5] * 
                       (1 - exp(-params[5] * t)) - params[4]/params[5] * t;
        
        double error_x = fabs(x_pred - data[i].first);
        double error_y = fabs(y_pred - data[i].second);
        double total_error = sqrt(error_x * error_x + error_y * error_y);
        
        total_error_x += error_x;
        total_error_y += error_y;
        max_error = std::max(max_error, total_error);
        
        // 计算相对误差百分比
        double range_x = 1.0;  // 根据实际数据范围调整
        double range_y = 1.0;
        double relative_error = (error_x/range_x + error_y/range_y) / 2 * 100;
        
        std::cout << "点 " << i << ": 误差=" << total_error << "px, 相对误差=" 
                  << relative_error << "%" << std::endl;
    }
    
    double avg_error_x = total_error_x / times.size();
    double avg_error_y = total_error_y / times.size();
    double avg_total_error = (avg_error_x + avg_error_y) / 2;
    
    std::cout << "平均误差: X=" << avg_error_x << "px, Y=" << avg_error_y << "px" << std::endl;
    std::cout << "最大误差: " << max_error << "px" << std::endl;
    
    // 检查是否满足3%误差要求
    if (avg_total_error < 3.0) 
    {
        std::cout << "满足3%误差要求!" << std::endl;
    } 
    else 
    {
        std::cout << "未满足3%误差要求!" << std::endl;
    }

}
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <ceres/ceres.h>
#include <cmath>

// 定义误差模型
struct ProjectileCostFunctor {
    ProjectileCostFunctor(double observed_x, double observed_y, double t)
        : observed_x_(observed_x), observed_y_(observed_y), t_(t) {}

    template <typename T>
    bool operator()(const T* const params, T* residuals) const {
        T x0 = params[0];
        T y0 = params[1];
        T v_x0 = params[2];
        T v_y0 = params[3];
        T g = params[4];
        T k = params[5];
        
        T delta_t = T(t_);
        T x_pred = x0 + v_x0 / k * (T(1.0) - ceres::exp(-k * delta_t));
        T y_pred = y0 + (v_y0 + g/k) / k * (T(1.0) - ceres::exp(-k * delta_t)) - g/k * delta_t;

        residuals[0] = x_pred - T(observed_x_);
        residuals[1] = y_pred - T(observed_y_);
        
        return true;
    }

    double observed_x_;
    double observed_y_;
    double t_;
};

// 计算拟合精度
void calculateAccuracy(const double* params, 
                     const std::vector<double>& times,
                     const std::vector<std::pair<double, double>>& data) 
{
    double total_error_x = 0, total_error_y = 0;
    double max_error = 0;
    
    std::cout << "\n=== 拟合精度分析 ===" << std::endl;
    
    for (size_t i = 0; i < times.size(); ++i) 
    {
        double t = times[i];
        double x_pred = params[0] + params[2] / params[5] * (1 - exp(-params[5] * t));
        double y_pred = params[1] + (params[3] + params[4]/params[5]) / params[5] * 
                       (1 - exp(-params[5] * t)) - params[4]/params[5] * t;
        
        double error_x = fabs(x_pred - data[i].first);
        double error_y = fabs(y_pred - data[i].second);
        double total_error = sqrt(error_x * error_x + error_y * error_y);
        
        total_error_x += error_x;
        total_error_y += error_y;
        max_error = std::max(max_error, total_error);
        
        std::cout << "点 " << i << ": 预测(" << x_pred << ", " << y_pred 
                  << ") 实际(" << data[i].first << ", " << data[i].second 
                  << ") 误差=" << total_error << "px" << std::endl;
    }
    
    double avg_error_x = total_error_x / times.size();
    double avg_error_y = total_error_y / times.size();
    
    std::cout << "平均误差: X=" << avg_error_x << "px, Y=" << avg_error_y << "px" << std::endl;
    std::cout << "最大误差: " << max_error << "px" << std::endl;
}

// 智能初始值估计
void estimateInitialParams(double* params, 
                          const std::vector<std::pair<double, double>>& data, 
                          const std::vector<double>& times) 
{
    if (data.size() < 2) {
        // 如果数据不足，使用默认值
        params[0] = data[0].first;
        params[1] = data[0].second;
        params[2] = 100.0;
        params[3] = 100.0;
        params[4] = 500.0;  // g在100-1000范围内取中间值
        params[5] = 0.1;    // k在0.01-1.0范围内取中间值
        return;
    }
    
    // 使用前几个点估计初始速度
    double dx = data[1].first - data[0].first;
    double dy = data[1].second - data[0].second;
    double dt = times[1] - times[0];
    
    params[0] = data[0].first;    // x0 用第一个检测点
    params[1] = data[0].second;   // y0 用第一个检测点  
    params[2] = dx / dt;          // v_x0 用差分估计
    params[3] = dy / dt;          // v_y0 用差分估计
    params[4] = 500.0;            // g 取范围中间值
    params[5] = 0.1;              // k 取范围中间值
    
    std::cout << "初始参数估计: v_x0=" << params[2] << ", v_y0=" << params[3] << std::endl;
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
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "FPS: " << fps << std::endl;

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
    } else 
    {
        std::cout << "Frame " << frame_index << ": No circles detected." << std::endl;
    }

    // 显示当前帧（可选，可以注释掉以提高处理速度）
    cv::imshow("Frame", frame);
    if (cv::waitKey(1) == 'q') {
        break;
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

// 初始值（使用智能估计）
double params[6];
estimateInitialParams(params, video_data, times);

// 添加参数块
problem.AddParameterBlock(params, 6);

// 设置参数边界
problem.SetParameterLowerBound(params, 4, 100.0);   // g 下限
problem.SetParameterUpperBound(params, 4, 1000.0);  // g 上限
problem.SetParameterLowerBound(params, 5, 0.01);    // k 下限
problem.SetParameterUpperBound(params, 5, 1.0);     // k 上限

// 使用鲁棒的损失函数来抑制异常值
ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

// 添加残差块
for (size_t i = 0; i < times.size(); ++i) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ProjectileCostFunctor, 2, 6>(
            new ProjectileCostFunctor(video_data[i].first, video_data[i].second, times[i])
        );
    problem.AddResidualBlock(cost_function, loss_function, params);
}

// 配置 Ceres 求解器（在求解之前设置！）
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_SCHUR;
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
std::cout << "x0 = " << params[0] << ", y0 = " << params[1] << ", ";
std::cout << "v_x0 = " << params[2] << " px/s, v_y0 = " << params[3] << " px/s, ";
std::cout << "g = " << params[4] << " px/s², k = " << params[5] << " 1/s" << std::endl;

// 计算精度
calculateAccuracy(params, times, video_data);

return 0;

}