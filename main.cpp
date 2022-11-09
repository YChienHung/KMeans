#include <iostream>
#include <ctime>
#include <vector>
#include <list>
#include <random>

using namespace std;

typedef struct {
    double x;
    double y;
} POINT;

typedef struct {
    POINT pt;
    int cluster;
} KMeansPoint;

double GetDistance(POINT pt1, POINT pt2) {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

list<int> GetRandom(int begin, int end, int count) {
    // 置随机数种子
    default_random_engine e((int) time(nullptr));
    uniform_int_distribution<int> u(begin, end);

    //定义链表，保存生成的随机数
    list<int> l;

    while (l.size() < count) {
        l.push_back(u(e));
        l.sort();//排序
        l.unique();//去除相邻的重复随机数中的第一个
    }

    return l;
}

//----------------------------------------------------------------
// 18周后面,或者19周
// 以课件为主
// 选择 简答 分析 编程(2,5，URDF/Xacro)
// ROS基本概念
// 不使用工具如何构建工程
// 常用Linux 命令
// *** ROS 通讯 三种, 使用场景，应用
// 常用API，订阅者、发布者 ，跟什么参数
// 理论模型的结构，能识别
// 加注释
// 第三章运行管理，launch，基本语法  ，标签 属性，  话题，节点，名称的重命名方法，解决方案有两种，三种类型，
// 常用组件 ，tf坐标变换，静态，动态，多坐标变换
// 机器人仿真，基本概念，优缺点，语法，Xacro如何优化URDF
//----------------------------------------------------------------

// 记录棋盘中的所有坐标
vector<KMeansPoint> g_pt;
// 中心坐标
vector<POINT> g_center_point;
// 中心的数量
int g_k;

int main() {
    // 定义输出格式
    cout.precision(2);
    cout.flags(std::ostream::fixed);

    // 初始化样本坐标
    g_pt.push_back({0, 0, 0});
    g_pt.push_back({5, 6, 0});
    g_pt.push_back({1, 2, 0});
    g_pt.push_back({2, 7, 0});
    g_pt.push_back({4, 3, 0});
    g_pt.push_back({6, 3, 0});
    g_pt.push_back({2, 1, 0});
    g_pt.push_back({5, 5, 0});
    g_pt.push_back({7, 8, 0});
    g_pt.push_back({8, 1, 0});

    cout << "请输入中心个数：";
    cin >> g_k;

    cout << "初始中心坐标：";
    list<int> temp_index = GetRandom(0, g_pt.size() - 1, g_k);
    for (auto i: temp_index) {
        g_center_point.push_back(g_pt[i].pt);
        cout << "(" << g_pt[i].pt.x << "," << g_pt[i].pt.y << ")" << " ";
    }
    cout << endl;

    bool isStatic = false;
    while (!isStatic) {

        // 更新样本归类
        double min_distance;
        double temp_distance;
        vector<POINT> temp_center;
        for (auto &item: g_pt) {
            min_distance = GetDistance(item.pt, g_center_point[0]);
            for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
                temp_distance = GetDistance(item.pt, g_center_point[center_index]);
                if ((min_distance - temp_distance) > 1e-6) {
                    min_distance = temp_distance;
                    item.cluster = center_index;
                }
            }
        }

        // 更新中心坐标
        POINT temp_pt;
        int cluster_count;
        temp_center.clear();
        for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
            temp_center.push_back(g_center_point[center_index]);
            temp_pt.x = 0;
            temp_pt.y = 0;
            cluster_count = 0;
            for (auto &item: g_pt) {
                if (item.cluster == center_index) {
                    temp_pt.x += item.pt.x;
                    temp_pt.y += item.pt.y;
                    cluster_count++;
                }
            }

            if (cluster_count != 0) {
                g_center_point[center_index].x = temp_pt.x / cluster_count;
                g_center_point[center_index].y = temp_pt.y / cluster_count;
            }
        }

        // 假设已经稳定，若出现变化则数据不稳定
        isStatic = true;
        for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
            if (GetDistance(temp_center[center_index], g_center_point[center_index]) > 1e-3) {
                isStatic = false;
                break;
            }
        }
    }

    cout << "最终中心坐标：";
    for (auto &center_pt: g_center_point) {
        cout << "(" << center_pt.x << "," << center_pt.y << ")" << " ";
    }
    cout << endl;

    for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
        cout << "第" << center_index + 1 << "类包括：";
        for (int point_index = 0; point_index < g_pt.size(); ++point_index) {
            if (g_pt[point_index].cluster == center_index) {
                cout << "P" << point_index + 1;
                cout << "(" << g_pt[point_index].pt.x << "," << g_pt[point_index].pt.y << ")" << " ";
            }
        }
        cout << endl;
    }

    cout << "归类属性：";
    for (auto &i: g_pt) {
        cout << "\t" << i.cluster + 1;
    }
    cout << endl;
    for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
        cout << "第" << center_index + 1 << "类距离：";
        for (auto &item: g_pt) {
            cout << "\t" << GetDistance(g_center_point[center_index], item.pt);
        }
        cout << endl;
    }

    return 0;
}