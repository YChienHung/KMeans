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
    // �����������
    default_random_engine e((int) time(nullptr));
    uniform_int_distribution<int> u(begin, end);

    //���������������ɵ������
    list<int> l;

    while (l.size() < count) {
        l.push_back(u(e));
        l.sort();//����
        l.unique();//ȥ�����ڵ��ظ�������еĵ�һ��
    }

    return l;
}

//----------------------------------------------------------------
// 18�ܺ���,����19��
// �Կμ�Ϊ��
// ѡ�� ��� ���� ���(2,5��URDF/Xacro)
// ROS��������
// ��ʹ�ù�����ι�������
// ����Linux ����
// *** ROS ͨѶ ����, ʹ�ó�����Ӧ��
// ����API�������ߡ������� ����ʲô����
// ����ģ�͵Ľṹ����ʶ��
// ��ע��
// ���������й���launch�������﷨  ����ǩ ���ԣ�  ���⣬�ڵ㣬���Ƶ�������������������������֣��������ͣ�
// ������� ��tf����任����̬����̬��������任
// �����˷��棬���������ȱ�㣬�﷨��Xacro����Ż�URDF
//----------------------------------------------------------------

// ��¼�����е���������
vector<KMeansPoint> g_pt;
// ��������
vector<POINT> g_center_point;
// ���ĵ�����
int g_k;

int main() {
    // ���������ʽ
    cout.precision(2);
    cout.flags(std::ostream::fixed);

    // ��ʼ����������
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

    cout << "���������ĸ�����";
    cin >> g_k;

    cout << "��ʼ�������꣺";
    list<int> temp_index = GetRandom(0, g_pt.size() - 1, g_k);
    for (auto i: temp_index) {
        g_center_point.push_back(g_pt[i].pt);
        cout << "(" << g_pt[i].pt.x << "," << g_pt[i].pt.y << ")" << " ";
    }
    cout << endl;

    bool isStatic = false;
    while (!isStatic) {

        // ������������
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

        // ������������
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

        // �����Ѿ��ȶ��������ֱ仯�����ݲ��ȶ�
        isStatic = true;
        for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
            if (GetDistance(temp_center[center_index], g_center_point[center_index]) > 1e-3) {
                isStatic = false;
                break;
            }
        }
    }

    cout << "�����������꣺";
    for (auto &center_pt: g_center_point) {
        cout << "(" << center_pt.x << "," << center_pt.y << ")" << " ";
    }
    cout << endl;

    for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
        cout << "��" << center_index + 1 << "�������";
        for (int point_index = 0; point_index < g_pt.size(); ++point_index) {
            if (g_pt[point_index].cluster == center_index) {
                cout << "P" << point_index + 1;
                cout << "(" << g_pt[point_index].pt.x << "," << g_pt[point_index].pt.y << ")" << " ";
            }
        }
        cout << endl;
    }

    cout << "�������ԣ�";
    for (auto &i: g_pt) {
        cout << "\t" << i.cluster + 1;
    }
    cout << endl;
    for (int center_index = 0; center_index < g_center_point.size(); ++center_index) {
        cout << "��" << center_index + 1 << "����룺";
        for (auto &item: g_pt) {
            cout << "\t" << GetDistance(g_center_point[center_index], item.pt);
        }
        cout << endl;
    }

    return 0;
}