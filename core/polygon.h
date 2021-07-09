#pragma once

#include <atomic>
#include <vector>

/**
 * 
 * 
 * @brief 在创建polygon 的时候，注意：
 * 1. 若存在多个点通用的x 或者 y 且都在polygon的extreme points 中
 * 这样会造成之后的分解工作的效果较差。
 * 
 * 
 * 
*/

#define PolygonPrintf(...) //printf(__VA_ARGS__)

namespace polygon{

template<class T>
struct point2d_t{
    static point2d_t CreatePoint(const T& x, const T& y){
        return point2d_t<T>(x, y);
    }
    unsigned long long id;
    T x;
    T y;
    static point2d_t<T>* star_center;
    point2d_t(const T &x, const T &y) : x(x), y(y)
    {
        id = idcounterP;
        ++idcounterP;
    }
    bool Equal(point2d_t<T>* p){
        if(this->x == p->x && this->y == p->y)
            return true;
        else
            return false;
    }
    private:   
        static std::atomic_ullong idcounterP;
};
template<class T>
std::atomic_ullong point2d_t<T>::idcounterP = {0};
template<class T>
point2d_t<T>* point2d_t<T>::star_center = nullptr;

typedef point2d_t<int> point2di_t;
typedef point2d_t<float> point2df_t;
typedef point2d_t<double> point2dd_t;

template <class T>
struct extreme_edge_t{
    unsigned long long id;
    point2d_t<T>* p_start;
    point2d_t<T>* p_end;
    extreme_edge_t(point2d_t<T> *ps, point2d_t<T> *pe) : p_start(ps), p_end(pe)
    {
        id = idcounterE;
        ++idcounterE;
    }
    extreme_edge_t(){
        id = idcounterE;
        ++idcounterE;
    }
    private:
        static std::atomic_ullong idcounterE;
};
template <class T>
std::atomic_ullong extreme_edge_t<T>::idcounterE = {0};
typedef extreme_edge_t<double> extreme_edge_d_t;

// manger is ptr.
template <class T>
class Polygon
{
public:
    Polygon(std::vector<point2d_t<T>> &setpoints);
    void SorttoStarPolygon();
    void CreateExtremeEdges();

    // for test
    // add xmin same x
    void AddNodeForXminsameX(point2d_t<T>* p);
    // add xmax same x
    void AddNodeForXmaxsameX(point2d_t<T>* p);
    // add ymin same y
    void AddNodeForYminsameY(point2d_t<T>* p);
    // add ymax same y
    void AddNodeForYmaxsameY(point2d_t<T>* p);

    void AddNodeAny(point2d_t<T>* p);
    const point2d_t<T>* GetMinX_y();
    const point2d_t<T>* GetMaxX_y();
    const point2d_t<T>* GetMinY_x();
    const point2d_t<T>* GetMaxY_x();

    void GetExtremePoints(std::vector<point2d_t<T>*>& expoints);
    void GetExtremeEdges(std::vector<extreme_edge_t<T>> &exedges);
    void GetTempPoints(std::vector<point2d_t<T>*>& points);
    size_t SizeOfExtremePoints(){return extreme_points_.size();}

    T GetExIndexX(int idx){return extreme_points_[idx]->x;}
    T GetExIndexY(int idx){return extreme_points_[idx]->y;}
private:
    std::vector<point2d_t<T>*> points_;
    std::vector<point2d_t<T>*> extreme_points_;
    std::vector<extreme_edge_t<T>> extreme_edges_;

    // just for test
    point2d_t<T>* pminx_ = nullptr;
    point2d_t<T>* pmaxx_ = nullptr;
    point2d_t<T>* pminy_ = nullptr;
    point2d_t<T>* pmaxy_ = nullptr;
};


template <class T, class D>
bool compare_extreme_first(const std::pair<T, D> &elem0, const std::pair<T, D> &elem1);

template <class T, class D>
bool compare_extreme_second(const std::pair<T, D> &elem0, const std::pair<T, D> &elem1);

template <class T>
bool ToLeftTestPointsPtr(const point2d_t<T> *p0, const point2d_t<T> *p1, const point2d_t<T> *p2);

template <class T>
bool ToLeftTest(const point2d_t<T>* p0, const point2d_t<T>* p1);
}

#include "polygon.cpp"

  