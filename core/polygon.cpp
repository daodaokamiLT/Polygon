#include "polygon.h"
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "functional_vector.h"
#include <algorithm>
#include <stack>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
namespace polygon{


    // add xmin same x
    /**
     * @brief Must added after Sorted
     * 
     * 
    */
    template <class T>
    void Polygon<T>::AddNodeForXminsameX(point2d_t<T>* p){
        if( pminx_!= nullptr )
        if(p->x == pminx_->x){
            // printf("push succ minx.\n");
            points_.emplace_back(p);
            // printf("after points size is %d.\n", points_.size());
        }
    }
    // add xmax same x
    template <class T>
    void Polygon<T>::AddNodeForXmaxsameX(point2d_t<T>* p){
        if( pmaxx_!= nullptr )
        if(p->x == pmaxx_->x){
            // printf("push succ maxx.\n");
            points_.emplace_back(p);
            // printf("after points size is %d.\n", points_.size());
        }
    }
    // add ymin same y
    template <class T>
    void Polygon<T>::AddNodeForYminsameY(point2d_t<T>* p){
        if( pminy_!= nullptr )
        if(p->y == pminy_->y){
            // printf("push succ miny.\n");
            points_.emplace_back(p);
            // printf("after points size is %d.\n", points_.size());
        }
    }
    // add ymax same y
    template <class T>
    void Polygon<T>::AddNodeForYmaxsameY(point2d_t<T>* p){
        if( pmaxy_!= nullptr )
        if(p->y == pmaxy_->y){
            // printf("push succ maxy.\n");
            points_.emplace_back(p);
            // printf("after points size is %d.\n", points_.size());
        }
    }

    template <class T>
    void Polygon<T>::AddNodeAny(point2d_t<T>* p){
        points_.emplace_back(p);
    }

    template<class T>
    const point2d_t<T>* Polygon<T>::GetMinX_y(){
        return pminx_;
    }
    template<class T>
    const point2d_t<T>* Polygon<T>::GetMaxX_y(){
        return pmaxx_;
    }
    template<class T>
    const point2d_t<T>* Polygon<T>::GetMinY_x(){
        return pminy_;
    }
    template<class T>
    const point2d_t<T>* Polygon<T>::GetMaxY_x(){
        return pmaxy_;
    }


    template <class T, class D>
    bool compare_extreme_first(const std::pair<T, D>& elem0, const std::pair<T, D>& elem1){
        return elem0.first < elem1.first;
    }

    template <class T, class D>
    bool compare_extreme_second(const std::pair<T, D>& elem0, const std::pair<T, D>& elem1){
        return elem0.second < elem1.second;
    }

    // just used for sort
    template<class T>
    bool ToLeftTest(const point2d_t<T>* p0, const point2d_t<T>* p1){
        if(!p0->star_center || !p1->star_center || p0->star_center != p1->star_center){
            printf("error, star_center points has error.\n");
            exit(-1);
        } 
        Eigen::Matrix3d det3 = Eigen::Matrix3d::Zero();
        det3(0,0) = (double)p0->star_center->x; det3(0,1) = (double)p0->star_center->y; det3(0,2) = 1;
        det3(1,0) = (double)p0->x;              det3(1,1) = (double)p0->y;              det3(1,2) = 1;
        det3(2,0) = (double)p1->x;              det3(2,1) = (double)p1->y;              det3(2,2) = 1;
        double res = det3.determinant();
        if(res >= 0){
            return true;
        }
        else{
            return false;
        }
    }
    template <class T>
    bool ToInLeftTest(const point2d_t<T>* p0, const point2d_t<T>* p1){
        return !ToLeftTest(p0, p1);
    }


    template <class T>
    bool ToLeftTest_ISOL(const point2d_t<T>* p0, const point2d_t<T>* p1, const point2d_t<T>* p2){
        Eigen::Matrix3d det3 = Eigen::Matrix3d::Zero();
        
        double res = det3.determinant();
        det3(0,0) = (double)p0->x; det3(0,1) = (double)p0->y; det3(0,2) = 1;
        det3(1,0) = (double)p1->x; det3(1,1) = (double)p1->y; det3(1,2) = 1;
        det3(2,0) = (double)p2->x; det3(2,1) = (double)p2->y; det3(2,2) = 1;
        if(res >= 0)
            return true;
        else
            return false;
    }

    template <class T>
    bool ToLeftTestPointsPtr(const point2d_t<T>* p0, const point2d_t<T>* p1, const point2d_t<T>* p2){
        if(!p0 || !p1 || !p2){
            printf("error, points has null, error.\n");
            exit(-1);
        }
        Eigen::Matrix3d det3 = Eigen::Matrix3d::Zero();        
        det3(0,0) = (double)p0->x;           det3(0,1) = (double)p0->y;           det3(0,2) = 1;
        det3(1,0) = (double)p1->x;           det3(1,1) = (double)p1->y;           det3(1,2) = 1;
        det3(2,0) = (double)p2->x;           det3(2,1) = (double)p2->y;           det3(2,2) = 1;
        double res = det3.determinant();
        if(res >= 0){
            return true;
        }
        else if(res < 0){
            return false;
        }
    }


    template<class T>
    Polygon<T>::Polygon(std::vector<point2d_t<T>>& setpoints){
        points_.resize(setpoints.size(), nullptr);
        int count = 0;
        for(auto& p : setpoints){
            points_[count] = &p;
            ++count;
        }//get the address
    }

    template<class T>
    void Polygon<T>::SorttoStarPolygon(){
        T max_x = std::numeric_limits<T>::min(), min_x = std::numeric_limits<T>::max(), min_y = std::numeric_limits<T>::max(), max_y = std::numeric_limits<T>::min();
        unsigned long long maxx_idx=0, minx_idx=0, miny_idx=0, maxy_idx=0;
        unsigned long long count_minx = 0, count_maxx = 0, count_miny = 0, count_maxy = 0;
        unsigned long long selected_id = 0;
        for(int i=0; i<points_.size(); ++i){
            T dx = points_[i]->x;
            T dy = points_[i]->y;
            if(max_x < dx){
                max_x = dx;
                maxx_idx = i;
                count_maxx = 0;
            }else if(max_x == dx){
                ++count_maxx;
            }
            if(min_x > dx){
                min_x = dx; 
                minx_idx = i;
                count_minx = 0;
            }else if(min_x == dx){
                ++count_minx;
            }
            if(max_y < dy){
                max_y = dy;
                maxy_idx = i;
                count_maxy = 0;
            }else if(max_y == dy){
                ++count_maxy;
            }
            if(min_y > dy){
                min_y = dy;
                miny_idx = i;
                count_miny = 0;
            }else if(min_y == dy){
                ++count_miny;
            }   
        }
        bool mayresort = false;
        if (minx_idx == miny_idx)
        { // the leftbotton
            selected_id = minx_idx;
        }
        else if (maxx_idx == maxy_idx)
        { // the righttop
            selected_id = maxx_idx;
        }
        else if (minx_idx == maxy_idx)
        { // the lefttop
            selected_id = minx_idx;
        }
        else if (maxx_idx == miny_idx)
        { // the rightbottom
            selected_id = maxx_idx;
        }
        else
        {
            // if all this point cannot check a extreme point, just select a point a these four result.
            // select a count size is miniest element as extreme point

            std::array<std::pair<unsigned long long, T>, 4> extremepoints;
            extremepoints[0] = std::make_pair(count_maxx, maxx_idx);
            extremepoints[1] = std::make_pair(count_minx, minx_idx);
            extremepoints[1] = std::make_pair(count_maxy, maxy_idx);
            extremepoints[2] = std::make_pair(count_miny, miny_idx);
            std::sort(extremepoints.begin(), extremepoints.end(), compare_extreme_first<unsigned long long, T>);
            selected_id = extremepoints[0].second;
            if (extremepoints[0].first > 1)
            {
                mayresort = true;
            }
        }
        // just for test
        pminx_ = points_[minx_idx];
        pmaxx_ = points_[maxx_idx];
        pminy_ = points_[miny_idx];
        pmaxy_ = points_[maxy_idx];
        // now the result is a starpolygon.
        printf("selected element points is selected_id %lld, position is (%lf, %lf).\n", selected_id, (double)points_[selected_id]->x, (double)points_[selected_id]->y);
        
        point2d_t<T>::star_center = points_[selected_id];
        point2d_t<T>* temp = points_[0];
        points_[0] = points_[selected_id];
        points_[selected_id] = temp;
        auto iter = points_.begin();
        ++iter; // start at second.
        std::sort(iter, points_.end(), ToLeftTest<T>);
    }


    template<class T>
    void Polygon<T>::CreateExtremeEdges(){
        if(points_.size() < 3){
            printf("points size is < 3, cannot create polygon.\n");
            exit(-1);
        }
        FunctionalVectorPtr<point2d_t<T>> prepared_points((int)points_.size());
        FunctionalVectorPtr<point2d_t<T>> accepted_points((int)points_.size());
        // main is a que, push data at the end of the vector
        for(int i=0; i<2; ++i){
            accepted_points.Push(points_[i]);
        }
        for(int i=2; i<points_.size(); ++i){
            prepared_points.Push(points_[i]);
        }
        // printf("prepared_points size is: %d.\n", (int)prepared_points.Size());
        while(!prepared_points.IsEmpty()){
            
            auto ptop = prepared_points.Top();
            int curIndex = accepted_points.Size();
            if(curIndex < 2){
                printf("error, cannot create polygon.\n");
                exit(-1);
            }
            // printf("accept points curIndex-2(%d) element is (%lf, %lf), curIndex-1(%d) element is (%lf, %lf).\n", curIndex-2, accepted_points.Index(curIndex-2)->x, accepted_points.Index(curIndex-2)->y, curIndex-1, accepted_points.Index(curIndex-1)->x, accepted_points.Index(curIndex-1)->y);
            // printf("prepared_point top is (%lf, %lf).\n---------------------------------------------\n", prepared_points.Top()->x, prepared_points.Top()->y);
            if(ToLeftTestPointsPtr(accepted_points.Index(curIndex-2), accepted_points.Index(curIndex-1), prepared_points.Top())){
                // 若是Left， add into accept
                // printf("isaccept..\n");
                accepted_points.Push(prepared_points.Top());
                prepared_points.Pop_Front();
            }
            else{
                // printf("cannot accept..\n");
                accepted_points.Pop_Tail();
            }
        }
        // printf("accepted_points.size() %d.\n", accepted_points.Size());
        for(int i=0; i<accepted_points.Size()-1; ++i){
            // extreme_edges_.emplace_back();
            extreme_edges_.emplace_back(extreme_edge_t<T>(accepted_points.Index(i), accepted_points.Index(i+1)));
            extreme_points_.emplace_back(accepted_points.Index(i));
        }
        // // head and tail link
        extreme_points_.emplace_back(accepted_points.Tail());
        extreme_edges_.emplace_back(extreme_edge_t<T>(accepted_points.Tail(), accepted_points.Top()));

        // printf("extreme_edges size is %d.\n", (int)extreme_edges_.size());
    }

    template<class T>
    void Polygon<T>::GetExtremeEdges(std::vector<extreme_edge_t<T>>& exedges){
        exedges.assign(extreme_edges_.begin(), extreme_edges_.end());
    }
    template<class T>
    void Polygon<T>::GetTempPoints(std::vector<point2d_t<T>*>& resorted_points){
        resorted_points.assign(points_.begin(), points_.end());
    }
    template<class T>
    void Polygon<T>::GetExtremePoints(std::vector<point2d_t<T>*>& expoints){
        expoints.assign(extreme_points_.begin(), extreme_points_.end());
    }

}