#pragma once

#include <random>
#include "../core/polygon.h"

namespace polygon{
    template <class T>
    void CreateRandomPoints2d(unsigned long long contained_size, std::pair<T, T> range_min, std::pair<T, T> range_max, std::vector<point2d_t<T>>& created_points){
        srand((unsigned)time(0));

        if(range_min.first > range_max.first){
            T min_first = range_min.first;
            range_min.first = range_max.first;
            range_max.first = min_first;
        }
        if(range_min.second > range_max.second){
            T min_second = range_min.second;
            range_min.second = range_max.second;
            range_max.second = min_second;
        }
        T range_x = (T)(range_max.first - range_min.first + 1);
        T range_y = (T)(range_max.second - range_min.second + 1);
        
        for(unsigned long long i = 0; i < contained_size; ++i){
            // the int part
            T x = rand() % (int)range_x + range_min.first;
            T y = rand() % (int)range_y + range_min.second;
            // the double part
            T dx = rand() % 10000 / 10000.;
            T dy = rand() % 10000 / 10000.;
            
            created_points.emplace_back(point2d_t<T>::CreatePoint(x+dx, y+dy));
            // printf("create p x, y (%lf, %lf).\n", created_points[created_points.size()-1].x, created_points[created_points.size()-1].y);
        }
    
    }
}

