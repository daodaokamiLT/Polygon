#pragma once

// use the array to manage the lockedsize array
#include <vector>
#include <string>
// cannot use ptr
// forbiden to use a array ptr as T
namespace polygon{
template <class T>
class FunctionalVector{
    public:
        FunctionalVector(const int containedSize = 100):datas_(containedSize+1), full_contained_size_(containedSize){}
        ~FunctionalVector(){
            datas_.clear();
            start_index_ = 0;
            end_index_ = 0;
        }
        int Size();      // return currnt valid size
        bool Push(const T &t); // push data in
        void Push_Force(const T &t);
        const T& Top();  // return a shallow copu

        const T& Tail();
        void Pop_Front();
        void Pop_Tail();
        void Clear();
        const T& Index(int id);

        bool IsEmpty();
        bool IsFull();
        std::string ToString();
    private:
        std::vector<T> datas_;
        // [start_index, end_index)
        int start_index_ = 0;       
        int end_index_ = 0;
        int full_contained_size_;
        // start_index + 1 == end_index is full
        // start_index == end_index no data
        T nan_; // 只要默认构造函数的都可以判断
};

/**
 * 
 * a just shallow copy or depth copy?
 * 
 * to manage this result. and cannot use the array data in this vec.
*/
template <class T>
class FunctionalVectorPtr{
    public:
        FunctionalVectorPtr(const int containedSize = 100, const bool shallowcopy = true):datas_(containedSize+1), full_contained_size_(containedSize), shallowcopy_(shallowcopy){}
        ~FunctionalVectorPtr(){
            if(!shallowcopy_){
                for(auto& ptr : datas_){
                    delete ptr;
                }
            }
            datas_.clear();
            start_index_ = 0;
            end_index_ = 0;
        }
        
        int Size();      // return currnt valid size
        bool Push(T* t); // push data in
        void Push_Force(T* t);
        T* Top();  // return a shallow copu

        T* Tail();
        void Pop_Front();
        void Pop_Tail();
        void Clear();
        T* Index(int id);

        bool IsEmpty();
        bool IsFull();
        std::string ToString();

    private:
        std::vector<T*> datas_;
        // [start_index, end_index)
        int start_index_ = 0;       
        int end_index_ = 0;
        int full_contained_size_;
        // start_index + 1 == end_index is full
        // start_index == end_index no data
        T* nan_ = nullptr; // 只要默认构造函数的都可以判断
        bool shallowcopy_ = true;
};
}

#include "functional_vector.cpp"

