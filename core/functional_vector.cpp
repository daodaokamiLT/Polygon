#include "functional_vector.h"

namespace polygon
{   
    /**
     * Get current element size
     * must <= fullcontained size
     * 
    */
    template <class T>
    int FunctionalVector<T>::Size()
    {   
        if(end_index_ == start_index_){
            return 0;
        }
        else if(end_index_ < start_index_){
            return datas_.size() - (start_index_ - end_index_);
        }
        else{// if(end_index_ > start_index_){
            return end_index_ - start_index_;
        }
    }
    template <class T>  
    bool FunctionalVector<T>::IsEmpty(){
        if(end_index_ == start_index_)
            return true;
        else
            return false;
    }
    template <class T>
    bool FunctionalVector<T>::IsFull(){
        if(start_index_ - 1 == end_index_ || end_index_ + 1 == start_index_ || start_index_ + full_contained_size_ == end_index_ || end_index_ - full_contained_size_ == start_index_)
            return true;
        else
            return false;
    }
    
    template <class T>
    bool FunctionalVector<T>::Push(const T &t)
    {
        if(IsFull()){
            return false;
        }
        else{
            datas_[end_index_] = t;
            ++end_index_;
            if (end_index_ == datas_.size())
            {
                end_index_ = 0;
            }
        }
    }

    template <class T>
    void FunctionalVector<T>::Push_Force(const T &t){
        datas_[end_index_] = t;
        ++end_index_;
        // printf("endIndex is %d, data size is %d.", end_index_, (int)datas_.size());
        if (end_index_ == datas_.size())
        {
            end_index_ = 0;
        }
        if(start_index_ == end_index_){
            ++ start_index_;
        }
    }

    template <class T>
    const T& FunctionalVector<T>::Top()
    {
        if(IsEmpty()){
            printf("no element is container.\n");
            return nan_;
        }
        return datas_[start_index_];
    }

    template <class T>
    const T& FunctionalVector<T>::Tail()
    {
        if(IsEmpty){
            printf("no element is container.\n");
            return nan_;
        }

        int temp_end_index = end_index_ - 1;
        if(temp_end_index < 0){
            temp_end_index += datas_.size();
        }
        return datas_[temp_end_index];
    }

    template <class T>
    void FunctionalVector<T>::Pop_Front()
    {
        if(IsEmpty()){
            return;
        }
        ++ start_index_;
        if(start_index_ == datas_.size())
            start_index_ = 0;
    }

    template <class T>
    void FunctionalVector<T>::Pop_Tail()
    {
        if(IsEmpty()){
            return ;
        }
        // printf("start index %d, end index %d.\n", start_index_, end_index_);
        int temp_end_index = end_index_-1;
        if(temp_end_index < 0){
            temp_end_index += datas_.size();
        }
        end_index_ = temp_end_index;
    }

    template <class T>
    void FunctionalVector<T>::Clear()
    {
        // printf("clearing...\n");
        start_index_ = end_index_ = 0;
    }

    template <class T>
    std::string FunctionalVector<T>::ToString()
    {
        std::string str = "";
        if (IsEmpty())
        {
            return str + "empty";
        }
        int temp_index = start_index_;
        int count = 0;
        while (true)
        {
            T &t = datas_[temp_index];
            ++count;
            str += std::to_string(t);
            str += " ";
            if(temp_index != end_index_){
                ++temp_index;
                if(temp_index == datas_.size()){
                    temp_index = 0;
                }
                if(temp_index == end_index_){
                    break;
                }  
            }
        }
        if(count != Size()){
            printf("count size is not equal with function.\n");
            exit(-1);
        }
        return str;
    }

    template <class T>
    const T& FunctionalVector<T>::Index(int id) // start with 0
    {
        if(id >= Size()){
            printf("error, out of range.\n");
            exit(-1);
        }
        else{
            int index = start_index_ + id;
            if(index >= datas_.size()){
                index -= datas_.size();
            }
            return datas_[index];
        }
    }

    /**
     * 
     * just for ptr
     * 
     * 
     * **/   

    template <class T>
    int FunctionalVectorPtr<T>::Size()
    {   
        if(end_index_ == start_index_){
            return 0;
        }
        else if(end_index_ < start_index_){
            return datas_.size() - (start_index_ - end_index_);
        }
        else{// if(end_index_ > start_index_){
            return end_index_ - start_index_;
        }
    }

    template <class T>  
    bool FunctionalVectorPtr<T>::IsEmpty(){
        if(end_index_ == start_index_)
            return true;
        else
            return false;
    }
    template <class T>
    bool FunctionalVectorPtr<T>::IsFull(){
        if(start_index_ - 1 == end_index_ || end_index_ + 1 == start_index_ || start_index_ + full_contained_size_ == end_index_ || end_index_ - full_contained_size_ == start_index_)
            return true;
        else
            return false;
    }
    
    template <class T>
    bool FunctionalVectorPtr<T>::Push(T* t)
    {
        if(IsFull()){
            return false;
        }
        else{
            datas_[end_index_] = t;
            ++end_index_;
            if (end_index_ == datas_.size())
            {
                end_index_ = 0;
            }
        }
    }

    template <class T>
    void FunctionalVectorPtr<T>::Push_Force(T* t){
        datas_[end_index_] = t;
        ++end_index_;
        // printf("endIndex is %d, data size is %d.", end_index_, (int)datas_.size());
        if (end_index_ == datas_.size())
        {
            end_index_ = 0;
        }
        if(start_index_ == end_index_){
            if(!shallowcopy_){
                delete start_index_;
            }
            datas_[start_index_] == nullptr;
            ++ start_index_;
        }
    }

    template <class T>
    T* FunctionalVectorPtr<T>::Top()
    {
        if(IsEmpty()){
            printf("no element is container.\n");
            return nan_;
        }
        return datas_[start_index_];
    }

    template <class T>
    T* FunctionalVectorPtr<T>::Tail()
    {
        if(IsEmpty()){
            printf("no element is container.\n");
            return nan_;
        }

        int temp_end_index = end_index_ - 1;
        if(temp_end_index < 0){
            temp_end_index += datas_.size();
        }
        return datas_[temp_end_index];
    }

    template <class T>
    void FunctionalVectorPtr<T>::Pop_Front()
    {
        if(IsEmpty()){
            return;
        }
        if(!shallowcopy_ && datas_[start_index_]){
            delete datas_[start_index_];
        }
        datas_[start_index_] = nullptr;
        ++ start_index_;
        if(start_index_ == datas_.size())
            start_index_ = 0;
    }

    template <class T>
    void FunctionalVectorPtr<T>::Pop_Tail()
    {
        if(IsEmpty()){
            return ;
        }
        // printf("start index %d, end index %d.\n", start_index_, end_index_);
        int temp_end_index = end_index_-1;
        if(temp_end_index < 0){
            temp_end_index += datas_.size();
        }
        if(!shallowcopy_ && datas_[temp_end_index]){
            delete datas_[temp_end_index];
        }
        end_index_ = temp_end_index;
    }

    template <class T>
    void FunctionalVectorPtr<T>::Clear()
    {
        // printf("clearing...\n");
        for(int i=0; i<datas_.size(); ++i){
            if(!shallowcopy_ && datas_[i]){
                delete datas_[i];
            }
            datas_[i] = nullptr;
        }
        start_index_ = end_index_ = 0;
    }

    template <class T>
    std::string FunctionalVectorPtr<T>::ToString()
    {
        std::string str = "";
        if (IsEmpty())
        {
            return str + "empty";
        }
        int temp_index = start_index_;
        int count = 0;
        while (true)
        {
            T* t = datas_[temp_index];
            ++count;
            str += std::to_string(*t);
            str += " ";
            if(temp_index != end_index_){
                ++temp_index;
                if(temp_index == datas_.size()){
                    temp_index = 0;
                }
                if(temp_index == end_index_){
                    break;
                }  
            }
        }
        if(count != Size()){
            printf("count size is not equal with function.\n");
            exit(-1);
        }
        return str;
    }

    template <class T>
    T* FunctionalVectorPtr<T>::Index(int id) // start with 0
    {
        if(id >= Size()){
            printf("error, out of range.\n");
            exit(-1);
        }
        else{
            int index = start_index_ + id;
            if(index >= datas_.size()){
                index -= datas_.size();
            }
            return datas_[index];
        }
    }
}
