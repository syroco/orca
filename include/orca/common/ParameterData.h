#pragma once

#include <orca/utils/Utils.h>

namespace orca
{
namespace common
{
/**
* @brief This class holds the data for a parameter of any type
* 
*/
template<class T>
class ParameterData
{
public:
    ParameterData(){}
    virtual ~ParameterData() {}
    
    ParameterData(const T& val)
    : val_(val)
    , is_set_(true)
    {}

    template<class T2>
    ParameterData(const ParameterData<T2>& v)
    : ParameterData(v.get())
    {}
         
    T& get()
    {
        if(!is_set_)
            utils::orca_throw("Trying to get() but parameter is not set");
        return val_;
    }
    const T& get() const
    {
        if(!is_set_)
            utils::orca_throw("Trying to get() but parameter is not set");
        return val_;
    }

    void set(const T& val)
    {
        val_ = val;
        if(!is_set_)
            is_set_ = true;
    }
    
    bool isSet() const
    {
        return is_set_;
    }
private:
    bool is_set_ = false;
    T val_;
};

} // namespace common
} // namespace orca