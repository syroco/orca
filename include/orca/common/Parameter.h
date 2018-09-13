#pragma once
#include <orca/utils/Utils.h>
#include <exception>
// TODO : Try to remove this from headers
#include <yaml-cpp/yaml.h>

// Support for Eigen Vectors and Matrices
namespace YAML {

  template < typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
  struct convert< Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> >
  {
    static Node encode(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
    {
      Node node(NodeType::Sequence);

      // Save data given as a vector
      if (_Rows == 1 || _Cols == 1) {
        for (int row=0; row<matrix.rows(); row++)
          for (int col=0; col<matrix.cols(); col++)
            node.push_back(matrix(row,col));
        return node;
      }

      return node;
    }

    static bool decode(const Node& node, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
    {
      // Read data given as a vector
      if (_Rows == 1 || _Cols == 1) {
        (_Rows == 1 ? matrix.resize(_Rows, node.size()) : matrix.resize(node.size(), _Cols));
        for (int id=0; id<node.size(); id++)
          (node[0].size() == 0 ? matrix(id) = node[(int)id].as<_Scalar>() :  matrix(id) = node[(int)id][0].as<_Scalar>());
        return true;
      }

      return true;
    }
  };

} // namespace YAML

// Hack to remove shared_ptr YAML support error
namespace YAML {
  template < typename T>
  struct convert< std::shared_ptr<T> >
  {
    static Node encode(const std::shared_ptr<T>& s)
    {
      (void)s;
      return Node();
    }

    static bool decode(const Node& node, std::shared_ptr<T>& e)
    {
      return true;
    }
  };
}


namespace orca
{
namespace common
{

/**
* @brief The ParamPolicy defines if an error should be thrown when trying to do anything with the task
* before all #Required parameters are set at least once.
*/
enum class ParamPolicy
{
    Required = 0
    , Optional
};

/**
* @brief ParameterBase is the public interface to any parameter
* 
*/
class ParameterBase
{
public:
    using Ptr = std::shared_ptr<ParameterBase>;

    bool loadFromString(const std::string& s)
    {
        try 
        {
            if(onLoadFromString(s))
            {
                if(loading_success_)
                    loading_success_();
                return true;
            }
        } catch(std::exception& e) {
            utils::orca_throw(e.what());
        }

        if(loading_failed_) 
            loading_failed_();
        return false;
    }
    virtual void print() const = 0;
    virtual bool isSet() const = 0;
    const std::string& getName() const { return name_; }
    void setName(const std::string& name) { name_ = name; }
    void setRequired(bool is_required) { is_required_ = is_required; }
    bool isRequired() const { return is_required_; }
    void onLoadingSuccess(std::function<void(void)> loading_success_callback)
    {
        loading_success_ = loading_success_callback;
    }
    void onLoadingFailed(std::function<void(void)> loading_failed_callback)
    {
        loading_failed_ = loading_failed_callback;
    }
protected:
    virtual bool onLoadFromString(const std::string& s) = 0;
private:
    std::string name_;
    bool is_required_ = false;
    bool is_sub_param_ = false;
    std::function<void(void)> loading_success_;
    std::function<void(void)> loading_failed_;
};

template<class T>
/**
* @brief This class contains the data and throws exceptions if trying to access it but not set.
* 
*/
class ParameterData
{
public:
    ParameterData(){}
    
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
            utils::orca_throw("ParameterData is not set");
        return val_;
    }
    const T& get() const
    {
        if(!is_set_)
            utils::orca_throw("ParameterData is not set");
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



/**
* @brief This class holds the conversion from a string (YAML string) to the data type
* 
*/
template<class T>
class Parameter : public ParameterBase, public ParameterData<T>
{
public:
    bool onLoadFromString(const std::string& s)
    {
        YAML::Node node = YAML::Load(s);
        try
        {
            this->set( node.as<T>() );
        }
        catch(std::exception& e)
        {
            utils::orca_throw(utils::Formatter() << " - " << getName() << ": " << "Could not convert \"" << s << "\" to the type asked\n" << e.what());
        }
        return true;
    }

    void print() const
    {
        std::cout << " - " << getName() << ": " << this->get() << '\n';
    }
    
    bool isSet() const
    {
        return ParameterData<T>::isSet();
    }
    
    template<class T2>
    Parameter<T>& operator=(T2 val)
    {
        this->set(val);
        return *this;
    }
// TODO : figure out how to remove the need to param->get()
// These dont work yet
//     template<class T2>
//     T2& operator=(Parameter<T>& val)
//     {
//         return this->get();
//     }
//     template<class T2>
//     T2& operator=(const Parameter<T>& val)
//     {
//         return this->get();
//     }
};

} // namespace common
} // namespace orca

template<class T>
inline ::std::ostream& operator<<(::std::ostream& os, const orca::common::Parameter<T>& p)
{
    os << p.get();
    return os;
}
