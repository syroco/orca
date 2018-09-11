#pragma once
#include <orca/utils/Utils.h>
#include <exception>
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

// Hack to remove shared_ptr (TaskBase::Ptr for example) YAML support
namespace YAML {
  template < typename T>
  struct convert< std::shared_ptr<T> >
  {
    static Node encode(const std::shared_ptr<T>& s)
    {
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

class ParameterBase : public utils::SharedPointer<ParameterBase>
{
public:
    virtual bool loadFromString(const std::string& s) = 0;
    virtual void print() const = 0;
    virtual bool isSet() const = 0;
    const std::string& getName() const { return name_; }
    void setName(const std::string& name) { name_ = name; }
    void setRequired(bool is_required) { is_required_ = is_required; }
    bool isRequired() const { return is_required_; } 
private:
    std::string name_;
    bool is_required_ = false;
};

template<class T>
class ParameterData
{
public:
    ParameterData()
    : val_(std::make_shared<T>())
    {}
    
    ParameterData(const T& val)
    : val_(std::make_shared<T>(val))
    {}

    template<class T2>
    ParameterData(const ParameterData<T2>& v)
    : ParameterData(v.get())
    {}
         
    T& get()
    {
        if(!val_)
            throw std::runtime_error(utils::Formatter() << "ParameterData is not set");
        return *val_;
    }
    const T& get() const
    {
        if(!val_)
            throw std::runtime_error(utils::Formatter() << "ParameterData is not set");
        return *val_;
    }

    void set(const T& val)
    {
        if(!val_)
            val_ = std::make_shared<T>(val);
        else
            *val_ = val;
    }
    
    bool isSet() const
    {
        return bool(val_);
    }
private:
    std::shared_ptr<T> val_;
};


template<class T>
class Parameter : public ParameterBase, public ParameterData<T>
{
public:
    
    bool loadFromString(const std::string& s)
    {
        YAML::Node node = YAML::Load(s);
        if(node.IsSequence())
        {
            std::cerr << " - " << getName() << ": " << "Node \"" << s << "\" is a sequence" 
                << "\nPlease pecialize your Parameter templates to support your custom types"
                << '\n';
            return false;
        }
        try
        {
            this->set( node.as<T>() );
        }
        catch(std::exception& e)
        {
            std::cerr << " - " << getName() << ": " << "Could not convert \"" << s << "\" to the type asked\n" << e.what() << '\n';
            return false;
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
};

} // namespace common
} // namespace orca
