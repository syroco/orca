#pragma once
#include <orca/common/ParameterBase.h>
#include <orca/common/ParameterData.h>
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
        for (auto row=0; row<matrix.rows(); row++)
          for (auto col=0; col<matrix.cols(); col++)
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
        for (auto id=0; id<node.size(); id++)
          (node[0].size() == 0 ? matrix(id) = node[id].as<_Scalar>() :  matrix(id) = node[id][0].as<_Scalar>());
        return true;
      }

      return true;
    }
  };

} // namespace YAML

// Special case for shared_ptr
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

#include <orca/optim/QPSolverImplType.h>
namespace YAML {
  template<>
  struct convert< orca::optim::QPSolverImplType >
  {
    static Node encode(const orca::optim::QPSolverImplType& s)
    {
      auto node = YAML::Node();
      node.push_back(orca::optim::QPSolverImplTypeToString(s));
      return node;
    }

    static bool decode(const Node& node, orca::optim::QPSolverImplType& e)
    {
      e = orca::optim::QPSolverImplTypeFromString(node[0].as<std::string>());
      return true;
    }
  };
}

#include <orca/optim/ResolutionStrategy.h>
namespace YAML {
  template<>
  struct convert< orca::optim::ResolutionStrategy >
  {
    static Node encode(const orca::optim::ResolutionStrategy& s)
    {
      auto node = YAML::Node();
      node.push_back(orca::optim::ResolutionStrategyToString(s));
      return node;
    }

    static bool decode(const Node& node, orca::optim::ResolutionStrategy& e)
    {
      e = orca::optim::ResolutionStrategyFromString(node[0].as<std::string>());
      return true;
    }
  };
}


namespace orca
{
namespace common
{

/**
* @brief This class holds the conversion from a string (YAML string) to the data type
* 
*/
template<class T>
class Parameter : public ParameterBase, public ParameterData<T>
{
public:
    Parameter()
    {}
    template<class T2>
    Parameter(const T2& t)
    {
        ParameterData<T>::set(t);
    }
    bool onLoadFromString(const std::string& s)
    {
        YAML::Node node = YAML::Load(s);
        try
        {
            ParameterData<T>::set( node.as<T>() );
        }
        catch(std::exception& e)
        {
            utils::orca_throw(utils::Formatter() << "parameter '" << getName() << "': " 
                << "Could not convert \"" << s 
                << "\" to the type asked\n" << e.what());
        }
        return true;
    }

    void print() const
    {
        std::cout << "Parameter '" << getName() << "': " << ParameterData<T>::get() << '\n';
    }
    
    bool isSet() const
    {
        return ParameterData<T>::isSet();
    }
    
    T& get()
    {
        try {
            return ParameterData<T>::get();
        } catch (std::exception& e) {
            utils::orca_throw(utils::Formatter() << "Parameter '" << getName() << "' : " << e.what());
        }
        return __fix_warnings__;
    }
    const T& get() const
    {
        try {
            return ParameterData<T>::get();
        } catch (std::exception& e) {
            utils::orca_throw(utils::Formatter() << "Parameter '" << getName() << "' : " << e.what());
        }
        return __fix_warnings__;
    }
    
    template<class T2>
    Parameter<T>& operator=(T2 val)
    {
        this->set(val);
        return *this;
    }
private:
    T __fix_warnings__;
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

template<class T>
class Parameter<std::list<T > > : public ParameterBase, public ParameterData< std::list<T > >
{
public:   
    Parameter() {}
    virtual ~Parameter() {}
    Parameter(const std::list<T >& t)
    {
        ParameterData< std::list<T > >::set(t);
    }
    bool onLoadFromString(const std::string& s)
    {
        YAML::Node node = YAML::Load(s);
        std::list<T > l;
        for(auto n : node)
        {
            auto task_base_name = n.first.as<std::string>();
            Parameter<T> p;
            p.setName(task_base_name);
            p.setRequired(true);
            YAML::Emitter out; 
            out << n.second;
            if(p.loadFromString(out.c_str()))
                l.push_back( p.get() );
        }
        ParameterData< std::list<T > >::set(l);
        return true;
    }

    void print() const
    {
        std::cout << "Parameter '" << getName() << "' : list of " << typeid(T).name() << '\n';
        for(auto p : this->get())
        {
            p->print();
        }
    }
    
    bool isSet() const
    {
        return ParameterData<std::list<T > >::isSet();
    }
    
    std::list<T >& get()
    {
        try {
            return ParameterData< std::list<T > >::get();
        } catch (std::exception& e) {
            utils::orca_throw(utils::Formatter() << "Parameter '" << getName() << "' : " << e.what());
        }
        return __fix_warnings__;
    }
    const std::list<T >& get() const
    {
        try {
            return ParameterData< std::list<T > >::get();
        } catch (std::exception& e) {
            utils::orca_throw(utils::Formatter() << "Parameter '" << getName() << "' : " << e.what());
        }
        return __fix_warnings__;
    }
    
    template<class T2>
    Parameter<std::list<T > >& operator=(std::list<std::shared_ptr<T2> > val)
    {
        this->set(val);
        return *this;
    }
private:
  std::list<T> __fix_warnings__;
};

} // namespace common
} // namespace orca

template<class T>
::std::ostream& operator<<(::std::ostream& os, const orca::common::Parameter<T>& p)
{
    return os << p.get();
}
