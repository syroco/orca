#pragma once
#include <orca/utils/Utils.h>
#include <orca/common/Parameter.h>
#include <orca/common/Factory.h>
#include <typeinfo>

namespace orca
{
namespace common
{

inline std::string findType(const YAML::Node& n)
{
    for(auto c : n)
    {
        try{
            if(c.first.as<std::string>() == "type")
                return c.second.as<std::string>();
        }catch(...){}
    }
    return std::string();
}

template<class T>
class Parameter<std::shared_ptr<T> > : public ParameterBase, public ParameterData< std::shared_ptr<T> >
{
public:    

    bool onLoadFromString(const std::string& s)
    {
        YAML::Node node = YAML::Load(s);
        auto type_name = findType(node);
        if(type_name.empty())
        {
            utils::orca_throw(utils::Formatter() << "Could not find \"type\" in the yaml file for param " 
            << getName() << " of type " << typeid(T).name() << "::Ptr"
            << "\nConfig provided : \n" << s );
        }
        
        std::cout << "Param " << getName() << " is of type " << type_name;
        auto task_base = Factory::Instance()->createPtr(getName(),type_name);
        
        if(!task_base)
        {
            utils::orca_throw(utils::Formatter() << "Param " << getName() << " Could not create a class of type " << type_name << "" );
        }
        
        std::cout << "Param " << getName() << " created class of type " << type_name << ", now configuring it if it has params itself."<< '\n';
        
        if(!task_base->configureFromString(s))
        {
            return false;
        }
        this->set( std::dynamic_pointer_cast<T>( task_base ));
        return true;
    }

    void print() const
    {
        std::cout << getName() << " [ " << typeid(T).name() << "::Ptr" << '\n';
    }
    
    bool isSet() const
    {
        return ParameterData<std::shared_ptr<T> >::isSet();
    }
    
    template<class T2>
    Parameter<std::shared_ptr<T> >& operator=(std::shared_ptr<T2> val)
    {
        this->set(val);
        return *this;
    }
};

template<class T>
class Parameter<std::list<std::shared_ptr<T> > > : public ParameterBase, public ParameterData< std::list<std::shared_ptr<T> > >
{
public:    
    
    bool onLoadFromString(const std::string& s)
    {
        std::cout << getName() << " Analysing list" << '\n';
        YAML::Node node = YAML::Load(s);
        for(auto n : node)
        {
            auto task_base_name = n.first.as<std::string>();
            std::cout << " -  " << task_base_name << '\n';
            Parameter<std::shared_ptr<T> > p;
            p.setName(task_base_name);
            p.setRequired(true);
            YAML::Emitter out; 
            out << n.second;
            if(p.loadFromString(out.c_str()))
            {
                if(!utils::exists(p.get(),ParameterData< std::list<std::shared_ptr<T> > >::get()))
                {
                    ParameterData< std::list<std::shared_ptr<T> > >::get().push_back( p.get() );
                }
            }
        }
        return true;
    }

    void print() const
    {
        std::cout << getName() << " list of shared_ptr" << '\n';
    }
    
    bool isSet() const
    {
        return ParameterData<std::list<std::shared_ptr<T> > >::isSet();
    }
    
    template<class T2>
    Parameter<std::list<std::shared_ptr<T> > >& operator=(std::list<std::shared_ptr<T2> > val)
    {
        this->set(val);
        return *this;
    }
};

} // namespace common
} // namespace orca
