#include <memory>
#include <iostream>
#include <list>
using namespace std;

struct TaskBase {
    virtual ~TaskBase() = default; // make it polymorphism-able
    virtual void print()
    {
        cout << "TaskBase" << endl;
    }
};

struct Task : TaskBase
{
    virtual void print()
    {
        cout << "Task" << endl;
    }
};
struct Constraint : TaskBase
{
    virtual void print()
    {
        cout << "Constraint" << endl;
    }
};

struct WrenchTaskBase : TaskBase {
    void f(){}
    virtual void print()
    {
        cout << "WrenchTaskBase" << endl;
    }
};

struct Contact : Constraint,WrenchTaskBase {
    virtual void print()
    {
        cout << "Contact" << endl;
    }
};


struct AccTask : Task {
    virtual void print()
    {
        cout << "AccTask" << endl;
    }
};

int main()
{
    auto acc = std::make_shared<AccTask>();
    auto contact = std::make_shared<Contact>();

    std::list<std::shared_ptr<Constraint> > constraints;
    constraints.push_back(contact);

    for(auto c : constraints)
    {
        c->print();
        if(dynamic_pointer_cast<WrenchTaskBase>(c))
        {
            auto w = reinterpret_cast<WrenchTaskBase*>(c.get());
            cout << "Base of wrench" << endl;
            w->f(); // particular fucntion of WrenchTaskBase
            w->WrenchTaskBase::print(); // calls explicitely WrenchTaskBase function
        }
        else
        {
            cout << "NOT base of wrench" << endl;
        }
    }
    return 0;
}
