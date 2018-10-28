#include "orca/optim/ControlVariableMapping.h"

namespace orca
{
namespace optim
{
    
unsigned int ControlVariableMapping::generate(unsigned int ndof, unsigned int nwrenches)
{
    const int fulldim = (floating_base_ ? 6 : 0) + ndof;

    size_map_[    ControlVariable::X                          ] = 2 * fulldim + nwrenches * 6;
    size_map_[    ControlVariable::GeneralisedAcceleration    ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseAcceleration   ] = (floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointAcceleration     ] = ndof;
    size_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseWrench         ] = (floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointTorque           ] = ndof;
    size_map_[    ControlVariable::ExternalWrench             ] = 6;
    size_map_[    ControlVariable::ExternalWrenches           ] = nwrenches * 6;
    size_map_[    ControlVariable::Composite                  ] = 0;
    size_map_[    ControlVariable::None                       ] = 0;

    index_map_[    ControlVariable::X                          ] = 0;
    index_map_[    ControlVariable::GeneralisedAcceleration    ] = 0;
    index_map_[    ControlVariable::FloatingBaseAcceleration   ] = 0;
    index_map_[    ControlVariable::JointAcceleration     ] = (floating_base_ ? 6 : 0);
    index_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    index_map_[    ControlVariable::FloatingBaseWrench         ] = fulldim;
    index_map_[    ControlVariable::JointTorque           ] = fulldim + (floating_base_ ? 6 : 0);
    index_map_[    ControlVariable::ExternalWrench             ] = 2 * fulldim;
    index_map_[    ControlVariable::ExternalWrenches           ] = 2 * fulldim;
    index_map_[    ControlVariable::Composite                  ] = 0;
    index_map_[    ControlVariable::None                       ] = 0;

    return this->getSize(ControlVariable::X);
}

const std::map<ControlVariable, unsigned int >& ControlVariableMapping::getIndexMap() const
{
    return index_map_;
}
const std::map<ControlVariable, unsigned int >& ControlVariableMapping::getSizeMap() const
{
    return size_map_;
}

unsigned int ControlVariableMapping::getIndex(ControlVariable var) const
{
    return index_map_.at(var);
}
unsigned int ControlVariableMapping::getSize(ControlVariable var) const
{
    return size_map_.at(var);
}
unsigned int ControlVariableMapping::getTotalSize() const
{
    return this->getSize(ControlVariable::X);
}

} // namespace optim
} // namespace orca
