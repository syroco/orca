// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

#pragma once
#include "orca/constraint/Contact.h"
#include "orca/constraint/EqualityConstraint.h"
#include "orca/common/Wrench.h"

namespace orca
{
namespace constraint
{

class DynamicsEquationConstraint : public EqualityConstraint
{
public:
    DynamicsEquationConstraint(const std::string& name);

protected:
    void onActivation(){}
    void onUpdateConstraintFunction(double current_time, double dt);
    void onDeactivation(){}
    void onResize();
private:
    std::list< std::shared_ptr< const common::Wrench> > wrenches_;
    std::map<optim::ControlVariable, unsigned int > idx_map_;
    std::map<optim::ControlVariable, unsigned int > size_map_;
    int ndof_ = 0,fulldim_ = 0;

};

}
}
