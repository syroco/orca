// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
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

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/


#include <orca/orca.h>
#include <orca/common/Factory.h>

using namespace orca::all;

int main(int argc, char const *argv[])
{
    // Get the urdf file from the command line
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " (optionally -l debug/info/warning/error)" << "\n";
        return -1;
    }
    // Cartesian Task
    auto cart_task = std::make_shared<CartesianTask>("CartTask_EE");
    // Set the frame you want to control. Here we want to control the link_7.
    cart_task->configureFromString("{ servo_controller: { type: orca::common::CartesianAccelerationPID, control_frame: link_7,"
    "pid: { type: orca::common::PIDController, dimension: 6, p_gain: [1.,1.,1.,1.,1.,1.], d_gain: [1.,1.,1.,1.,1.,1.], i_gain: [0.,0.,0.,0.,0.,0.], windup_limit: [0.,0.,0.,0.,0.,0.]  } } }"
    
    );
    cart_task->print();
    std::cout << "Task configured ? " << std::boolalpha << cart_task->isConfigured() << '\n';
    return 0;
}
