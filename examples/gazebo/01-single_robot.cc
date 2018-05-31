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

#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

using namespace orca::gazebo;

int main(int argc, char** argv)
{
    // Get the urdf file from the command line
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    // Instanciate the gazebo server with de dedfault empty world
    // This is equivalent to GazeboServer gz("worlds/empty.world")
    GazeboServer s;
    // Insert a model onto the server and create the GazeboModel from the return value
    // You can also set the initial pose, and override the name in the URDF
    auto m = GazeboModel(s.insertModelFromURDFFile(urdf_url));

    // This is how you can get the full state of the robot
    std::cout << "Model \'" << m.getName() << "\' State :\n" << '\n';
    std::cout << "- Gravity "                   << m.getGravity().transpose()                << '\n';
    std::cout << "- Base velocity\n"            << m.getBaseVelocity().transpose()           << '\n';
    std::cout << "- Tworld->base\n"             << m.getWorldToBaseTransform().matrix()      << '\n';
    std::cout << "- Joint positions "           << m.getJointPositions().transpose()         << '\n';
    std::cout << "- Joint velocities "          << m.getJointVelocities().transpose()        << '\n';
    std::cout << "- Joint external torques "    << m.getJointExternalTorques().transpose()   << '\n';
    std::cout << "- Joint measured torques "    << m.getJointMeasuredTorques().transpose()   << '\n';

    // You can optionally register a callback that will be called
    // after every WorldUpdateEnd, so the internal gazebo model is updated
    // and you can get the full state (q,qdot,Tworld->base, etc)
    m.setCallback([&](uint32_t n_iter,double current_time,double dt)
    {
        std::cout << "[" << m.getName() << "]" << '\n'
            << "- iteration    " << n_iter << '\n'
            << "- current time " << current_time << '\n'
            << "- dt           " << dt << '\n';
        // Example : get the minimal state
        const Eigen::VectorXd& q = m.getJointPositions();
        const Eigen::VectorXd& qdot = m.getJointVelocities();

        std::cout << "ExtTrq " << m.getJointExternalTorques().transpose() << '\n';
        std::cout << "MeaTrq " << m.getJointMeasuredTorques().transpose() << '\n';
    });

    // Run the main simulation loop.
    // This is a blocking call that runs the simulation steps
    // It can be stopped by CTRL+C
    // You can optionally add a callback that happends after WorldUpdateEnd
    s.run();
    return 0;
}
