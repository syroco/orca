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
using namespace orca::all;

#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>
using namespace orca::gazebo;

int main(int argc, char const *argv[])
{
    // Get the urdf file from the command line
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf (optionally -l debug/info/warning/error)" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    //  Parse logger level as --log_level (or -l) debug/warning etc
    orca::utils::Logger::parseArgv(argc, argv);

    // Create the kinematic model that is shared by everybody. Here you can pass a robot name
    auto robot_model = std::make_shared<RobotModel>();

     //  If you don't pass a robot name, it is extracted from the urdf
    if(!robot_model->loadModelFromFile(urdf_url))
        return -1;

    // All the transformations (end effector pose for example) will be expressed wrt this base frame
    robot_model->print();
    
    GazeboServer gz_server(argc,argv);
    auto gz_native_model = gz_server.insertModelFromURDFFile(urdf_url);
    
    if(!gz_native_model)
        return -1;
    
    GazeboModel gz_model(gz_native_model);

    std::cout << "Gazebo model has " << gz_model.getNDof() << " joints" << '\n';
    
    std::cout << "Gazebo is running (CTRL+C to quit), open the gui via 'gzclient'" << '\n';
    gz_server.run();
    return 0;
}