Reconfigure your robot's initial state
--------------------------------------



.. code-block:: cpp


  #include <orca/orca.h>
  #include <gazebo/gazebo.hh>
  #include <gazebo/common/common.hh>
  #include <gazebo/physics/physics.hh>
  #include <gazebo/gazebo_client.hh>
  #include <sdf/parser_urdf.hh>
  #include <fstream>
  #include <thread>
  #include <iostream>
  #include <fstream>

  ...

  bool setModelConfiguration(gazebo::physics::ModelPtr gazebo_model , std::vector<std::string> joint_names,std::vector<double> joint_positions)
  {
      if (!gazebo_model)
      {
	  std::cerr  << "Model is not loaded" << std::endl;
	  return false;
      }

      if (joint_names.size() != joint_positions.size())
      {
	  std::cerr <<  "joint_names lenght should be the same as joint_positions : " << joint_names.size() << " vs " << joint_positions.size() << std::endl;
	  return false;
      }

      auto world = gazebo::physics::get_world();
      // make the service call to pause gazebo
      bool is_paused = world->IsPaused();
      if (!is_paused) world->SetPaused(true);

      std::map<std::string, double> joint_position_map;
      for (unsigned int i = 0; i < joint_names.size(); i++)
      {
	joint_position_map[joint_names[i]] = joint_positions[i];
      }

      gazebo_model->SetJointPositions(joint_position_map);

      // resume paused state before this call
      world->SetPaused(is_paused);

      return true;
  }

  int main(int argc, char** argv)
  {
    ...
    setModelConfiguration(  gz_model , { "joint_0","joint_1" ,"joint_2", "joint_3","joint_4","joint_5","joint_6"} , {1.0,0.,0.0,-1.57,0.0,1.57,0.});
    ...
    return 0

    }

The function ``SetModelConfiguration`` change robot's initial state. In main program, you specify the joint's name and the value of joint's initial position.
Here is a example who use Kuka lwr4+ with 7 joints.

.. code-block :: cpp

    setModelConfiguration(  gz_model , { "joint_0","joint_1" ,"joint_2", "joint_3","joint_4","joint_5","joint_6"} , {1.0,0.,0.0,-1.57,0.0,1.57,0.});
