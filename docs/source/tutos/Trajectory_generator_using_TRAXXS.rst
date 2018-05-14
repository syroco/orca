Trajectory generator: Traxxs 
--------------------------------------


TRAXXS is a framework to create and manage trajectories. See the complete information in "https://github.com/akeolab/traxxs/tree/4a2baff9064df5b3b7f288a44c2189c909ac1fed".


This tutorials not show you how to use TRAXXS for generate a trajectory, because there is many example in the web of TRAXXS. Let's see how to use TRAXXS in ORCA.


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
   #include <vector>

   #include <traxxs/tracker/tracker.hpp>
   #include <traxxs/impl/traxxs_softmotion/traxxs_softmotion.hpp> 
   #include "samples_helpers.hpp"
   #include "samples_helpers_cart.hpp"
   
   
   
   template< class T >
   using sptr = std::shared_ptr<T>;
   using namespace traxxs;
   
   int main(int argc, char** argv)
   { 
      ...
      
      // Velocity, acceleration, and jeck bounds
      path::PathBounds4d path_bounds;              
      path_bounds.dx << 0.4, 0.4, 0.4, 1.0;
      path_bounds.ddx = 100.0 * path_bounds.dx;
      path_bounds.j = 200.0 * path_bounds.ddx;
   
      std::vector< std::shared_ptr < path::PathSegment > > segments;   // Declare the  complete segment
      segments =  createMonSegments(path_bounds);                             // Create the segment via "createMonSegments"      function and apply the bounds  
    
      //Transform the path to a trajectory
      auto trajectory = std::make_shared< trajectory::Trajectory >();       
      if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
      return 1;

      trajectory::TrajectoryState state, state_new;
     
      // the status of the tracker, the tracker is used to send the next position 
      tracker::TrackerStatus status;
      auto validator = std::make_shared< tracker::TrackerSpaceValidatorPose4d >( 1e-2, 10 );
      tracker::TrackerSpacePursuit tracker( validator );
      //tracker::TrackerTimePursuit tracker;
      tracker.reset( trajectory );
      if ( !trajectory->getState( 0.0, state) ) {
      cerr << "Could not get state !\n";
      return 1;
      }
   
    for ( unsigned int i = 0; i < 20000; ++i )
    {       
   
      ...
      status = tracker.next( dt, state, state_new );
      // if the tracker fails
      if ( status == tracker::TrackerStatus::Error ) {
      std::cerr << "Tracker error.\n";
      break;
      }
      // The next position is used in cartesian acceleration task.
      ...
      Pos_des=state_new.x.segment(0,3);
      trajectoire(cart_pos_ref, 0,T,bottom,Pos_des);
      ...	
      // if the trajectory is finished  
      if ( status == tracker::TrackerStatus::Finished )
      break;
      } 
      ...
      return 0
      }
      
