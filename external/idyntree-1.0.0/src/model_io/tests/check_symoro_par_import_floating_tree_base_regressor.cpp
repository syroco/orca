/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

/* Author: Silvio Traversaro */

#include <iDynTree/ModelIO/symoro_par_import.hpp>
#include <iDynTree/ModelIO/symoro_par_import_serialization.hpp>

#include <kdl_codyco/treeinertialparameters.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_codyco/treefksolverpos_iterative.hpp>

#include <kdl/treefksolverpos_recursive.hpp>

#include <ctime>

#include <kdl/frames_io.hpp>

using namespace KDL;
using namespace std;
using namespace iDynTree;
using namespace KDL::CoDyCo;

void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
  cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has " << GetTreeElementChildren(link->second).size() << " children" << endl;
  //cout << " frame_to_tip: " << GetTreeElementSegment(link->second)..getFrameToTip() << endl;
  for (unsigned int i=0; i<GetTreeElementChildren(link->second).size(); i++)
    printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
}



double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

#include "symoro_generated_HRP2JRL_regressor.cpp"

int main(int argc, char** argv)
{
  srand(time(NULL));
  if (argc < 2){
    std::cerr << "Expect .par file to parse" << std::endl;
    return -1;
  }

  symoro_par_model mdl;

  if( !parModelFromFile(argv[1],mdl) ) {cerr << "Could not parse SyMoRo par robot model" << endl; return EXIT_FAILURE;}

  std::cout << "Extracted par file" << std::endl;
  std::cout << mdl.toString() << std::endl;


  Tree my_tree;
  TreeSerialization serialization;
  if (!treeFromSymoroParFile(argv[1],my_tree,true))
  {cerr << "Could not generate robot model and extract kdl tree" << endl; return EXIT_FAILURE;}
  if (!treeSerializationFromSymoroParFile(argv[1],serialization,true))
  {cerr << "Could not generate robot model and extract kdl_codyco serialization" << endl; return EXIT_FAILURE;}


  // walk through tree
  cout << " ======================================" << endl;
  cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
  cout << " ======================================" << endl;
  SegmentMap::const_iterator root = my_tree.getRootSegment();
  printLink(root, "");

  //Solving regressor in both mode

  JntArray q,dq,ddq,torques,torques_converted;
  std::vector<Wrench> f,f_ext;
  Wrench base_force, base_force_converted;
  Twist base_vel, base_acc;

  q = dq = ddq = torques = torques_converted = JntArray(my_tree.getNrOfJoints());
  f = f_ext = std::vector<Wrench>(my_tree.getNrOfSegments(),KDL::Wrench::Zero());

  double q_range = 1;
  double dq_range = 1;
  double ddq_range = 1;
  double base_vel_range = 1;
  double base_acc_range = 1;

  for(int i=0; i < my_tree.getNrOfJoints(); i++ )
  {
        q(i) = q_range*random_double();
        dq(i) = dq_range*random_double();
        ddq(i) = ddq_range*random_double();
  }

  //base_vel = base_vel_range*Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
  base_vel = Twist(Vector(1.0,0.0,0.0),Vector(0.0,0.0,1.0));
  base_acc = base_acc_range*Twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));


      double tol = 1e-3;

  /*
  //First we check the geometry
  KDL::Frame H_kdl, H_kdl_rec, H_symoro;
  KDL::CoDyCo::TreeFkSolverPos_iterative pos_slv(my_tree);
  if( pos_slv.JntToCart(q,H_kdl,"Link6") != 0 ) { std::cout << "Failed geom solver " << std::endl; return EXIT_FAILURE; }
  symoro_generated_ee_transform(q,H_symoro);


  std::cout << "H_kdl    " << H_kdl << std::endl;
  std::cout << "H_symoro " << H_symoro << std::endl;
  std::cout << (H_kdl.p-H_symoro.p).Norm() << std::endl;


  if( (H_kdl.p-H_symoro.p).Norm() > tol ) { std::cout << "Failed geometry check " << std::endl; }
  */

  TreeInertialParametersRegressor slv(my_tree,Vector::Zero(),serialization);

  int np = (my_tree.getNrOfSegments()+1)*10;

  Eigen::MatrixXd regr_dirl, regr_symoro;
  regr_dirl.resize(6,np);
  regr_symoro.resize(6,np);

  regr_dirl.setZero();

  Eigen::MatrixXd fb_regr_dirl;
  fb_regr_dirl.resize(my_tree.getNrOfJoints()+6,np);

  fb_regr_dirl.setZero();

  //Calculate regressor in both the ways
  regr_symoro.setZero();
  regr_dirl.setZero();

  if( slv.dynamicsRegressor(q,dq,ddq,base_vel,base_acc,fb_regr_dirl) != 0 ) { cout << "fb dirl regressor failed with code " << slv.dynamicsRegressor(q,dq,ddq,base_vel,base_acc,fb_regr_dirl) << " " << endl; return EXIT_FAILURE; }
  if( symoro_generated_HRP2JRL_IMU_regressor(q,dq,ddq,base_vel,base_acc,regr_symoro) != 0 ) { cout << "SyMoRo regressor failed " << endl; return EXIT_FAILURE; }

  std::cout << "Serialization used " << std::endl;
  std::cout << slv.getUndirectedTree().getSerialization().toString() << std::endl;

  regr_dirl = fb_regr_dirl.block(0,0,6,fb_regr_dirl.cols());

  //Symoro doesn't generate the regressor for the base link
  regr_dirl.block(0,0,6,10).setZero();


  for(int i=0; i < regr_dirl.rows(); i++ ) {
      for(int j=0; j < regr_dirl.cols(); j++ ) {
          double err;
          double range = fabs(q_range+dq_range+ddq_range+base_acc_range+base_vel_range);
          if( range > 1 ) {
            err = fabs(regr_dirl(i,j)-regr_symoro(i,j))/range;
          } else {
            err = fabs(regr_dirl(i,j)-regr_symoro(i,j));
          }
          std::cout << "Relative error element " << i << " " << j << " is " << err << " abs(" << regr_dirl(i,j) << " - " << regr_symoro(i,j) << " )"<< std::endl;
          if( err > tol ) return EXIT_FAILURE;
      }
  }

  return EXIT_SUCCESS;
}


