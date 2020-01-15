/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef _KDL_CODYCO_TREEJNTTOCOMJACSOLVER_HPP_
#define _KDL_CODYCO_TREEJNTTOCOMJACSOLVER_HPP_

#ifdef __DEPRECATED
  #warning <treejnttocomjacsolver.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include "treeserialization.hpp"
#include "undirectedtree.hpp"
#include "undirectedtreesolver.hpp"
#include "crba_loops.hpp"

#include "momentumjacobian.hpp"

namespace KDL {
namespace CoDyCo {

/**
 * Class for calculating the jacobian of the center of mass
 * and the jacobian of the spatial momentum
 */
class TreeJntToCOMJacSolver: public UndirectedTreeSolver {

private:
    std::vector<RigidBodyInertia> Ic;
    MomentumJacobian h_jac;
    RigidBodyInertia I_com;

    
public:
    explicit TreeJntToCOMJacSolver(const Tree& tree, const TreeSerialization & serialization=TreeSerialization());

    virtual ~TreeJntToCOMJacSolver();

    /*
     * Calculate the floating base jacobian for the center of mass velocity, expressed with respect 
     * to the orientation of the base link and in the center of mass, to obtain the proper 3d velocity 
     * of the center of mass.
     *
     * Only the first three rows are the velocity of the center of mass, the remaing three rows
     * are the jacobian of the average angular velocity of the robot, as defined in:
     * @article{Orin2013,
     *      author = {Orin, David E. and Goswami, Ambarish and Lee, Sung-Hee},
     *      doi = {10.1007/s10514-013-9341-4},
     *      issn = {0929-5593},
     *      journal = {Autonomous Robots},
     *      title = {{Centroidal dynamics of a humanoid robot}},
     *      volume = {35},
     *      year = {2013}
     * }
     * 
     */
    int JntToCOMJac(const JntArray& q_in, Jacobian& jac);
    
    /*
     * Calculate the floating base jacobian for the center of mass velocity, expressed with respect 
     * to the orientation of the base link and in the center of mass.
     *
     * The used O(n) algorithm is based on che CRBA algorithm as explained in:
     * @article{Orin2013,
     *      author = {Orin, David E. and Goswami, Ambarish and Lee, Sung-Hee},
     *      doi = {10.1007/s10514-013-9341-4},
     *      issn = {0929-5593},
     *      journal = {Autonomous Robots},
     *      title = {{Centroidal dynamics of a humanoid robot}},
     *      volume = {35},
     *      year = {2013}
     * }
     * 
     */
    int JntToMomentumJac(const JntArray& q_in, MomentumJacobian& jac);

};

}//End of namespace
}

#endif /* TREEJNTTOJACSOLVER_H_ */
