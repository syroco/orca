/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef KDL_CODYCO_RNEA_LOOPS_HPP
#define KDL_CODYCO_RNEA_LOOPS_HPP

#ifdef __DEPRECATED
  #warning <rnea_loops.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include "undirectedtree.hpp"

namespace KDL {
namespace CoDyCo {

    /**
     * Perform the kinetic phase of the RNEA algorithm
     *
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     */
     int rneaKinematicLoop(const UndirectedTree & undirected_tree,
                           const KDL::JntArray &q,
                           const KDL::JntArray &q_dot,
                           const KDL::JntArray &q_dotdot,
                           const Traversal & kinetic_traversal,
                           const Twist& base_velocity,
                           const Twist& base_acceleration,
                                 std::vector<Twist>& v,
                                 std::vector<Twist>& a);

    /**
     * Perform the kinetic phase of the RNEA algorithm
     *
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     */
     int rneaKinematicLoop(const UndirectedTree & undirected_tree,
                           const KDL::JntArray &q,
                           const KDL::JntArray &q_dot,
                           const KDL::JntArray &q_dotdot,
                           const Traversal & kinetic_traversal,
                           const Twist& base_velocity,
                           const Twist& base_acceleration,
                                 std::vector<Twist>& v,
                                 std::vector<Twist>& a,
                                 std::vector<Wrench>& f_gi
                          );

    /**
     * Perform the dynamics phase of the RNEA algorithm, where the kinematic
     * quantites (kin_X) where calculate with a loop with a different base
     *
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     * @param[out] f is the vector (size: getNrOfLinks()) of internal wrenches.
     *            In particular, f[link_index] is the wrench that the link dynamical_traversal.getParent(link_index)
     *            is exerting on the link link_index, expressed
     *            in the link_index plucker frame.
     *
     */
    int rneaDynamicLoop(const UndirectedTree & undirected_tree,
                         const KDL::JntArray &q,
                         const Traversal & dynamical_traversal,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                         const std::vector<Wrench>& f_ext,
                         std::vector<Wrench>& f,
                         KDL::JntArray &torques,
                         Wrench & base_force);

     int rneaDynamicLoop(const UndirectedTree & undirected_tree,
                         const KDL::JntArray &q,
                         const Traversal & dynamical_traversal,
                         const std::vector<Wrench>& f_gi,
                         const std::vector<Wrench>& f_ext,
                         std::vector<Wrench>& f,
                         KDL::JntArray &torques,
                         Wrench & base_force);

}
}



#endif
