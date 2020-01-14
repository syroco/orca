/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl/tree.hpp>
#include <sstream>
#include <algorithm>
#include <map>
#include <stack>
#include <iostream>
#include <cassert>

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/utils.hpp>

namespace KDL {
namespace CoDyCo {
    int UndirectedTreeLink::globalIterator2localIndex(LinkMap::const_iterator link_iterator) const
    {
        int i;
        #ifndef NDEBUG
        if( !is_adjacent_to(link_iterator) )
        std::cerr << "UndirectedTreeLink::globalIterator2localIndex fatal error: " << this->getName() << " is not adjacent to " << link_iterator->getName() << std::endl;
        #endif
        assert( is_adjacent_to(link_iterator) );
        for(i=0; i < (int)getNrOfAdjacentLinks(); i++ ) {
            if( adjacent_link[i] == link_iterator ) {
                break;
            }
        }
            assert(i >= 0);
            assert(i < (int)getNrOfAdjacentLinks());
            return i;
    }

    unsigned int UndirectedTreeLink::getNrOfAdjacentLinks() const
    {
        assert(adjacent_joint.size() == adjacent_link.size());
        return adjacent_joint.size();
    }

    bool UndirectedTreeLink::is_adjacent_to(LinkMap::const_iterator link_iterator) const
    {
        for(int i=0; i < (int)getNrOfAdjacentLinks(); i++ ) {
            if( adjacent_link[i] == link_iterator ) {
                return true;
            }
        }
        return false;
    }

    Frame & UndirectedTreeLink::pose(int adjacent_index, const double& q) const
    {
        return adjacent_joint[adjacent_index]->pose(q,is_this_parent[adjacent_index]);
    }

    Twist & UndirectedTreeLink::S(int adjacent_index, const double& q) const
    {
        return adjacent_joint[adjacent_index]->S(q,is_this_parent[adjacent_index]);
    }

    Twist UndirectedTreeLink::vj(int adjacent_index, const double& q,const double& qdot) const
    {
        return adjacent_joint[adjacent_index]->vj(q,qdot,is_this_parent[adjacent_index]);
    }

    Frame & UndirectedTreeLink::pose(LinkMap::const_iterator adjacent_iterator, const double& q) const
    {
        return pose(globalIterator2localIndex(adjacent_iterator),q);
    }


    Twist & UndirectedTreeLink::S(LinkMap::const_iterator adjacent_iterator, const double& q) const
    {
        return S(globalIterator2localIndex(adjacent_iterator),q);
    }

    Twist UndirectedTreeLink::vj(LinkMap::const_iterator adjacent_iterator, const double& q,const double& qdot) const
    {
        int adjacent_index = this->globalIterator2localIndex(adjacent_iterator);
        return vj(adjacent_index,q,qdot);
    }

    JunctionMap::const_iterator UndirectedTreeLink::getAdjacentJoint(int adjacent_index) const
    {
        assert( adjacent_index >= 0 && adjacent_index < (int)adjacent_joint.size() );
        return adjacent_joint[adjacent_index];
    }

    LinkMap::const_iterator UndirectedTreeLink::getAdjacentLink(int adjacent_index) const
    {
        assert( adjacent_index >= 0 && adjacent_index < (int)adjacent_joint.size());
        if( adjacent_joint[adjacent_index]->parent->getLinkIndex() == getLinkIndex() ) {
            assert( adjacent_joint[adjacent_index]->child == adjacent_link[adjacent_index] );
            return adjacent_joint[adjacent_index]->child;
        } else {
            assert( adjacent_joint[adjacent_index]->child->getLinkIndex() == getLinkIndex() );
            assert( adjacent_joint[adjacent_index]->parent == adjacent_link[adjacent_index] );
            return adjacent_joint[adjacent_index]->parent;
        }
    }

    JunctionMap::const_iterator UndirectedTreeLink::getAdjacentJoint(LinkMap::const_iterator adjacent_iterator) const
    {
        int adjacent_index = globalIterator2localIndex(adjacent_iterator);
        return adjacent_joint[adjacent_index];
    }

    std::string UndirectedTreeLink::toString() const
    {
        std::stringstream ss;
        ss << link_name << " " << link_nr << " "  << " mass " << I.getMass() << " com " << I.getCOG();
        return ss.str();
    }


}
}//end of namespace
