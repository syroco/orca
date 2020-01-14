/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Model.h>

#include <iDynTree/Core/ArticulatedBodyInertia.h>

namespace iDynTree
{

LinkPositions::LinkPositions(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

LinkPositions::LinkPositions(const Model& model)
{
    resize(model);
}

void LinkPositions::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkPositions::resize(unsigned int nrOfLinks)
{
    Transform identityTransform = Transform::Identity();
    this->m_linkPos.resize(nrOfLinks);

    for(size_t link=0; link < nrOfLinks; link++)
    {
        this->m_linkPos[link] = identityTransform;
    }
}

bool LinkPositions::isConsistent(const Model& model) const
{
    return (this->m_linkPos.size() == model.getNrOfLinks());
}

size_t LinkPositions::getNrOfLinks() const
{
    return this->m_linkPos.size();
}

const Transform& LinkPositions::operator()(const LinkIndex link) const
{
    return this->m_linkPos[link];
}

Transform& LinkPositions::operator()(const LinkIndex link)
{
    return this->m_linkPos[link];
}

std::string LinkPositions::toString(const Model& model) const
{
   std::stringstream ss;

    size_t nrOfLinks = this->getNrOfLinks();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        ss << "Position for link " << model.getLinkName(l) << ":" << this->operator()(l).toString() << std::endl;
    }
    return ss.str();
}


LinkPositions::~LinkPositions()
{
    resize(0);
}


LinkWrenches::LinkWrenches(const Model& model)
{
    resize(model);
}

LinkWrenches::LinkWrenches(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

void LinkWrenches::resize(const Model& model)
{
    this->resize(model.getNrOfLinks());
}

void LinkWrenches::resize(unsigned int nrOfLinks)
{
    iDynTree::Wrench zeroWrench = iDynTree::Wrench::Zero();
    this->m_linkWrenches.resize(nrOfLinks,zeroWrench);
}

bool LinkWrenches::isConsistent(const Model& model) const
{
    return (model.getNrOfLinks() == m_linkWrenches.size());
}

size_t LinkWrenches::getNrOfLinks() const
{
    return m_linkWrenches.size();
}

Wrench& LinkWrenches::operator()(const LinkIndex link)
{
    return this->m_linkWrenches[link];
}

const Wrench& LinkWrenches::operator()(const LinkIndex link) const
{
    return this->m_linkWrenches[link];
}

std::string LinkWrenches::toString(const Model& model) const
{
   std::stringstream ss;

    size_t nrOfLinks = this->getNrOfLinks();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        ss << "Wrench for link " << model.getLinkName(l) << ":" << this->operator()(l).toString() << std::endl;
    }
    return ss.str();
}

void LinkWrenches::zero()
{
    size_t nrOfLinks = this->getNrOfLinks();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        this->operator()(l).zero();
    }
}


LinkWrenches::~LinkWrenches()
{
    resize(0);
}

LinkInertias::LinkInertias(unsigned int nrOfLinks)
{
    this->resize(nrOfLinks);
}

LinkInertias::LinkInertias(const Model& model)
{
    this->resize(model);
}

void LinkInertias::resize(const Model& model)
{
    this->resize(model.getNrOfLinks());
}

void LinkInertias::resize(unsigned int nrOfLinks)
{
    /**
     * We reset the vector elements to be zero because the resize is
     * already an expensive operation that should not
     * be performed in efficient loop.
     */
    iDynTree::SpatialInertia zeroInertia;
    zeroInertia.zero();
    this->m_linkInertials.resize(nrOfLinks,zeroInertia);
}

bool LinkInertias::isConsistent(const Model& model) const
{
    return (this->m_linkInertials.size() == model.getNrOfLinks());
}

SpatialInertia& LinkInertias::operator()(const LinkIndex link)
{
    return this->m_linkInertials[(size_t)link];
}

const SpatialInertia& LinkInertias::operator()(const LinkIndex link) const
{
    return this->m_linkInertials[(size_t)link];
}


LinkInertias::~LinkInertias()
{

}

LinkVelArray::LinkVelArray(const Model& model)
{
    resize(model);
}

LinkVelArray::LinkVelArray(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

const Twist& LinkVelArray::operator()(const LinkIndex link) const
{
    return this->m_linkTwist[link];
}

Twist& LinkVelArray::operator()(const LinkIndex link)
{
    return this->m_linkTwist[link];
}

void LinkVelArray::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkVelArray::resize(unsigned int nrOfLinks)
{
    iDynTree::Twist zeroTwist = iDynTree::Twist::Zero();
    this->m_linkTwist.resize(nrOfLinks,zeroTwist);
}

bool LinkVelArray::isConsistent(const Model& model) const
{
    return (this->m_linkTwist.size() == model.getNrOfLinks());
}

size_t LinkVelArray::getNrOfLinks() const
{
    return this->m_linkTwist.size();
}

std::string LinkVelArray::toString(const Model& model) const
{
    std::stringstream ss;

    size_t nrOfLinks = this->getNrOfLinks();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        ss << "Twist for link " << model.getLinkName(l) << ":" << this->operator()(l).toString() << std::endl;
    }
    return ss.str();
}


LinkVelArray::~LinkVelArray()
{

}

LinkAccArray::LinkAccArray(const Model& model)
{
    resize(model);
}

LinkAccArray::LinkAccArray(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

const SpatialAcc& LinkAccArray::operator()(const LinkIndex link) const
{
    return this->m_linkAcc[link];
}

SpatialAcc& LinkAccArray::operator()(const LinkIndex link)
{
    return this->m_linkAcc[link];
}

void LinkAccArray::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkAccArray::resize(unsigned int nrOfLinks)
{
    iDynTree::SpatialAcc zeroAcc = iDynTree::SpatialAcc::Zero();
    this->m_linkAcc.resize(nrOfLinks,zeroAcc);
}

bool LinkAccArray::isConsistent(const Model& model) const
{
    return (this->m_linkAcc.size() == model.getNrOfLinks());
}

unsigned int LinkAccArray::getNrOfLinks() const
{
    return this->m_linkAcc.size();
}

std::string LinkAccArray::toString(const Model& model) const
{
    std::stringstream ss;

    size_t nrOfLinks = this->getNrOfLinks();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        ss << "Acceleration for link " << model.getLinkName(l) << ":" << this->operator()(l).toString() << std::endl;
    }
    return ss.str();
}

LinkAccArray::~LinkAccArray()
{

}

LinkArticulatedBodyInertias::LinkArticulatedBodyInertias(const Model& model)
{
    resize(model);
}

LinkArticulatedBodyInertias::LinkArticulatedBodyInertias(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

const ArticulatedBodyInertia& LinkArticulatedBodyInertias::operator()(const LinkIndex link) const
{
    return this->m_linkABIs[link];
}

ArticulatedBodyInertia& LinkArticulatedBodyInertias::operator()(const LinkIndex link)
{
    return this->m_linkABIs[link];
}

void LinkArticulatedBodyInertias::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkArticulatedBodyInertias::resize(unsigned int nrOfLinks)
{
    ArticulatedBodyInertia abi;
    abi.zero();
    this->m_linkABIs.resize(nrOfLinks,abi);
}

bool LinkArticulatedBodyInertias::isConsistent(const Model& model) const
{
    return (this->m_linkABIs.size() == model.getNrOfLinks());
}


LinkArticulatedBodyInertias::~LinkArticulatedBodyInertias()
{

}










}
