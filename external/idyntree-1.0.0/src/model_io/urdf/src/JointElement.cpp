/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Author: Francesco Romano - Google LLC
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "JointElement.h"

#include "OriginElement.h"
#include "URDFParsingUtils.h"

#include <iDynTree/XMLAttribute.h>

#include <iDynTree/Model/IJoint.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/PrismaticJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>

#include <array>

namespace iDynTree {
    
    JointElement::JointElement(std::unordered_map<std::string, JointElement::JointInfo>& joints,
                 std::unordered_map<std::string, JointElement::JointInfo>& fixedJoints)
    : iDynTree::XMLElement("joint")
    , m_joints(joints)
    , m_fixedJoints(fixedJoints)
    , m_jointFrame(Transform::Identity())
    , m_axis(Axis(Direction(1.0, 0.0, 0.0), Position(0.0, 0.0, 0.0))) { }
    
    bool JointElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) {
        // attributes: name
        auto found = attributes.find("name");
        if (found != attributes.end()) {
            m_jointName = found->second->value();
        }
        found = attributes.find("type");
        if (found != attributes.end()) {
            m_jointType = found->second->value();
            // Explicitly check the supported types
            if (m_jointType != "fixed"
                && m_jointType != "revolute"
                && m_jointType != "continuous"
                && m_jointType != "prismatic")
            {
                std::string errStr = "Joint " + m_jointName + " has type " + m_jointType + " that is not currently supported by iDynTree.";
                reportError("JointElement", "setAttributes", errStr.c_str());
                return false;
            }
        }
        return true;
    }
    
    std::shared_ptr<XMLElement> JointElement::childElementForName(const std::string& name) {
        // As an alternative for simple elements, instead of creating other classes,
        // I implement the simple functions of the generic element
        if (name == "origin") {
            return std::make_shared<OriginElement>(m_jointFrame);
        } else if (name == "parent" || name == "child") {
            std::string& ref = (name == "parent") ? m_parentLink : m_childLink;
            XMLElement* element = new XMLElement(name);
            element->setAttributeCallback([&ref](const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) {
                auto linkName = attributes.find("link");
                if (linkName != attributes.end()) {
                    ref = linkName->second->value();
                }
                return true;
            });
            return std::shared_ptr<XMLElement>(element);
        } else if (name == "axis") {
            XMLElement* element = new XMLElement(name);
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) {
                auto xyz = attributes.find("xyz");
                if (xyz != attributes.end()) {
                    Vector3 direction;
                    if (vector3FromString(xyz->second->value(), direction)) {
                        m_axis = Axis(Direction(direction(0), direction(1), direction(2)),
                                      Position(0.0, 0.0, 0.0));
                    }
                }
                return true;
            });
            return std::shared_ptr<XMLElement>(element);
        } else if (name == "limit") {
            m_limits = std::make_shared<Limits>();
            m_limits->positionLower = .0;
            m_limits->positionUpper = .0;

            // TODO: check how the defaults/required works
            XMLElement* element = new XMLElement(name);
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) {

                auto found = attributes.find("lower");
                if (found != attributes.end()) {
                    double value = 0;
                    if (stringToDoubleWithClassicLocale(found->second->value(), value)) {
                        m_limits->positionLower = value;
                    }
                }
                found = attributes.find("upper");
                if (found != attributes.end()) {
                    double value = 0;
                    if (stringToDoubleWithClassicLocale(found->second->value(), value)) {
                        m_limits->positionUpper = value;
                    }
                }
                found = attributes.find("effort");
                if (found != attributes.end()) {
                    double value = 0;
                    if (stringToDoubleWithClassicLocale(found->second->value(), value)) {
                        m_limits->effort = value;
                    }
                }
                found = attributes.find("velocity");
                if (found != attributes.end()) {
                    double value = 0;
                    if (stringToDoubleWithClassicLocale(found->second->value(), value)) {
                        m_limits->velocity = value;
                    }
                }
                // TODO: check if we need to impose restrictions on the joint type

                return true;
            });
            return std::shared_ptr<XMLElement>(element);
        }
        return std::make_shared<XMLElement>(name);
    }
    
    void JointElement::exitElementScope()
    {
        JointElement::JointInfo info;
        info.joint = nullptr;
        std::unordered_map<std::string, JointElement::JointInfo>* map = nullptr;
        
        if (m_jointType == "fixed")
        {
            info.joint = std::make_shared<FixedJoint>(m_jointFrame);
            map = &m_fixedJoints;
        }
        else if (m_jointType == "revolute" || m_jointType == "continuous")
        {
            RevoluteJoint* rev_joint = new RevoluteJoint();
            rev_joint->setRestTransform(m_jointFrame);
            info.joint = std::shared_ptr<IJoint>(rev_joint);
            map = &m_joints;
        }
        else if (m_jointType == "prismatic")
        {
            PrismaticJoint* prism_joint = new PrismaticJoint();
            prism_joint->setRestTransform(m_jointFrame);
            info.joint = std::shared_ptr<IJoint>(prism_joint);
            map = &m_joints;
        }

        if (info.joint && map) {
            info.parentLinkName = m_parentLink;
            info.childLinkName = m_childLink;
            info.axis = m_axis;

            if (m_limits) {
                // Limits found
                info.joint->enablePosLimits(true);
                info.joint->setPosLimits(0, m_limits->positionLower, m_limits->positionUpper);
            } else if (m_jointType == "revolute" || m_jointType == "prismatic") {
                std::string errStr = "Joint " + m_jointName + " misses the limit tag.";
                reportWarning("JointElement", "", errStr.c_str());
            }
            
            map->insert(std::unordered_map<std::string, JointElement::JointInfo>::value_type(m_jointName, info));
        }
    }
}
