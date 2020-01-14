/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/TransformSemantics.h>
#include <iDynTree/Core/RotationSemantics.h>
#include <iDynTree/Core/PositionSemantics.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/PrivatePreProcessorUtils.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>


#include <iostream>
#include <sstream>

#include <cassert>

namespace iDynTree
{

    TransformSemantics::TransformSemantics(PositionSemantics & position, RotationSemantics & rotation): positionSemantics(position),
                                                                                                        rotationSemantics(rotation)
    {

    }

    const PositionSemantics & TransformSemantics::getPositionSemantics() const
    {
        return this->positionSemantics;
    }

    const RotationSemantics & TransformSemantics::getRotationSemantics() const
    {
        return this->rotationSemantics;
    }

    bool TransformSemantics::check_position2rotationConsistency(const PositionSemantics& position, const RotationSemantics& rotation)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(position.getCoordinateFrame(), rotation.getCoordinateFrame()),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "position and rotation expressed in a different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(position.getPoint(), rotation.getOrientationFrame()),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "position point is different from the orientation frame origin\n")
                && reportErrorIf(!checkEqualOrUnknown(position.getReferencePoint(), rotation.getReferenceOrientationFrame()),
                                 IDYNTREE_PRETTY_FUNCTION,
                                 "position Ref point is different from the Ref orientation frame origin\n"));
    }

    bool TransformSemantics::setPositionSemantics(const PositionSemantics& /*position*/)
    {
        bool status = true;

        // check consistency of setted position with existing rotation
        iDynTreeSemanticsOp(status = this->check_position2rotationConsistency(position, this->getRotationSemantics()));

        // set semantics
        iDynTreeSemanticsOp(this->positionSemantics = position);

        return status;
    }

    bool TransformSemantics::setRotationSemantics(const RotationSemantics& /*rotation*/)
    {
        bool status = true;

        // check consistency of setted position with existing rotation
        iDynTreeSemanticsOp(status = this->check_position2rotationConsistency(this->getPositionSemantics(), rotation));

        // set semantics
        iDynTreeSemanticsOp(this->rotationSemantics = rotation);

        return status;
    }

    TransformSemantics & TransformSemantics::operator= (const TransformSemantics & /*other*/)
    {
        return *this;
    }

    std::string TransformSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
        << " point " << this->getPositionSemantics().getPoint()
        << " orientationFrame " << this->getRotationSemantics().getOrientationFrame()
        << " referencePoint " << this->getPositionSemantics().getReferencePoint()
        << " referenceOrientationFrame " << this->getRotationSemantics().getReferenceOrientationFrame();

        return ss.str();
    }

    std::string TransformSemantics::reservedToString() const
    {
        return this->toString();
    }



}
