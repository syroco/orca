/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INERTIA_NON_LINEAR_PARAMETRIZATION_H
#define IDYNTREE_INERTIA_NON_LINEAR_PARAMETRIZATION_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>

namespace iDynTree
{
    class RigidBodyInertiaNonLinearParametrization
    {
    public:
        double mass;
        Position com;

        /**
         * Rotation matrix that takes a vector expressed in the centroidal
         * frame (origin in the center of mass, orientation as the principal
         * axis of the 3D inertia at the center of mass).
         */
        Rotation link_R_centroidal;

        Vector3 centralSecondMomentOfMass;

        /**
         * Get the transform that applied to a quantity expressed in the centroidal
         * frame it express it in the link frame ( link_H_centroidal ).
         */
        Transform getLinkCentroidalTransform() const;

        /**
         * Build the parametrization from a RigidBodyInertia.
         */
        void fromRigidBodyInertia(const SpatialInertia & inertia);

        /**
         * Build the nonlinear parametrization from a vector of 10 inertial parameters.
         */
        void fromInertialParameters(const Vector10 & inertialParams);

        /**
         * Convert the parametrization to a RigidBodyInertia class.
         */
        SpatialInertia toRigidBodyInertia() const;

        /**
         * Check that the mass is positive
         * and the central second moments of mass are nonnegative.
         */
        bool isPhysicallyConsistent() const;

        /**
         * Serialize the nonlinear parametrization as a
         * vector of sixteen elements.
         */
        Vector16 asVectorWithRotationAsVec() const;

        /**
         * Build the nonlinear parametrization from a vector of sixteen elements.
         */
        void fromVectorWithRotationAsVec(const Vector16 & vec);

        /**
         * Given the mapping between this nonlinear representation
         * and the classical inertia parameters. This function
         * return the gradient between the inertial parameters
         */
        Matrix10x16 getGradientWithRotationAsVec() const;
    };


}

#endif
