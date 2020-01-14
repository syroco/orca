/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_GEOM_VECTOR_3_H
#define IDYNTREE_GEOM_VECTOR_3_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>
#include <iDynTree/Core/Utils.h>

#define GEOMVECTOR3SEMANTICS_TEMPLATE_HDR \
template <class MotionForceSemanticsT>

#define GEOMVECTOR3_TEMPLATE_HDR \
template <class MotionForceT>

namespace iDynTree
{
    class Rotation;

    /**
     * Template class providing the semantics for any geometric relation vector.
     */
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    class GeomVector3Semantics
    {
    protected:
        int body;
        int refBody;
        int coordinateFrame;

    public:
        /**
         * Constructors:
         */
        inline GeomVector3Semantics() {}
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        GeomVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        GeomVector3Semantics(const GeomVector3Semantics & other);

        void setToUnknown();


        /**
         * Getters, setters & helpers
         */
        int getBody() const;
        int getRefBody() const;
        int getCoordinateFrame() const;
        bool isUnknown() const;

        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        bool changeCoordFrame(const RotationSemantics & newCoordFrame, MotionForceSemanticsT & result) const;
        static bool compose(const MotionForceSemanticsT & op1, const MotionForceSemanticsT & op2, MotionForceSemanticsT & result);
        static bool inverse(const MotionForceSemanticsT & op, MotionForceSemanticsT & result);

        bool dot(const typename DualMotionForceSemanticsT<MotionForceSemanticsT>::Type & other) const;
    };

    /**
     * Template class providing the raw coordinates and semantics for any geometric relation vector.
     *
     * \ingroup iDynTreeCore
     *
     * A geometrical vector can be used to describe a motion or a force vector.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to motion and force vectors or to just provide an interface.
     *
     */
    GEOMVECTOR3_TEMPLATE_HDR
    class GeomVector3: public Vector3
    {
    public:
        typedef typename MotionForce_traits<MotionForceT>::SemanticsType MotionForceSemanticsT;

        MotionForceSemanticsT semantics;

        typedef GeomVector3<MotionForceT> MotionForceTbase;

        /**
         * constructors
         */
        inline GeomVector3() {}
        GeomVector3(const double* in_data, const unsigned int in_size);
        GeomVector3(const GeomVector3 & other);

        /**
         * Getters & setters
         */
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        const MotionForceSemanticsT& getSemantics() const;
        IDYNTREE_DEPRECATED_WITH_MSG("All iDynTree semantics class and  methods will be removed in iDynTree 2.0")
        void setSemantics(MotionForceSemanticsT& _semantics);

        /**
         * Geometric operations
         */
        MotionForceT changeCoordFrame(const Rotation & newCoordFrame) const;
        static MotionForceT compose(const MotionForceTbase & op1, const MotionForceT & op2);
        static MotionForceT inverse(const MotionForceTbase & op);

        /**
         * dot product
         */
        double dot(const typename MotionForce_traits<MotionForceT>::DualSpace & other) const;

        /**
         * overloaded operators
         */
        MotionForceT operator+(const MotionForceT &other) const;
        MotionForceT operator-(const MotionForceT &other) const;
        MotionForceT operator-() const;

        template <class DerivedSpatialVecT, class LinearVector3T, class AngularVector3T>
        class SpatialVector;
    };

}

#endif /* IDYNTREE_GEOM_VECTOR_3_H */
