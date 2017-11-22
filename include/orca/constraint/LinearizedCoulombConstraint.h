// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
// 
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
// 
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use, 
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info". 
// 
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability. 
// 
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or 
// data to be ensured and,  more generally, to use and operate it in the 
// same conditions as regards security. 
// 
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

#pragma once
#include <orca/constraint/GenericConstraint.h>

namespace orca
{
namespace constraint
{

class LinearizedCoulombConstraint : public GenericConstraint
{
public:
    /** Constructor
        * Build a linear function Af+b such as Af+b<=0 is a discretized cone with \a numberOfFaces conservatively
        * representing a Coulomb friction cone with friction coefficient \a frictionCoeff.
        * f is supposed to represent a 3-dimensionnal force in a local frame whose z-axis is the normal vector at the
        * contact point (so that it is the axis of the cone). b should therefore be equal to zero (which is its default
        * value, but could have its components set to another value to account for  a (possibly negative) margin.
        *
        * By default, the first edge of the cone will be in the xz plane. This can however be changed by setting the
        * static parameter ANGLE_OFFSET to a non-zero value.
        *
        * \pre f.getSize() == 3
        * \pre numberOfFaces >= 3
        * \pre frictionCoeff > 0
        */
    LinearizedCoulombConstraint();
    double getFrictionCoeff() const;
    double getMargin() const;
    void setAngleOffset(double angle_offset);
    void setFrictionCoeff(double coeff);
    void setMargin(double margin);
    void setConeOrientation(const Eigen::Matrix3d& R);
    const Eigen::Matrix3d& getConeOrientation() const;
    void setNumberOfFaces(int nfaces);
    void updateConstraintFunction();
    void resize();
private:
    double friction_coeff_ = 1;                                          ///< the friction coefficient
    double margin_ = 0;                                      ///< margin on the cone. Positive means tighter
    Eigen::Matrix3d R_cone_;     ///< orientation of the cone
    double angle_offset_ = 0;                                ///< offset for the orientation of the plane
    int number_of_faces_ = 4;
    Eigen::Vector3d v1_, v2_, n_;
};

}
}
