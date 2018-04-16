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

#include "orca/util/Utils.h"
#include "orca/math/Utils.h"
#include <iostream>

namespace orca
{
namespace math
{

struct Size
{
    template <typename Derived> Size(const Eigen::MatrixBase<Derived>& mat)
    : Size(mat.rows(),mat.cols())
    {}
    
    Size(int rows = 0, int cols = 0)
    : rows_(rows)
    , cols_(cols)
    {}
    Size(const Size& s)
    : Size(s.rows(),s.cols())
    {}
    bool operator==(const Size& s) const
    {
        return (s.rows() == rows_) && (s.cols() == cols_);
    }
    bool operator!=(const Size& s) const
    {
        return !(*this == s);
    }
    int rows() const { return rows_;}
    int cols() const { return cols_;}
private:
    int rows_ = 0;
    int cols_ = 0;
};


class GenericFunction
{
public:
    virtual void print() const = 0;
    virtual void resize(int rows, int cols) = 0;
    virtual Size getSize() const = 0;
    virtual int cols() const = 0;
    virtual int rows() const = 0;
};


inline std::ostream& operator<<(std::ostream& os, const Size& s)
{
    os << s.rows() << "x" << s.cols();
    return os;
}

}
}
