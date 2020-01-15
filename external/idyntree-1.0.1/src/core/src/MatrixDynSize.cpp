/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <sstream>
#include <cstring>

namespace iDynTree
{

MatrixDynSize::MatrixDynSize(): m_data(0), m_rows(0), m_cols(0), m_capacity(0)
{

}

MatrixDynSize::MatrixDynSize(unsigned int _rows,
                             unsigned int _cols): m_rows(_rows),
                                                  m_cols(_cols)
{
    if( this->m_rows*this->m_cols == 0 )
    {
        this->m_capacity = 0;
        this->m_data = 0;
    }
    else
    {
        this->m_capacity = this->m_rows*this->m_cols;
        this->m_data = new double[this->m_capacity];
    }

    zero();
}


MatrixDynSize::MatrixDynSize(const double* in_data,
                             const unsigned int in_rows,
                             const unsigned int in_cols): m_rows(in_rows),
                                                          m_cols(in_cols)
{
    if( this->m_rows*this->m_cols == 0 )
    {
        this->m_data = 0;
        this->m_capacity = 0;
    }
    else
    {
        this->m_capacity = this->m_rows*this->m_cols;
        this->m_data = new double[this->m_capacity];
        std::memcpy(this->m_data,in_data,in_rows*in_cols*sizeof(double));
    }
}

MatrixDynSize::MatrixDynSize(const MatrixDynSize& other)
: m_rows(other.m_rows)
, m_cols(other.m_cols)
, m_capacity(other.m_rows * other.m_cols)
{
    if (this->m_capacity == 0)
    {
        this->m_data = 0;
        return;
    }

    this->m_data = new double[this->m_capacity];
    std::memcpy(this->m_data, other.m_data, m_capacity * sizeof(double));
}

MatrixDynSize& MatrixDynSize::operator=(const MatrixDynSize& other)
{
    if (this == &other) return *this;

    // Copy the new rows and columns
    m_rows = other.m_rows;
    m_cols = other.m_cols;
    unsigned requiredCapacity = m_rows * m_cols;

    // if other is empty, return
    if (requiredCapacity == 0) return  *this;

    // If the copied data fits in the currently allocated buffer,
    // use that one (if the user want to free the memory can use
    // the shrink_to_fit method).
    // Otherwise, allocate a new buffer after deleting the old one)
    if (m_capacity < requiredCapacity) {
        // need to allocate new buffer
        // if old buffer exists, delete it
        if (m_capacity > 0) {
            delete [] m_data;
        }
        m_data = new double[requiredCapacity];
        m_capacity = requiredCapacity;
    }

    // Now we are sure that m_capacity >= m_rows*m_cols
    std::memcpy(this->m_data, other.m_data, requiredCapacity * sizeof(double));
    return *this;
}

MatrixDynSize::~MatrixDynSize()
{
    if( this->m_capacity > 0 )
    {
        delete[] this->m_data;
        this->m_data = 0;
    }
}

void MatrixDynSize::zero()
{
    for(unsigned int row=0; row < this->rows(); row++ )
    {
        for(unsigned int col=0; col < this->cols(); col++ )
        {
            this->m_data[rawIndexRowMajor(row,col)] = 0.0;
        }
    }
}

unsigned int MatrixDynSize::rows() const
{
    return this->m_rows;
}

unsigned int MatrixDynSize::cols() const
{
    return this->m_cols;
}

double* MatrixDynSize::data()
{
    return this->m_data;
}

const double* MatrixDynSize::data() const
{
    return this->m_data;
}

double& MatrixDynSize::operator()(const unsigned int row, const unsigned int col)
{
    assert(row < this->rows());
    assert(col < this->cols());
    return this->m_data[rawIndexRowMajor(row,col)];
}

double MatrixDynSize::operator()(const unsigned int row, const unsigned int col) const
{
    assert(row < this->rows());
    assert(col < this->cols());
    return this->m_data[rawIndexRowMajor(row,col)];
}

double MatrixDynSize::getVal(const unsigned int row, const unsigned int col) const
{
    if( row > this->rows() ||
        col  > this->cols() )
    {
        reportError("MatrixDynSize","getVal","indices out of bounds");
        return 0.0;
    }

    return this->m_data[rawIndexRowMajor(row,col)];
}

bool MatrixDynSize::setVal(const unsigned int row, const unsigned int col, const double new_el)
{
    if( row > this->rows() ||
        col   > this->cols() )
    {
        reportError("MatrixDynSize","setVal","indices out of bounds");
        return false;
    }

    this->m_data[rawIndexRowMajor(row,col)] = new_el;
    return true;
}

size_t MatrixDynSize::capacity() const
{
    return this->m_capacity;
}

void MatrixDynSize::reserve(const size_t _newCapacity)
{
    assert(this->m_rows*this->m_cols <= this->m_capacity);

    // If the capacity is already bigger than the requested
    // capacity, just return
    if( _newCapacity <= this->capacity() )
    {
        return;
    }

    // to match user expectation, we need to preserve
    // old data at least on a call to reserve
    changeCapacityAndCopyData(_newCapacity);
}

void MatrixDynSize::shrink_to_fit()
{
    assert(this->m_rows*this->m_cols <= this->m_capacity);

    changeCapacityAndCopyData(this->m_rows*this->m_cols);
}

void MatrixDynSize::changeCapacityAndCopyData(const unsigned int _newCapacity)
{
    // same capacity => do nothing
    if (m_capacity == _newCapacity) return;

    if (_newCapacity == 0) {
        // Simply delete all buffers
        delete [] m_data;
        m_capacity = 0;
        return;
    }

    // we have to create a new buffer and copy old data (if exists)
    double* newBuffer = new double[_newCapacity];
    m_capacity = _newCapacity;
    // zero the buffer
    std::fill(newBuffer, newBuffer + _newCapacity, 0.0);

    if (m_data) {
        // something to copy from old buffer (m_data)
        std::memcpy(newBuffer, m_data, std::min(this->m_rows * this->m_cols, _newCapacity) * sizeof(double));
    }

    delete [] m_data;
    m_data = newBuffer;
}

void MatrixDynSize::resize(const unsigned int _newRows, const unsigned int _newCols)
{
    if( (_newRows == this->rows()) &&
        (_newCols == this->cols()) )
    {
        return;
    }

    reserve(_newRows*_newCols);

    this->m_rows = _newRows;
    this->m_cols = _newCols;
}

void MatrixDynSize::fillRowMajorBuffer(double* rowMajorBuf) const
{
    // MatrixDynSize stores data in row major, a simply
    // memcpy will be sufficient
    memcpy(rowMajorBuf,this->m_data,this->rows()*this->cols()*sizeof(double));
}

void MatrixDynSize::fillColMajorBuffer(double* colMajorBuf) const
{
    for(unsigned int row = 0; row < this->rows(); row++ )
    {
        for(unsigned int col = 0; col < this->cols(); col++ )
        {
            colMajorBuf[this->rawIndexColMajor(row,col)] =
                this->m_data[this->rawIndexRowMajor(row,col)];
        }
    }
}

unsigned int MatrixDynSize::rawIndexRowMajor(int row, int col) const
{
    return (this->m_cols*row + col);
}

unsigned int MatrixDynSize::rawIndexColMajor(int row, int col) const
{
    return (row + this->m_rows*col);
}


std::string MatrixDynSize::toString() const
{
    std::stringstream ss;

    for(unsigned int row=0; row < this->rows(); row++ )
    {
        for(unsigned int col=0; col < this->cols(); col++ )
        {
            ss << this->m_data[this->rawIndexRowMajor(row,col)] << " ";
        }
        ss << std::endl;
    }

    return ss.str();
}

std::string MatrixDynSize::reservedToString() const
{
    return this->toString();
}


}
