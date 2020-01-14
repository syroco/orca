/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_MATRIX_FIX_SIZE_H
#define IDYNTREE_MATRIX_FIX_SIZE_H


#include <iDynTree/Core/Utils.h>

#include <cassert>
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    /**
     * Class providing a simple form of matrix with dynamic size.
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int nRows, unsigned int nCols>
    class MatrixFixSize
    {
    private:
        /**
         * Return the raw index in the data vector of the
         * element corresponding to row and col, using row
         * major ordering.
         */
        unsigned int rawIndexRowMajor(int row, int col) const;

        /**
         * Return the raw index in the data vector of the
         * element corresponding to row and col, using col
         * major ordering.
         *
         * \warning The class stores data in row major order,
         *          this function is used just in the fillColMajorBuffer
         *          method.
         */
        unsigned int rawIndexColMajor(int row, int col) const;

    protected:
        /**
         * Storage for the MatrixDynSize
         *
         * Pointer to an area of size() doubles, managed by this class.
         *
         * \warning this class stores data using the row major order
         */
        double m_data[nRows*nCols];


    public:
        /**
         * Default constructor.
         * The data is not reset to 0 for perfomance reason.
         * Please initialize the data in the vector before any use.
         */
        MatrixFixSize();


        /**
         * Constructor from a C-style matrix (row major).
         *
         * \warning this class stores data using the row major order
         */
        MatrixFixSize(const double * in_data, const unsigned int in_rows, const unsigned int in_cols);

        /**
         * @name Matrix interface methods.
         * Methods exposing a matrix-like interface to MatrixFixSize.
         *
         */
        ///@{
        double operator()(const unsigned int row, const unsigned int col) const;
        double& operator()(const unsigned int row, const unsigned int col);
        double getVal(const unsigned int row, const unsigned int col) const;
        bool setVal(const unsigned int row, const unsigned int col, const double new_el);
        unsigned int rows() const;
        unsigned int cols() const;
        ///@}

        /**
         * Raw data accessor
         *
         * \warning this class stores matrix data using the row major order
         * @return a const pointer to a vector of rows()*cols() doubles
         */
        const double * data() const;

        /**
         * Raw data accessor
         *
         * \warning this class stores matrix data using the row major order
         * @return a pointer to a vector of rows()*cols() doubles
         */
        double * data();

        /**
         * Assign all element of the matrix to 0.
         */
        void zero();

        /**
         * Assume that rowMajorBuf is pointing to
         * a buffer of rows()*cols() doubles, and fill
         * it with the content of this matrix, using
         * row major order.
         *
         * @param rowMajorBuf pointer to the buffer to fill
         *
         * @todo provide this for all matrix types
         *
         * \warning use this function only if you are
         *          an expert C user
         */
        void fillRowMajorBuffer(double * rowMajorBuf) const;

        /**
         * Assume that colMajorBuf is pointing to
         * a buffer of rows()*cols() doubles, and fill
         * it with the content of this matrix, using
         * column major order.
         *
         * @param colMajorBuf pointer to the buffer to fill
         *
         * @todo provide this for all matrix types
         *
         * \warning use this function only if you are
         *          an expert C user
         */
        void fillColMajorBuffer(double * colMajorBuf) const;


        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };


    // Implementation
    template<unsigned int nRows, unsigned int nCols>
    MatrixFixSize<nRows,nCols>::MatrixFixSize()
    {
    }

    template<unsigned int nRows, unsigned int nCols>
    MatrixFixSize<nRows,nCols>::MatrixFixSize(const double* in_data,
                                              const unsigned int in_rows,
                                              const unsigned int in_cols)
    {
        if( in_rows != nRows ||
            in_cols != nCols )
        {
            reportError("MatrixFixSize","constructor","input matrix does not have the right size");
            this->zero();
        }
        else
        {
            std::memcpy(this->m_data,in_data,nRows*nCols*sizeof(double));
        }
    }

    template<unsigned int nRows, unsigned int nCols>
    void MatrixFixSize<nRows,nCols>::zero()
    {
        for(unsigned int row=0; row < this->rows(); row++ )
        {
            for(unsigned int col=0; col < this->cols(); col++ )
            {
                this->m_data[rawIndexRowMajor(row,col)] = 0.0;
            }
        }
    }

    template<unsigned int nRows, unsigned int nCols>
    unsigned int MatrixFixSize<nRows,nCols>::rows() const
    {
        return nRows;
    }

    template<unsigned int nRows, unsigned int nCols>
    unsigned int MatrixFixSize<nRows,nCols>::cols() const
    {
        return nCols;
    }

    template<unsigned int nRows, unsigned int nCols>
    double* MatrixFixSize<nRows,nCols>::data()
    {
        return this->m_data;
    }

    template<unsigned int nRows, unsigned int nCols>
    const double* MatrixFixSize<nRows,nCols>::data() const
    {
        return this->m_data;
    }

    template<unsigned int nRows, unsigned int nCols>
    double& MatrixFixSize<nRows,nCols>::operator()(const unsigned int row, const unsigned int col)
    {
        assert(row < nRows);
        assert(col < nCols);
        return this->m_data[rawIndexRowMajor(row,col)];
    }

    template<unsigned int nRows, unsigned int nCols>
    double MatrixFixSize<nRows,nCols>::operator()(const unsigned int row, const unsigned int col) const
    {
        assert(row < nRows);
        assert(col < nCols);
        return this->m_data[rawIndexRowMajor(row,col)];
    }

    template<unsigned int nRows, unsigned int nCols>
    double MatrixFixSize<nRows,nCols>::getVal(const unsigned int row, const unsigned int col) const
    {
        if( row >= this->rows() ||
            col  >= this->cols() )
        {
            reportError("MatrixDynSize","getVal","indices out of bounds");
            return 0.0;
        }

        return this->m_data[rawIndexRowMajor(row,col)];
    }

    template<unsigned int nRows, unsigned int nCols>
    bool MatrixFixSize<nRows,nCols>::setVal(const unsigned int row, const unsigned int col, const double new_el)
    {
        if( row >= this->rows() ||
            col   >= this->cols() )
        {
            reportError("MatrixDynSize","setVal","indices out of bounds");
            return false;
        }

        this->m_data[rawIndexRowMajor(row,col)] = new_el;
        return true;
    }

    template<unsigned int nRows, unsigned int nCols>
    void MatrixFixSize<nRows,nCols>::fillRowMajorBuffer(double* rowMajorBuf) const
    {
        // MatrixFixSize stores data in row major, a simply
        // memcpy will be sufficient
        memcpy(rowMajorBuf,this->m_data,this->rows()*this->cols()*sizeof(double));
    }

    template<unsigned int nRows, unsigned int nCols>
    void MatrixFixSize<nRows,nCols>::fillColMajorBuffer(double* colMajorBuf) const
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

    template<unsigned int nRows, unsigned int nCols>
    unsigned int MatrixFixSize<nRows,nCols>::rawIndexRowMajor(int row, int col) const
    {
        return (nCols*row + col);
    }

    template<unsigned int nRows, unsigned int nCols>
    unsigned int MatrixFixSize<nRows,nCols>::rawIndexColMajor(int row, int col) const
    {
        return (row + nRows*col);
    }

    template<unsigned int nRows, unsigned int nCols>
    std::string MatrixFixSize<nRows,nCols>::toString() const
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

    template<unsigned int nRows, unsigned int nCols>
    std::string MatrixFixSize<nRows,nCols>::reservedToString() const
    {
        return this->toString();
    }

    // Explicit instantiations
    // The explicit instantiations are the only ones that can be used in the API
    //  and the only ones that users are supposed to manipulate manipulate
    // Add all the explicit instantiation that can be useful, but remember to add
    // them also in the iDynTree.i SWIG file
    typedef MatrixFixSize<1,6> Matrix1x6;
    typedef MatrixFixSize<2,3> Matrix2x3;
    typedef MatrixFixSize<3,3> Matrix3x3;
    typedef MatrixFixSize<4,4> Matrix4x4;
    typedef MatrixFixSize<6,1> Matrix6x1;
    typedef MatrixFixSize<6,6> Matrix6x6 ;
    typedef MatrixFixSize<6,10> Matrix6x10;
    typedef MatrixFixSize<10,16> Matrix10x16;

}

#endif /* IDYNTREE_MATRIX_FIX_SIZE_H */
