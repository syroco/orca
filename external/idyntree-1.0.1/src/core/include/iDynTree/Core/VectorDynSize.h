/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_DYNAMIC_SIZE_VECTOR_H
#define IDYNTREE_DYNAMIC_SIZE_VECTOR_H

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
#include <iDynTree/Core/Span.h>
#endif

#include <string>

namespace iDynTree
{
    /**
     * Class providing a simple form of vector with dynamic size.
     * It is designed to provide seamless integration with SWIG.
     *
     * \ingroup iDynTreeCore
     */
    class VectorDynSize
    {
    protected:
        /**
         * Storage for the VectorDynSize
         *
         * Pointer to an area of capacity() doubles, managed by this class.
         */
        double * m_data;

        /**
         * Size of the vector.
         */
        unsigned int m_size;

        /**
         * The buffer to which m_data is pointing is m_capacity*sizeof(double).
         */
        unsigned int m_capacity;

        /**
         * Set the capacity of the vector, resizing the buffer pointed by m_data.
         */
        void changeCapacityAndCopyData(const unsigned int _newCapacity);

    public:
        /**
         * Default constructor: initialize the size of the array to zero.
         */
        VectorDynSize();

        /**
         * Constructor from the size, all the element assigned to 0
         *
         * @param _size the desired size of the array.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(unsigned int _size);

        /**
         * Constructor from a C-style array.
         *
         * Build
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(const double * in_data, const unsigned int in_size);

        /**
         * Copy constructor
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(const VectorDynSize& vec);

        /**
         * Denstructor
         *
         * \warning performs dynamic memory allocation operations
         */
        virtual ~VectorDynSize();

        /**
         * Copy assignment operator.
         *
         * The vector will be resize to match the
         * size of the argument, and the data will
         * then be copied.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize & operator=(const VectorDynSize& vec);

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /**
         * Copy assignment operator.
         *
         * The vector will be resize to match the
         * size of the argument, and the data will
         * then be copied.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize & operator=(const Span<const double>& vec);
#endif
        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to PositionRaw.
         */
        ///@{
        double operator()(const unsigned int index) const;

        double& operator()(const unsigned int index);

        double operator[](const unsigned int index) const;

        double& operator[](const unsigned int index);

        double getVal(const unsigned int index) const;

        bool setVal(const unsigned int index, const double new_el);

        unsigned int size() const;

        ///@}

        /**
         * Raw data accessor
         *
         * @return a const pointer to a vector of size() doubles
         */
        const double * data() const;

        /**
         * Raw data accessor
         *
         * @return a pointer to a vector of size() doubles
         */
        double * data();

        /**
         * Assign all element of the vector to 0.
         */
        void zero();

        /**
         * Increase the capacity of the vector preserving content.
         *
         * @param newCapacity the new capacity of the vector
         * \warning performs dynamic memory allocation operations if newCapacity > capacity()
         * \note if newCapacity <= capacity(), this method does nothing and the capacity will remain unchanged.
         */
        void reserve(const unsigned int newCapacity);

        /**
         * Change the size of the vector preserving old content.
         *
         * @param newSize the new size of the vector
         * \warning performs dynamic memory allocation operations if newSize > capacity()
         */
        void resize(const unsigned int newSize);

        /**
         * Change the capacity of the vector to match the size.
         *
         * \warning performs dynamic memory allocation operations if size() != capacity()
         */
        void shrink_to_fit();

        /**
         * The buffer pointed by data() has size capacity()*sizeof(double)
         */
        size_t capacity() const;

        /**
         * Assume that buf is pointing to
         * a buffer of size() doubles, and fill
         * it with the content of this vector.
         *
         * @param buf pointer to the buffer to fill
         *
         * @todo provide this for all matrix types
         *
         * \warning use this function only if you are
         *          an expert C user
         */
        void fillBuffer(double * buf) const;

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /** Typedefs to enable make_span.
         */
        ///@{
        typedef double value_type;

        typedef std::allocator<double> allocator_type;

        typedef typename std::allocator_traits<std::allocator<double>>::pointer pointer;

        typedef typename std::allocator_traits<std::allocator<double>>::const_pointer const_pointer;
        ///@}
#endif

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };
}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */
