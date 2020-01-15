/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_UTILS_H
#define IDYNTREE_UTILS_H

#include <cstddef>

/**
 * \brief Macro to suppress unused variable warnings
 *
 * see http://stackoverflow.com/a/4851173
 */
#define IDYNTREE_UNUSED(var) ((void)var)


// Note: we set IDYNTREE_DEPRECATED to nothing when compiling idyntree-core and
// idyntree-high-level because some deprecated functions (mainly related to semantics)
// need still too be used, as they are going to be removed without any replacement
// When this functions will be removed as part of iDynTree 2.0, we can remove this special case
// Furthermore, SWIG has some problems with this attributes, so until we use a recent SWIG version
// we also disabled them for SWIG
/**
 * \brief Macro to deprecate functions and methods
 *
 * see https://blog.samat.io/2017/02/27/Deprecating-functions-and-methods-in-Cplusplus/
 */
#if defined(idyntree_core_EXPORTS) || defined(idyntree_high_level_EXPORTS) || defined(SWIG)
#define IDYNTREE_DEPRECATED
#define IDYNTREE_DEPRECATED_WITH_MSG(msg)
#else
#define IDYNTREE_DEPRECATED [[deprecated]]
#define IDYNTREE_DEPRECATED_WITH_MSG(msg) [[deprecated(msg)]]
#endif

namespace iDynTree
{
    extern int UNKNOWN;

    /// Default tolerance for methods with a tolerance, setted to 1e-10
    extern double DEFAULT_TOL;

    /**
     * Function embedding the semantic checks
     *
     * This function can throw an exception if the semantic check detects an error (returns False).
     */
    void assertWoAbort(const char * semCheck, const char * file, const char* func, int line);

    /**
     * Helper class for semantic checking.
     *
     * Return true if two values are equal, or if one of the two is unknown
     * All negative values are used for represent an unknown value.
     */
    bool checkEqualOrUnknown(const int op1, const int op2);

    /**
     * Helper function for reporting error if the semantic check fails.
     *
     */
    void reportError(const char * className, const char* methodName, const char * errorMessage);

    /**
     * Call report error if condition is true
     */
    bool reportErrorIf(bool condition, const char * className_methodName, const char * errorMessage);

    /**
     * Helper function for reporting warnings in iDynTree
     *
     */
    void reportWarning(const char * className, const char* methodName, const char * errorMessage);

    /**
     * Helper function for reporting information messages in iDynTree
     *
     */
    void reportInfo(const char* className, const char* methodName, const char* message);
    
    /**
     * Helper function for reporting debug messages in iDynTree
     *
     */
    void reportDebug(const char* className, const char* methodName, const char* message);
    
    /**
     * Convert a double from degrees to radians.
     */
    double deg2rad(const double valueInDeg);

    /**
     * Convert a double from radians to degree.
     */
    double rad2deg(const double valueInRad);

    /**
     * Simple structure describing a range of rows or columns in a vector or a matrix.
     * The offset attributes indicates the index of the first element of the range, while
     * the size indicates the size of the range.
     *
     * Example: offset = 2, size = 3 measn the elements 2 3 4 .
     */
    struct IndexRange
    {
        std::ptrdiff_t offset;
        std::ptrdiff_t size;

        bool isValid() const;
        static IndexRange InvalidRange();
    };

    /**
     * Enum describing the possible matrix storage ordering
     */
    enum MatrixStorageOrdering {
        RowMajor, /*!< Row Major ordering, i.e. matrix is serialized row by row */
        ColumnMajor /*!< Column Major ordering, i.e. matrix is serialized row by column */
    };

    /**
     * Check whether two doubles are equal given a tolerance tol.
     */
    bool checkDoublesAreEqual(const double & val1, const double & val2, double tol = DEFAULT_TOL);

}

#endif
