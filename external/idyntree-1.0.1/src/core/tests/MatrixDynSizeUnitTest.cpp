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
#include <iDynTree/Core/TestUtils.h>


using namespace iDynTree;

void checkCapacity()
{
    MatrixDynSize mat;

    ASSERT_EQUAL_DOUBLE(mat.rows(),0);
    ASSERT_EQUAL_DOUBLE(mat.cols(),0);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),0);

    mat.resize(10,20);

    ASSERT_EQUAL_DOUBLE(mat.rows(),10);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),10*20);

    mat.reserve(1000);

    ASSERT_EQUAL_DOUBLE(mat.rows(),10);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),1000);

    mat.reserve(2000);

    ASSERT_EQUAL_DOUBLE(mat.rows(),10);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),2000);

    mat.resize(5,20);

    ASSERT_EQUAL_DOUBLE(mat.rows(),5);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),2000);

    mat.shrink_to_fit();

    ASSERT_EQUAL_DOUBLE(mat.rows(),5);
    ASSERT_EQUAL_DOUBLE(mat.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),100);
}

void checkCopyOperator()
{
    // Create an empty matrix
    MatrixDynSize mat;

    ASSERT_EQUAL_DOUBLE(mat.rows(),0);
    ASSERT_EQUAL_DOUBLE(mat.cols(),0);
    ASSERT_EQUAL_DOUBLE(mat.capacity(),0);

    // Create a 20x20 matrix
    MatrixDynSize mat2(20,20);
    getRandomMatrix(mat2);

    ASSERT_EQUAL_DOUBLE(mat2.rows(),20);
    ASSERT_EQUAL_DOUBLE(mat2.cols(),20);
    ASSERT_EQUAL_DOUBLE(mat2.capacity(),20*20);

    // Assign the 20x20 matrix to the empty matrix
    // and check if they are qual
    mat = mat2;

    ASSERT_EQUAL_MATRIX(mat,mat2);

    // Now assign to mat a smaller 10x10 matrix, and we verify that the capacity is always 20*20
    MatrixDynSize mat3(10,10);
    getRandomMatrix(mat3);

    mat = mat3;

    ASSERT_EQUAL_MATRIX(mat,mat3);

    ASSERT_EQUAL_DOUBLE(mat.capacity(),20*20);

}

int main()
{
    checkCapacity();
    checkCopyOperator();
}
