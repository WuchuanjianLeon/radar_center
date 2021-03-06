/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "stm32f10x.h"
#include <math.h>

#include "SRCKF.h"
#include "FastMath.h"
#include "Quaternion.h"

//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
#define SRCKF_PQ_INITIAL 0.000001f
#define SRCKF_PW_INITIAL 0.000001f

#define SRCKF_QQ_INITIAL 0.0000045f
#define SRCKF_QW_INITIAL 0.0000025f

#define SRCKF_RA_INITIAL 0.07f
#define SRCKF_RM_INITIAL 0.105f
//////////////////////////////////////////////////////////////////////////
//
arm_status arm_sqrt_f32(
	float32_t in,
	float32_t * pOut)
{
// if(in >= 0.0f)
// {
	*pOut = sqrt(in);
	return (ARM_MATH_SUCCESS);
	//}
// else
// {
//	*pOut = 0.0f;
//	return (ARM_MATH_ARGUMENT_ERROR);
// }

}

arm_status arm_mat_mult_f32(
	const arm_matrix_instance_f32 * pSrcA,
	const arm_matrix_instance_f32 * pSrcB,
	arm_matrix_instance_f32 * pDst)
{
	float32_t *pIn1 = pSrcA->pData;				 /* input data matrix pointer A */
	float32_t *pIn2 = pSrcB->pData;				 /* input data matrix pointer B */
	float32_t *pInA = pSrcA->pData;				 /* input data matrix pointer A  */
	//	float32_t *pSrcB = pSrcB->pData;				/* input data matrix pointer B */
	float32_t *pOut = pDst->pData;				 /* output data matrix pointer */
	float32_t *px;								 /* Temporary output data matrix pointer */
	float32_t sum;								 /* Accumulator */
	uint16_t numRowsA = pSrcA->numRows;			 /* number of rows of input matrix A */
	uint16_t numColsB = pSrcB->numCols;			 /* number of columns of input matrix B */
	uint16_t numColsA = pSrcA->numCols;			 /* number of columns of input matrix A */
	uint16_t col, i = 0u, j, row = numRowsA, colCnt;		/* loop counters */
	arm_status status;							 /* status of matrix multiplication */

	/* Check for matrix mismatch condition */
	if((pSrcA->numCols != pSrcB->numRows) ||
			(pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
	{

		/* Set status as ARM_MATH_SIZE_MISMATCH */
		status = ARM_MATH_SIZE_MISMATCH;
	}
	else
	{
		/* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
		/* row loop */
		do
		{
			/* Output pointer is set to starting address of the row being processed */
			px = pOut + i;

			/* For every row wise process, the column loop counter is to be initiated */
			col = numColsB;

			/* For every row wise process, the pIn2 pointer is set
			 ** to the starting address of the pSrcB data */
			pIn2 = pSrcB->pData;

			j = 0u;

			/* column loop */
			do
			{
				/* Set the variable sum, that acts as accumulator, to zero */
				sum = 0.0f;

				/* Initiate the pointer pIn1 to point to the starting address of the column being processed */
				pIn1 = pInA;

				/* Apply loop unrolling and compute 4 MACs simultaneously. */
				colCnt = numColsA >> 2;

				/* matrix multiplication		*/
				while(colCnt > 0u)
				{
					/* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
					sum += *pIn1++ * (*pIn2);
					pIn2 += numColsB;
					sum += *pIn1++ * (*pIn2);
					pIn2 += numColsB;
					sum += *pIn1++ * (*pIn2);
					pIn2 += numColsB;
					sum += *pIn1++ * (*pIn2);
					pIn2 += numColsB;

					/* Decrement the loop count */
					colCnt--;
				}

				/* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.
				 ** No loop unrolling is used. */
				colCnt = numColsA % 0x4u;

				while(colCnt > 0u)
				{
					/* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
					sum += *pIn1++ * (*pIn2);
					pIn2 += numColsB;

					/* Decrement the loop counter */
					colCnt--;
				}

				/* Store the result in the destination buffer */
				*px++ = sum;

				/* Update the pointer pIn2 to point to the	starting address of the next column */
				j++;
				pIn2 = pSrcB->pData + j;

				/* Decrement the column loop counter */
				col--;

			}
			while(col > 0u);

			/* Update the pointer pInA to point to the  starting address of the next row */
			i = i + numColsB;
			pInA = pInA + numColsA;

			/* Decrement the row loop counter */
			row--;

		}
		while(row > 0u);

		/* Set status as ARM_MATH_SUCCESS */
		status = ARM_MATH_SUCCESS;
	}

	/* Return to application */
	return (status);

}

arm_status arm_mat_add_f32(
	const arm_matrix_instance_f32 * pSrcA,
	const arm_matrix_instance_f32 * pSrcB,
	arm_matrix_instance_f32 * pDst)
{
	float32_t *pIn1=pSrcA->pData;
	float32_t *pIn2=pSrcB->pData;
	float32_t *pOut=pDst->pData;
	float32_t inA1,inA2,inB1,inB2,out1,out2;
	uint32_t numSamples; 						  /* total number of elements in the matrix  */
	uint32_t blkCnt; 							  /* loop counters */
	arm_status status;							  /* status of matrix addition */
	if((pSrcA->numRows!=pSrcB->numRows)||
			(pSrcA->numCols!=pDst->numCols)||
			(pSrcA->numRows!=pDst->numRows)||
			(pSrcA->numCols!=pDst->numCols)
		)
	{
		status = ARM_MATH_SIZE_MISMATCH;
	}
	else
	{
		/* Total number of samples in the input matrix */
		numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;
		blkCnt = numSamples >> 2u;
		while(blkCnt > 0u)
		{
			/* C(m,n) = A(m,n) + B(m,n) */
			/* Add and then store the results in the destination buffer. */
			/* Read values from source A */
			inA1 = pIn1[0];

			/* Read values from source B */
			inB1 = pIn2[0];

			/* Read values from source A */
			inA2 = pIn1[1];

			/* out = sourceA + sourceB */
			out1 = inA1 + inB1;

			/* Read values from source B */
			inB2 = pIn2[1];

			/* Read values from source A */
			inA1 = pIn1[2];

			/* out = sourceA + sourceB */
			out2 = inA2 + inB2;

			/* Read values from source B */
			inB1 = pIn2[2];

			/* Store result in destination */
			pOut[0] = out1;
			pOut[1] = out2;

			/* Read values from source A */
			inA2 = pIn1[3];

			/* Read values from source B */
			inB2 = pIn2[3];

			/* out = sourceA + sourceB */
			out1 = inA1 + inB1;

			/* out = sourceA + sourceB */
			out2 = inA2 + inB2;

			/* Store result in destination */
			pOut[2] = out1;

			/* Store result in destination */
			pOut[3] = out2;


			/* update pointers to process next sampels */
			pIn1 += 4u;
			pIn2 += 4u;
			pOut += 4u;
			/* Decrement the loop counter */
			blkCnt--;
		}

		/* If the numSamples is not a multiple of 4, compute any remaining output samples here.
		 ** No loop unrolling is used. */
		blkCnt = numSamples % 0x4u;
		while(blkCnt > 0u)
		{
			/* C(m,n) = A(m,n) + B(m,n) */
			/* Add and then store the results in the destination buffer. */
			*pOut++ = (*pIn1++) + (*pIn2++);

			/* Decrement the loop counter */
			blkCnt--;
		}

		/* set status as ARM_MATH_SUCCESS */
		status = ARM_MATH_SUCCESS;

	}

	return (status);
}

arm_status arm_mat_scale_f32(
	const arm_matrix_instance_f32 * pSrc,
	float32_t scale,
	arm_matrix_instance_f32 * pDst)
{
	float32_t *pIn = pSrc->pData;				  /* input data matrix pointer */
	float32_t *pOut = pDst->pData;				  /* output data matrix pointer */
	uint32_t numSamples; 						  /* total number of elements in the matrix */
	uint32_t blkCnt; 							  /* loop counters */
	arm_status status;							  /* status of matrix scaling	  */

	/* Check for matrix mismatch condition */
	if((pSrc->numRows != pDst->numRows) || (pSrc->numCols != pDst->numCols))
	{
		/* Set status as ARM_MATH_SIZE_MISMATCH */
		status = ARM_MATH_SIZE_MISMATCH;
	}
	else

	{
		/* Total number of samples in the input matrix */
		numSamples = (uint32_t) pSrc->numRows * pSrc->numCols;

		/* Loop Unrolling */
		blkCnt = numSamples >> 2;

		/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
		 ** a second loop below computes the remaining 1 to 3 samples. */
		while(blkCnt > 0u)
		{
			/* C(m,n) = A(m,n) * scale */
			/* Scaling and results are stored in the destination buffer. */
			*pOut++ = (*pIn++) * scale;
			*pOut++ = (*pIn++) * scale;
			*pOut++ = (*pIn++) * scale;
			*pOut++ = (*pIn++) * scale;

			/* Decrement the numSamples loop counter */
			blkCnt--;
		}

		/* If the numSamples is not a multiple of 4, compute any remaining output samples here.
		 ** No loop unrolling is used. */
		blkCnt = numSamples % 0x4u;

		while(blkCnt > 0u)
		{
			/* C(m,n) = A(m,n) * scale */
			/* The results are stored in the destination buffer. */
			*pOut++ = (*pIn++) * scale;

			/* Decrement the loop counter */
			blkCnt--;
		}

		/* Set status as ARM_MATH_SUCCESS */
		status = ARM_MATH_SUCCESS;
	}

	/* Return to application */
	return (status);

}

arm_status arm_mat_sub_f32(
	const arm_matrix_instance_f32 * pSrcA,
	const arm_matrix_instance_f32 * pSrcB,
	arm_matrix_instance_f32 * pDst)
{
	float32_t *pIn1 = pSrcA->pData;				 /* input data matrix pointer A */
	float32_t *pIn2 = pSrcB->pData;				 /* input data matrix pointer B */
	float32_t *pOut = pDst->pData;				 /* output data matrix pointer	*/
	uint32_t numSamples;							 /* total number of elements in the matrix	*/
	uint32_t blkCnt;								 /* loop counters */
	arm_status status;							 /* status of matrix subtraction */

	/* Check for matrix mismatch condition */
	if((pSrcA->numRows != pSrcB->numRows) ||
			(pSrcA->numCols != pSrcB->numCols) ||
			(pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
	{
		/* Set status as ARM_MATH_SIZE_MISMATCH */
		status = ARM_MATH_SIZE_MISMATCH;
	}
	else

	{
		/* Total number of samples in the input matrix */
		numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;

		/* Loop Unrolling */
		blkCnt = numSamples >> 2u;

		/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
		 ** a second loop below computes the remaining 1 to 3 samples. */
		while(blkCnt > 0u)
		{
			/* C(m,n) = A(m,n) - B(m,n) */
			/* Subtract and then store the results in the destination buffer. */
			*pOut++ = (*pIn1++) - (*pIn2++);
			*pOut++ = (*pIn1++) - (*pIn2++);
			*pOut++ = (*pIn1++) - (*pIn2++);
			*pOut++ = (*pIn1++) - (*pIn2++);

			/* Decrement the loop counter */
			blkCnt--;
		}

		/* If the numSamples is not a multiple of 4, compute any remaining output samples here.
		 ** No loop unrolling is used. */
		blkCnt = numSamples % 0x4u;

		while(blkCnt > 0u)
		{
			/* C(m,n) = A(m,n) - B(m,n) */
			/* Subtract and then store the results in the destination buffer. */
			*pOut++ = (*pIn1++) - (*pIn2++);

			/* Decrement the loop counter */
			blkCnt--;
		}

		/* Set status as ARM_MATH_SUCCESS */
		status = ARM_MATH_SUCCESS;
	}

	/* Return to application */
	return (status);

}


arm_status arm_mat_trans_f32(
	const arm_matrix_instance_f32 * pSrc,
	arm_matrix_instance_f32 * pDst)
{
	float32_t *pIn = pSrc->pData; 				 /* input data matrix pointer */
	float32_t *pOut = pDst->pData;				 /* output data matrix pointer */
	float32_t *px;								 /* Temporary output data matrix pointer */
	uint16_t nRows = pSrc->numRows;				 /* number of rows */
	uint16_t nColumns = pSrc->numCols;			 /* number of columns */
	uint16_t blkCnt, i = 0u, row = nRows; 		 /* loop counters */
	arm_status status;							 /* status of matrix transpose	*/



	/* Check for matrix mismatch condition */
	if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
	{
		/* Set status as ARM_MATH_SIZE_MISMATCH */
		status = ARM_MATH_SIZE_MISMATCH;
	}
	else
	{
		/* Matrix transpose by exchanging the rows with columns */
		/* row loop 	*/
		do
		{
			/* Loop Unrolling */
			blkCnt = nColumns >> 2;

			/* The pointer px is set to starting address of the column being processed */
			px = pOut + i;

			/* First part of the processing with loop unrolling.	Compute 4 outputs at a time.
			 ** a second loop below computes the remaining 1 to 3 samples. */
			while(blkCnt > 0u)		/* column loop */
			{
				/* Read and store the input element in the destination */
				*px = *pIn++;

				/* Update the pointer px to point to the next row of the transposed matrix */
				px += nRows;

				/* Read and store the input element in the destination */
				*px = *pIn++;

				/* Update the pointer px to point to the next row of the transposed matrix */
				px += nRows;

				/* Read and store the input element in the destination */
				*px = *pIn++;

				/* Update the pointer px to point to the next row of the transposed matrix */
				px += nRows;

				/* Read and store the input element in the destination */
				*px = *pIn++;

				/* Update the pointer px to point to the next row of the transposed matrix */
				px += nRows;

				/* Decrement the column loop counter */
				blkCnt--;
			}

			/* Perform matrix transpose for last 3 samples here. */
			blkCnt = nColumns % 0x4u;

			while(blkCnt > 0u)
			{
				/* Read and store the input element in the destination */
				*px = *pIn++;

				/* Update the pointer px to point to the next row of the transposed matrix */
				px += nRows;

				/* Decrement the column loop counter */
				blkCnt--;
			}

			i++;

			/* Decrement the row loop counter */
			row--;

		}
		while(row > 0u);			/* row loop end  */

		/* Set status as ARM_MATH_SUCCESS */
		status = ARM_MATH_SUCCESS;
	}

	/* Return to application */
	return (status);

}


arm_status arm_mat_inverse_f32(
	const arm_matrix_instance_f32 * src,
	arm_matrix_instance_f32 * dst)
{
	float32_t *pIn = src->pData; 				 /* input data matrix pointer */
	float32_t *pOut = dst->pData;				 /* output data matrix pointer */
	float32_t *pInT1, *pInT2; 					 /* Temporary input data matrix pointer */
	float32_t *pInT3, *pInT4; 					 /* Temporary output data matrix pointer */
	float32_t *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;	/* Temporary input and output data matrix pointer */
	uint32_t numRows = src->numRows; 			 /* Number of rows in the matrix  */
	uint32_t numCols = src->numCols; 			 /* Number of Cols in the matrix  */
	float32_t Xchg, in = 0.0f, in1;				 /* Temporary input values	*/
	uint32_t i, rowCnt, flag = 0u, j, loopCnt, k, l;		/* loop counters */
	arm_status status;							 /* status of matrix inverse */


	/* Check for matrix mismatch condition */
	if((src->numRows != src->numCols) || (dst->numRows != dst->numCols)
			|| (src->numRows != dst->numRows))
	{
		/* Set status as ARM_MATH_SIZE_MISMATCH */
		status = ARM_MATH_SIZE_MISMATCH;
	}
	else
	{

		/*--------------------------------------------------------------------------------------------------------------
		 * Matrix Inverse can be solved using elementary row operations.
		 *
		 *	Gauss-Jordan Method:
		 *
		 *	   1. First combine the identity matrix and the input matrix separated by a bar to form an
		 *		  augmented matrix as follows:
		 *						_				   _		 _		   _
		 *					   |  a11  a12 | 1	 0	|		|  X11 X12	|
		 *					   |		   |		|	=	|			|
		 *					   |_ a21  a22 | 0	 1 _|		|_ X21 X21 _|
		 *
		 *		2. In our implementation, pDst Matrix is used as identity matrix.
		 *
		 *		3. Begin with the first row. Let i = 1.
		 *
		 *		4. Check to see if the pivot for row i is zero.
		 *		   The pivot is the element of the main diagonal that is on the current row.
		 *		   For instance, if working with row i, then the pivot element is aii.
		 *		   If the pivot is zero, exchange that row with a row below it that does not
		 *		   contain a zero in column i. If this is not possible, then an inverse
		 *		   to that matrix does not exist.
		 *
		 *		5. Divide every element of row i by the pivot.
		 *
		 *		6. For every row below and	row i, replace that row with the sum of that row and
		 *		   a multiple of row i so that each new element in column i below row i is zero.
		 *
		 *		7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
		 *		   for every element below and above the main diagonal.
		 *
		 *		8. Now an identical matrix is formed to the left of the bar(input matrix, pSrc).
		 *		   Therefore, the matrix to the right of the bar is our solution(pDst matrix, pDst).
		 *----------------------------------------------------------------------------------------------------------------*/

		/* Working pointer for destination matrix */
		pInT2 = pOut;

		/* Loop over the number of rows */
		rowCnt = numRows;

		/* Making the destination matrix as identity matrix */
		while(rowCnt > 0u)
		{
			/* Writing all zeroes in lower triangle of the destination matrix */
			j = numRows - rowCnt;
			while(j > 0u)
			{
				*pInT2++ = 0.0f;
				j--;
			}

			/* Writing all ones in the diagonal of the destination matrix */
			*pInT2++ = 1.0f;

			/* Writing all zeroes in upper triangle of the destination matrix */
			j = rowCnt - 1u;
			while(j > 0u)
			{
				*pInT2++ = 0.0f;
				j--;
			}

			/* Decrement the loop counter */
			rowCnt--;
		}

		/* Loop over the number of columns of the input matrix.
		   All the elements in each column are processed by the row operations */
		loopCnt = numCols;

		/* Index modifier to navigate through the columns */
		l = 0u;

		while(loopCnt > 0u)
		{
			/* Check if the pivot element is zero..
			 * If it is zero then interchange the row with non zero row below.
			 * If there is no non zero element to replace in the rows below,
			 * then the matrix is Singular. */

			/* Working pointer for the input matrix that points
			 * to the pivot element of the particular row  */
			pInT1 = pIn + (l * numCols);

			/* Working pointer for the destination matrix that points
			 * to the pivot element of the particular row  */
			pInT3 = pOut + (l * numCols);

			/* Temporary variable to hold the pivot value */
			in = *pInT1;

			/* Destination pointer modifier */
			k = 1u;

			/* Check if the pivot element is zero */
			if(*pInT1 == 0.0f)
			{
				/* Loop over the number rows present below */
				i = numRows - (l + 1u);

				while(i > 0u)
				{
					/* Update the input and destination pointers */
					pInT2 = pInT1 + (numCols * l);
					pInT4 = pInT3 + (numCols * k);

					/* Check if there is a non zero pivot element to
					 * replace in the rows below */
					if(*pInT2 != 0.0f)
					{
						/* Loop over number of columns
						 * to the right of the pilot element */
						j = numCols - l;

						while(j > 0u)
						{
							/* Exchange the row elements of the input matrix */
							Xchg = *pInT2;
							*pInT2++ = *pInT1;
							*pInT1++ = Xchg;

							/* Decrement the loop counter */
							j--;
						}

						/* Loop over number of columns of the destination matrix */
						j = numCols;

						while(j > 0u)
						{
							/* Exchange the row elements of the destination matrix */
							Xchg = *pInT4;
							*pInT4++ = *pInT3;
							*pInT3++ = Xchg;

							/* Decrement the loop counter */
							j--;
						}

						/* Flag to indicate whether exchange is done or not */
						flag = 1u;

						/* Break after exchange is done */
						break;
					}

					/* Update the destination pointer modifier */
					k++;

					/* Decrement the loop counter */
					i--;
				}
			}

			/* Update the status if the matrix is singular */
			if((flag != 1u) && (in == 0.0f))
			{
				status = ARM_MATH_SINGULAR;

				break;
			}

			/* Points to the pivot row of input and destination matrices */
			pPivotRowIn = pIn + (l * numCols);
			pPivotRowDst = pOut + (l * numCols);

			/* Temporary pointers to the pivot row pointers */
			pInT1 = pPivotRowIn;
			pInT2 = pPivotRowDst;

			/* Pivot element of the row */
			in = *(pIn + (l * numCols));

			/* Loop over number of columns
			 * to the right of the pilot element */
			j = (numCols - l);

			while(j > 0u)
			{
				/* Divide each element of the row of the input matrix
				 * by the pivot element */
				in1 = *pInT1;
				*pInT1++ = in1 / in;

				/* Decrement the loop counter */
				j--;
			}

			/* Loop over number of columns of the destination matrix */
			j = numCols;

			while(j > 0u)
			{
				/* Divide each element of the row of the destination matrix
				 * by the pivot element */
				in1 = *pInT2;
				*pInT2++ = in1 / in;

				/* Decrement the loop counter */
				j--;
			}

			/* Replace the rows with the sum of that row and a multiple of row i
			 * so that each new element in column i above row i is zero.*/

			/* Temporary pointers for input and destination matrices */
			pInT1 = pIn;
			pInT2 = pOut;

			/* index used to check for pivot element */
			i = 0u;

			/* Loop over number of rows */
			/*  to be replaced by the sum of that row and a multiple of row i */
			k = numRows;

			while(k > 0u)
			{
				/* Check for the pivot element */
				if(i == l)
				{
					/* If the processing element is the pivot element,
					 only the columns to the right are to be processed */
					pInT1 += numCols - l;

					pInT2 += numCols;
				}
				else
				{
					/* Element of the reference row */
					in = *pInT1;

					/* Working pointers for input and destination pivot rows */
					pPRT_in = pPivotRowIn;
					pPRT_pDst = pPivotRowDst;

					/* Loop over the number of columns to the right of the pivot element,
					 to replace the elements in the input matrix */
					j = (numCols - l);

					while(j > 0u)
					{
						/* Replace the element by the sum of that row
						   and a multiple of the reference row	*/
						in1 = *pInT1;
						*pInT1++ = in1 - (in **pPRT_in++);

						/* Decrement the loop counter */
						j--;
					}

					/* Loop over the number of columns to
					 replace the elements in the destination matrix */
					j = numCols;

					while(j > 0u)
					{
						/* Replace the element by the sum of that row
						   and a multiple of the reference row	*/
						in1 = *pInT2;
						*pInT2++ = in1 - (in **pPRT_pDst++);

						/* Decrement the loop counter */
						j--;
					}

				}

				/* Increment the temporary input pointer */
				pInT1 = pInT1 + l;

				/* Decrement the loop counter */
				k--;

				/* Increment the pivot index */
				i++;
			}

			/* Increment the input pointer */
			pIn++;

			/* Decrement the loop counter */
			loopCnt--;

			/* Increment the index modifier */
			l++;
		}

		/* Set status as ARM_MATH_SUCCESS */
		status = ARM_MATH_SUCCESS;

		if((flag != 1u) && (in == 0.0f))
		{
			status = ARM_MATH_SINGULAR;
		}

	}

	/* Return to application */
	return (status);

}
void arm_mat_init_f32(
	arm_matrix_instance_f32 * S,
	uint16_t nRows,
	uint16_t nColumns,
	float32_t * pData)
{
	/* Assign Number of Rows */
	S->numRows = nRows;

	/* Assign Number of Columns */
	S->numCols = nColumns;

	/* Assign Data pointer */
	S->pData = pData;
}
void arm_fill_f32(
	float32_t value,
	float32_t * pDst,
	uint32_t blockSize)
{
	uint32_t blkCnt; 							  /* loop counter */
	/* Run the below code for Cortex-M4 and Cortex-M3 */
	float32_t in1 = value;
	float32_t in2 = value;
	float32_t in3 = value;
	float32_t in4 = value;

	/*loop Unrolling */
	blkCnt = blockSize >> 2u;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
		/* C = value */
		/* Fill the value in the destination buffer */
		*pDst++ = in1;
		*pDst++ = in2;
		*pDst++ = in3;
		*pDst++ = in4;

		/* Decrement the loop counter */
		blkCnt--;
	}

	/* If the blockSize is not a multiple of 4, compute any remaining output samples here.
	** No loop unrolling is used. */
	blkCnt = blockSize % 0x4u;

	while(blkCnt > 0u)
	{
		/* C = value */
		/* Fill the value in the destination buffer */
		*pDst++ = value;

		/* Decrement the loop counter */
		blkCnt--;
	}

}


void SRCKF_New(SRCKF_Filter* srckf)
{
	float32_t kesi;
	arm_matrix_instance_f32 KesiPuls, KesiMinu;
	float32_t KesiPuls_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM], KesiMinus_f32[SRCKF_STATE_DIM * SRCKF_STATE_DIM];
	float32_t *S = srckf->S_f32;
	float32_t *SQ = srckf->SQ_f32;
	float32_t *SR = srckf->SR_f32;
	//////////////////////////////////////////////////////////////////////////
	//initialise weight
	srckf->W = 1.0f / (float32_t)SRCKF_CP_POINTS;
	arm_sqrt_f32(srckf->W, &srckf->SW);
	//initialise kesi
	//generate the cubature point
	kesi = (float32_t)SRCKF_STATE_DIM;
	arm_sqrt_f32(kesi, &kesi);
	arm_mat_init_f32(&KesiPuls, SRCKF_STATE_DIM, SRCKF_STATE_DIM, KesiPuls_f32);
	arm_mat_zero_f32(&KesiPuls);
	arm_mat_init_f32(&KesiMinu, SRCKF_STATE_DIM, SRCKF_STATE_DIM, KesiMinus_f32);
	arm_mat_zero_f32(&KesiMinu);
	arm_mat_identity_f32(&KesiPuls, kesi);
	arm_mat_identity_f32(&KesiMinu, -kesi);
	arm_mat_init_f32(&srckf->Kesi, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->Kesi_f32);
	arm_mat_setsubmatrix_f32(&srckf->Kesi, &KesiPuls, 0, 0);
	arm_mat_setsubmatrix_f32(&srckf->Kesi, &KesiMinu, 0, SRCKF_STATE_DIM);
	arm_mat_init_f32(&srckf->iKesi, SRCKF_STATE_DIM, 1, srckf->iKesi_f32);
	arm_mat_zero_f32(&srckf->iKesi);

	//initialise state covariance
	arm_mat_init_f32(&srckf->S, SRCKF_STATE_DIM, SRCKF_STATE_DIM, srckf->S_f32);
	arm_mat_zero_f32(&srckf->S);
	S[0] = S[8] = S[16] = S[24] = SRCKF_PQ_INITIAL;
	S[32] = S[40] = S[48] = SRCKF_PW_INITIAL;
	arm_mat_init_f32(&srckf->ST, SRCKF_STATE_DIM, SRCKF_STATE_DIM, srckf->ST_f32);
	//initialise measurement covariance
	arm_mat_init_f32(&srckf->SY, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SY_f32);
	arm_mat_init_f32(&srckf->SYI, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SYI_f32);
	arm_mat_init_f32(&srckf->SYT, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SYT_f32);
	arm_mat_init_f32(&srckf->SYTI, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SYTI_f32);
	arm_mat_init_f32(&srckf->PXY, SRCKF_STATE_DIM, SRCKF_MEASUREMENT_DIM, srckf->PXY_f32);
	arm_mat_init_f32(&srckf->tmpPXY, SRCKF_STATE_DIM, SRCKF_MEASUREMENT_DIM, srckf->tmpPXY_f32);
	//initialise SQ
	arm_mat_init_f32(&srckf->SQ, SRCKF_STATE_DIM, SRCKF_STATE_DIM, srckf->SQ_f32);
	arm_mat_zero_f32(&srckf->SQ);
	SQ[0] = SQ[8] = SQ[16] = SQ[24] = SRCKF_QQ_INITIAL;
	SQ[32] = SQ[40] = SQ[48] = SRCKF_QW_INITIAL;
	//initialise SR
	arm_mat_init_f32(&srckf->SR, SRCKF_MEASUREMENT_DIM, SRCKF_MEASUREMENT_DIM, srckf->SR_f32);
	arm_mat_zero_f32(&srckf->SR);
	SR[0] = SR[7] = SR[14] = SRCKF_RA_INITIAL;
	SR[21] = SR[28] = SR[35] = SRCKF_RM_INITIAL;
	//other stuff
	//cubature points
	arm_mat_init_f32(&srckf->XCP, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->XCP_f32);
	//propagated cubature points
	arm_mat_init_f32(&srckf->XPCP, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->XPCP_f32);
	arm_mat_init_f32(&srckf->YPCP, SRCKF_MEASUREMENT_DIM, SRCKF_CP_POINTS, srckf->YPCP_f32);
	arm_mat_init_f32(&srckf->XCPCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->XCPCM_f32);
	arm_mat_init_f32(&srckf->tmpXCPCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS, srckf->tmpXCPCM_f32);
	arm_mat_init_f32(&srckf->YCPCM, SRCKF_MEASUREMENT_DIM, SRCKF_CP_POINTS, srckf->YCPCM_f32);
	arm_mat_init_f32(&srckf->YCPCMT, SRCKF_CP_POINTS, SRCKF_MEASUREMENT_DIM, srckf->YCPCMT_f32);

	arm_mat_init_f32(&srckf->XCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS + SRCKF_STATE_DIM, srckf->XCM_f32);
	//initialise fill SQ
	arm_mat_setsubmatrix_f32(&srckf->XCM, &srckf->SQ, 0, SRCKF_CP_POINTS);

	arm_mat_init_f32(&srckf->YCM, SRCKF_MEASUREMENT_DIM, SRCKF_CP_POINTS + SRCKF_MEASUREMENT_DIM, srckf->YCM_f32);
	//initialise fill SR
	arm_mat_setsubmatrix_f32(&srckf->YCM, &srckf->SR, 0, SRCKF_CP_POINTS);

	arm_mat_init_f32(&srckf->XYCM, SRCKF_STATE_DIM, SRCKF_CP_POINTS + SRCKF_MEASUREMENT_DIM, srckf->XYCM_f32);
	//Kalman gain
	arm_mat_init_f32(&srckf->K, SRCKF_STATE_DIM, SRCKF_MEASUREMENT_DIM, srckf->K_f32);
	//////////////////////////////////////////////////////////////////////////
	//state vector
	arm_mat_init_f32(&srckf->X, SRCKF_STATE_DIM, 1, srckf->X_f32);
	arm_mat_zero_f32(&srckf->X);
	arm_mat_init_f32(&srckf->tmpX, SRCKF_STATE_DIM, 1, srckf->tmpX_f32);
	arm_mat_zero_f32(&srckf->tmpX);
	//measurement vector
	arm_mat_init_f32(&srckf->Y, SRCKF_MEASUREMENT_DIM, 1, srckf->Y_f32);
	arm_mat_zero_f32(&srckf->Y);
	arm_mat_init_f32(&srckf->tmpY, SRCKF_MEASUREMENT_DIM, 1, srckf->tmpY_f32);
	arm_mat_zero_f32(&srckf->tmpY);
}

void SRCKF_Init(SRCKF_Filter* srckf, float32_t *accel, float32_t *mag)
{
	float32_t *X = srckf->X_f32;
	// local variables
	float32_t norma, normx, normy;

	//3x3 rotation matrix
	float32_t R[9];
	// place the un-normalized gravity and geomagnetic vectors into
	// the rotation matrix z and x axes
	R[2] = accel[0];
	R[5] = accel[1];
	R[8] = accel[2];
	R[0] = mag[0];
	R[3] = mag[1];
	R[6] = mag[2];
	// set y vector to vector product of z and x vectors
	R[1] = R[5] * R[6] - R[8] * R[3];
	R[4] = R[8] * R[0] - R[2] * R[6];
	R[7] = R[2] * R[3] - R[5] * R[0];
	// set x vector to vector product of y and z vectors
	R[0] = R[4] * R[8] - R[7] * R[5];
	R[3] = R[7] * R[2] - R[1] * R[8];
	R[6] = R[1] * R[5] - R[4] * R[2];
	// calculate the vector moduli invert
	norma = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	normx = FastSqrtI(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
	normy = FastSqrtI(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
	// normalize the rotation matrix
	// normalize x axis
	R[0] *= normx;
	R[3] *= normx;
	R[6] *= normx;
	// normalize y axis
	R[1] *= normy;
	R[4] *= normy;
	R[7] *= normy;
	// normalize z axis
	R[2] *= norma;
	R[5] *= norma;
	R[8] *= norma;
	Quaternion_FromRotationMatrix(R, X);
}

void SRCKF_Update(SRCKF_Filter* srckf, float32_t *gyro, float32_t *accel, float32_t *mag, float32_t dt)
{
	//////////////////////////////////////////////////////////////////////////
	float32_t q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//
	float32_t hx, hy, hz;
	float32_t bx, bz;
	float32_t _2mx, _2my, _2mz;
	//
	int col;
	float32_t dQ[4];
	float32_t norm;
	float32_t *X = srckf->X_f32, *Y = srckf->Y_f32;
	float32_t *tmpX = srckf->tmpX_f32, *tmpY = srckf->tmpY_f32;
	float32_t *iKesi = srckf->iKesi_f32;

	dQ[0] = 0.0f;
	dQ[1] = gyro[0] - X[4];
	dQ[2] = gyro[1] - X[5];
	dQ[3] = gyro[2] - X[6];
	//////////////////////////////////////////////////////////////////////////
	//time update
	for(col = 0; col < SRCKF_CP_POINTS; col++)
	{
		//evaluate the cubature points
		arm_mat_getcolumn_f32(&srckf->Kesi, iKesi, col);
		arm_mat_mult_f32(&srckf->S, &srckf->iKesi, &srckf->tmpX);
		arm_mat_add_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCP, tmpX, col);
		//evaluate the propagated cubature points
		Quaternion_RungeKutta4(tmpX, dQ, dt, 1);
		arm_mat_setcolumn_f32(&srckf->XPCP, tmpX, col);
	}
	//estimate the predicted state
	arm_mat_cumsum_f32(&srckf->XPCP, tmpX, X);
	arm_mat_scale_f32(&srckf->X, srckf->W, &srckf->X);
	//estimate the square-root factor of the predicted error covariance
	for(col = 0; col < SRCKF_CP_POINTS; col++)
	{
		arm_mat_getcolumn_f32(&srckf->XPCP, tmpX, col);
		arm_mat_sub_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_scale_f32(&srckf->tmpX, srckf->SW, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCM, tmpX, col);
	}
	//XCM[XPCP, SQ], SQ fill already
	arm_mat_qr_decompositionT_f32(&srckf->XCM, &srckf->ST);
	arm_mat_trans_f32(&srckf->ST, &srckf->S);
	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//normalize accel and magnetic
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;
	//////////////////////////////////////////////////////////////////////////
	norm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm;
	mag[1] *= norm;
	mag[2] *= norm;

	_2mx = 2.0f * mag[0];
	_2my = 2.0f * mag[1];
	_2mz = 2.0f * mag[2];
	for(col = 0; col < SRCKF_CP_POINTS; col++)
	{
		//evaluate the cubature points
		arm_mat_getcolumn_f32(&srckf->Kesi, iKesi, col);
		arm_mat_mult_f32(&srckf->S, &srckf->iKesi, &srckf->tmpX);
		arm_mat_add_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCP, tmpX, col);
		//evaluate the propagated cubature points
		//auxiliary variables to avoid repeated arithmetic
		q0q0 = tmpX[0] * tmpX[0];
		q0q1 = tmpX[0] * tmpX[1];
		q0q2 = tmpX[0] * tmpX[2];
		q0q3 = tmpX[0] * tmpX[3];
		q1q1 = tmpX[1] * tmpX[1];
		q1q2 = tmpX[1] * tmpX[2];
		q1q3 = tmpX[1] * tmpX[3];
		q2q2 = tmpX[2] * tmpX[2];
		q2q3 = tmpX[2] * tmpX[3];
		q3q3 = tmpX[3] * tmpX[3];
		//reference direction of earth's magnetic field
		hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
		hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
		hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
		arm_sqrt_f32(hx * hx + hy * hy, &bx);
		bz = hz;

		tmpY[0] = 2.0f * (q1q3 - q0q2);
		tmpY[1] = 2.0f * (q2q3 + q0q1);
		tmpY[2] = -1.0f + 2.0f * (q0q0 + q3q3);
		tmpY[3] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
		tmpY[4] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
		tmpY[5] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));
		arm_mat_setcolumn_f32(&srckf->YPCP, tmpY, col);
	}
	//estimate the predicted measurement
	arm_mat_cumsum_f32(&srckf->YPCP, tmpY, Y);
	arm_mat_scale_f32(&srckf->Y, srckf->W, &srckf->Y);
	//estimate the square-root of the innovation covariance matrix
	for(col = 0; col < SRCKF_CP_POINTS; col++)
	{
		arm_mat_getcolumn_f32(&srckf->YPCP, tmpY, col);
		arm_mat_sub_f32(&srckf->tmpY, &srckf->Y, &srckf->tmpY);
		arm_mat_scale_f32(&srckf->tmpY, srckf->SW, &srckf->tmpY);
		arm_mat_setcolumn_f32(&srckf->YCPCM, tmpY, col);
		arm_mat_setcolumn_f32(&srckf->YCM, tmpY, col);
	}
	//YCM[YPCP, SR], SR fill already
	arm_mat_qr_decompositionT_f32(&srckf->YCM, &srckf->SYT);

	//estimate the cross-covariance matrix
	for(col = 0; col < SRCKF_CP_POINTS; col++)
	{
		arm_mat_getcolumn_f32(&srckf->XCP, tmpX, col);
		arm_mat_sub_f32(&srckf->tmpX, &srckf->X, &srckf->tmpX);
		arm_mat_scale_f32(&srckf->tmpX, srckf->SW, &srckf->tmpX);
		arm_mat_setcolumn_f32(&srckf->XCPCM, tmpX, col);
	}
	arm_mat_trans_f32(&srckf->YCPCM, &srckf->YCPCMT);
	arm_mat_mult_f32(&srckf->XCPCM, &srckf->YCPCMT, &srckf->PXY);

	//estimate the kalman gain
	arm_mat_trans_f32(&srckf->SYT, &srckf->SY);
	arm_mat_inverse_f32(&srckf->SYT, &srckf->SYTI);
	arm_mat_inverse_f32(&srckf->SY, &srckf->SYI);
	arm_mat_mult_f32(&srckf->PXY, &srckf->SYTI, &srckf->tmpPXY);
	arm_mat_mult_f32(&srckf->tmpPXY, &srckf->SYI, &srckf->K);

	//estimate the updated state
	Y[0] = accel[0] - Y[0];
	Y[1] = accel[1] - Y[1];
	Y[2] = accel[2] - Y[2];
	Y[3] = mag[0] - Y[3];
	Y[4] = mag[1] - Y[4];
	Y[5] = mag[2] - Y[5];
	arm_mat_mult_f32(&srckf->K, &srckf->Y, &srckf->tmpX);
	arm_mat_add_f32(&srckf->X, &srckf->tmpX, &srckf->X);
	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;
	//estimate the square-root factor of the corresponding error covariance
	arm_mat_mult_f32(&srckf->K, &srckf->YCPCM, &srckf->tmpXCPCM);
	arm_mat_sub_f32(&srckf->XCPCM, &srckf->tmpXCPCM, &srckf->XCPCM);
	arm_mat_setsubmatrix_f32(&srckf->XYCM, &srckf->XCPCM, 0, 0);
	arm_mat_mult_f32(&srckf->K, &srckf->SR, &srckf->tmpPXY);
	arm_mat_setsubmatrix_f32(&srckf->XYCM, &srckf->tmpPXY, 0, SRCKF_CP_POINTS);
	arm_mat_qr_decompositionT_f32(&srckf->XYCM, &srckf->ST);
	arm_mat_trans_f32(&srckf->ST, &srckf->S);
}

void SRCKF_GetAngle(SRCKF_Filter* srckf, float32_t* rpy)
{
	float32_t R[3][3];
	float32_t *X = srckf->X_f32;
	//Z-Y-X
	R[0][0] = 2.0f * (X[0] * X[0] + X[1] * X[1]) - 1.0f;
	R[0][1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
	R[0][2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
	//R[1][0] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//R[1][1] = 2.0f * (X[0] * X[0] + X[2] * X[2]) - 1.0f;
	R[1][2] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
	//R[2][0] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//R[2][1] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	R[2][2] = 2.0f * (X[0] * X[0] + X[3] * X[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(R[1][2], R[2][2]);
	if (rpy[0] == SRCKF_PI)
		rpy[0] = -SRCKF_PI;
	//pitch
	if (R[0][2] >= 1.0f)
		rpy[1] = -SRCKF_HALFPI;
	else if (R[0][2] <= -1.0f)
		rpy[1] = SRCKF_HALFPI;
	else
		rpy[1] = FastAsin(-R[0][2]);
	//yaw
	rpy[2] = FastAtan2(R[0][1], R[0][0]);
	if (rpy[2] < 0.0f)
	{
		rpy[2] += SRCKF_TWOPI;
	}
	if (rpy[2] > SRCKF_TWOPI)
	{
		rpy[2] = 0.0f;
	}

	rpy[0] = SRCKF_TODEG(rpy[0]);
	rpy[1] = SRCKF_TODEG(rpy[1]);
	rpy[2] = SRCKF_TODEG(rpy[2]);
}
