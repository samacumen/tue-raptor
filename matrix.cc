/*
 * 5HC99 Quadcopter project, group 1.
 * Some matrix & vector types and functions
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matrix.h"


/********************
 * vector Class
 ********************/		
		
vector::vector(unsigned int length) {
	this->n = length;
	this->data = new float[length];
}

vector::vector(float x, float y, float z) {
	this->n = 3;
	this->data = new float[3];
	
	this->data[0] = x;
	this->data[1] = y;
	this->data[2] = z;
}

vector::vector(vector* src) {
	this->n = src->length();
	this->data = new float[n];
	for (unsigned int i=0; i<n; i++)
		this->data[i] = (*src)[i];
}

void vector::set(unsigned int index, float value) {
	if (index>n) {
		fprintf(stderr, "Index exceeds vector dimensions\n");
		exit(1);
	}
	this->data[index] = value;
}

unsigned int vector::length() const {
	return this->n;
}

void vector::normalize() {
	// Normalize the vector
	float scale = 0;
	for (unsigned int i=0;i<this->n;i++) {
		scale += this->data[i]*this->data[i];
	}
	scale = 1/sqrt(scale);
	for (unsigned int i=0;i<this->n;i++) {
		this->data[i] *= scale;
	}
	return;
}

void vector::print() const {
	printf("[");
	for (unsigned int i=0;i<n;i++)
		printf("\t%f\t", data[i]);
	printf("]\n");
	return;
}

// Overloaded operators

vector vector::operator+ (const vector param) const {
	// Elementwise adding
	if (n!=param.length()) {
		fprintf(stderr, "Vectors to add don't match in length\n");
		exit(1);
	}
	vector temp = new vector(n);
	for (unsigned int i=0; i<n; i++)
		temp.set(i, this->data[i] + param[i]);
	return (temp);
}

vector vector::operator- (const vector param) const {
	// Elementwise substraction
	if (n!=param.length()) {
		fprintf(stderr, "Vectors to add don't match in length\n");
		exit(1);
	}
	vector temp = new vector(n);
	for (unsigned int i=0; i<n; i++)
		temp.set(i, this->data[i] - param[i]);
	return (temp);
}

vector vector::operator* (float scale) const {
	// Multiply with scalar
	vector temp = new vector(n);
	for (unsigned int i=0; i<n; i++)
		temp.set(i, data[i] * scale);
	return (temp);
}

float vector::operator[] (const unsigned int index) const {
	if (index>n-1) {
		fprintf(stderr, "Index out of range for vector\n");
		exit(1);
	}
	return data[index];
}
		
// Vector helper functions
float vector_innerprod (vector* v1, vector* v2) {
	// Inner product
	if (v1->length()!=v2->length()) {
		fprintf(stderr, "Vectors for inner product don't match in length\n");
		exit(1);
	}
	float sum =0;
	for (unsigned int i=0; i<v1->length(); i++)
		sum += (*v1)[i] * (*v2)[i];
	return (sum);
}

/********************
 * matrix Class
 ********************/
 
matrix::matrix(unsigned int rows, unsigned int cols) {
	this->n = rows;
	this->m = cols;
	this->data = new float*[n];

	for (unsigned int i=0; i<n; i++) {
		this->data[i] = new float[n];
	}

}

matrix::matrix(matrix* src) {
	this->n = src->rows();
	this->m = src->cols();
	this->data = new float*[n];
	for (unsigned int i=0; i<n; i++)
		this->data[i] = new float[m];
	for (unsigned int i=0; i<n; i++) {
		for (unsigned int j=0; j<m; j++) {
			this->data[i][j] = src->data[i][j];
		}
	}
}

unsigned int matrix::cols() {
	return m;
}

unsigned int matrix::rows() {
	return n;
}
	
void matrix::print() const {	
	for (unsigned int i=0;i<n;i++) {
		printf("[");
		for (unsigned int j=0;j<m;j++)
			printf("\t%f\t", data[i][j]);
		printf("]\n");
	}
	printf("\n");
	return;
}

void matrix::transpose() {
	float ** newdata = new float*[m];
	for (unsigned int i=0; i<m; i++)
		newdata[i] = new float[n];
		
	for (unsigned int i=0; i<n; i++) {
		for (unsigned int j=0; j<m; j++) {
			newdata[j][i] = this->data[i][j];
		}
	}
	unsigned int m_old = this->m;
	this->m = this->n;
	this->n = m_old;
	delete this->data;
	this->data = newdata;
	return;
}

int matrix::invert() {
	// Matrix Inversion Routine from http://www.arduino.cc/playground/Code/MatrixMath
	// * This function inverts a matrix based on the Gauss Jordan method.
	// * Specifically, it uses partial pivoting to improve numeric stability.
	// * The algorithm is drawn from those presented in 
	//       NUMERICAL RECIPES: The Art of Scientific Computing.
	// * The function returns 1 on success, 0 on failure.
	// * NOTE: The result is written in the source matrix!
	if (m!=n) {
		fprintf(stderr, "Cannot invert non-square matrix! You might want to use pseudo_invert().\n");
		exit(1);		
	}
	unsigned int pivrow;	// keeps track of current pivot row
	unsigned int k,i,j;     // k: overall index along diagonal; i: row index; j: col index
	unsigned int pivrows[n]; // keeps track of rows swaps to undo at end
	float tmp;              // used for finding max value and making column swaps
	
	for (k = 0; k < n; k++) {
		// find pivot row, the row with biggest entry in current column
		tmp = 0;
		for (i = k; i < n; i++)
		{
			if (fabs(data[i][k]) >= tmp)      // 'Avoid using other functions inside abs()?'
			{
				tmp = fabs(data[i][k]);
				pivrow = i;
			}
		}
		
		// check for singular matrix
		if (data[pivrow][k] == 0.0f)
		{
			//Inversion failed due to singular matrix
			return 0;
		}
		
		// Execute pivot (row swap) if needed
		if (pivrow != k)
		{
			// swap row k with pivrow
			for (j = 0; j < n; j++)
			{
				tmp = data[k][j];
				data[k][j] = data[pivrow][j];
				data[pivrow][j] = tmp;
			}
		}
		pivrows[k] = pivrow;    // record row swap (even if no swap happened)
		
		tmp = 1.0f/data[k][k];  // invert pivot element
		data[k][k] = 1.0f;		// This element of input matrix becomes result matrix
		
		// Perform row reduction (divide every element by pivot)
		for (j = 0; j < n; j++)
		{
			data[k][j] *= tmp;
		}
		
		// Now eliminate all other entries in this column
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				tmp = data[i][k];
				data[i][k] = 0.0f;  // The other place where in matrix becomes result mat
				for (j = 0; j < n; j++)
				{
					data[i][j] = data[i][j] - data[k][j]*tmp;
				}
			}
		}
	}

	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
	for (k = n-1; (k >= 0)&&(k<n); k--)	// Hack because k is unsigned int
	{
		if (pivrows[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				tmp = data[i][k];
				data[i][k] = data[i][pivrows[k]];
				data[i][pivrows[k]] = tmp;
			}
		}
	}

	return 1;
}

matrix matrix::pseudo_inverse() {
	//pseudo_inverse(M) = inv(MT*M)*MT
	// If matrix is tall (n>m), this results in the left inverse: pseudo_inv(m)*m = I
	// If matrix is wide (m>n), this results in the right inverse: m*pseudo_inv(m) = I

	matrix MT = matrix(this);
	MT.transpose();			// MT is transposed of myself
	matrix temp = MT*this;
	temp.invert();
	matrix result = temp*MT;
	return result;
}

vector matrix::operator* (vector param) {
	// Matrix * column vector
	if (m!=param.length()) {
		fprintf(stderr, "Matrix/vector dimensions don't match\n");
		exit(1);
	}	
	vector result(n);
	float temp;
	for (unsigned int i=0; i<n; i++) {
		temp = 0;
		for (unsigned int j=0; j<m; j++) {
			temp += data[i][j]*param[j];
		}
		result.set(i,temp);
	}
	return result;
}

matrix matrix::operator* (matrix param) {
	// Matrix * Matrix
	if (m!=param.rows()) {
		fprintf(stderr, "Matrix dimensions don't match\n");
		exit(1);
	}	
	matrix result(n,param.cols());
	for (unsigned int i=0; i<n; i++) {
		for (unsigned int j=0; j<param.cols(); j++) {
			result.data[i][j] = 0;
			for (unsigned int k=0; k<m; k++) {
				result.data[i][j] += data[i][k] * param.data[k][j];
			}
		}
	}
	return result;
}
