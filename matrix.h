/*
 * 5HC99 Quadcopter project, group 1.
 * Some matrix & vector types and functions
 */
 
#ifndef _MATRIX_H
#define _MATRIX_H

class vector {
	public:
		// Methods
		vector(unsigned int length);		// n-dimensional vector
		vector(float x, float y, float z);	// 3d vector
		vector(vector* src);				// Copy src
		
		void set(unsigned int index, float value);	// Set an element
		unsigned int length() const;				
		void normalize();
		
		void print() const;
		
		// Overloaded operators
		vector operator + (const vector param) const;				// Elementwise adding
		vector operator - (const vector param) const;				// Elementwise subtracting
		vector operator * (float scale) const;				// Multiply with scalar
		float operator [] (const unsigned int index) const;		// Return indexed element
		
	private:
		float* data;
		unsigned int n;

};

// Vector helper functions
float vector_innerprod (vector* v1, vector* v2);


class matrix {
	public:
		// Variables
		float** data;

		// Methods
		matrix(unsigned int rows, unsigned int cols);
		matrix(matrix* src);
		
		unsigned int cols();
		unsigned int rows();

		void print() const;
		void transpose();
		int invert();
		matrix pseudo_inverse();		// Moore-Penrose pseudo inverse
		
		// Overloaded operators
		vector operator * (vector);		// Matrix * column vector
		matrix operator * (matrix);		// Matrix * Matrix
	private:
		unsigned int m;	// Columns
		unsigned int n;	// Rows
};

#endif
