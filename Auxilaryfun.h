#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>


// Define a structure to overload the vector type
template<typename T>
struct vector {
	T x, y, z;

	T norm() { return std::sqrt(x * x + y * y + z * z); };

	// Vector construction
	vector(T x = 0, T y = 0, T z = 0) : x(x), y(y), z(z) {}

	// Overload + operator
	vector<T> operator+(const vector<T> rhs) {
		return vector<T>(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	// Overload - operator
	vector<T> operator-(const vector<T> rhs) {
		return vector<T>(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	// Overload the == operator
	bool operator==(const vector<T> rhs) const {
		return x == rhs.x
			&& (y == rhs.y)
			&& (z == rhs.z);
	}

	// Overload * for multiplication of vector with a scalar
	vector<T> operator*(const double rhs) 
	{
		return vector<T>(x * rhs, y * rhs, z * rhs);
	}

	vector<T> operator/(const double rhs) 
	{
		return vector<T>(x/rhs,y/rhs,z/rhs);
	}
	
	// Cross product
	vector<T> operator*(const vector<T> rhs)
	{
		return vector<T>(y * rhs.z - z * rhs.y, z*rhs.x-x*rhs.z,  x*rhs.y-y*rhs.x);
	}

	// Dot product
	double DotProduct(const vector<T> v2)
	{
		return x*v2.x+ y * v2.y+ z * v2.z;
	}

	//friend std::istream& operator>>(std::istream&, vector<T>&);
	//friend std::ostream& operator<<(std::ostream&, vector<T>&);
};



// A function to split a line of string into a list of strings
std::vector<std::string> lineToListOfStrings(const std::string &Currline);

// A function to determine if a vector is in a list of vectors
bool isInVecofVec(std::vector<std::vector<int> > MainVec, std::vector<int> myVec);

// A function to find the number of shared elements in two vectors
std::vector<int> NumSharedElements(std::vector<int> vec1, std::vector<int> vec2);

// A function to determine how many planes in the sequence of planes shared edges with the arbitrary plane
size_t NumSharedPlanesInSequence(std::vector<int> seqPlanes, std::vector<std::vector<int> > sharedList, int myPlane);