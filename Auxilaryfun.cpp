
#include "Auxilaryfun.h"
#include <algorithm>
#include <iterator>

// A function to split a line of string into a list of strings
//std::vector<std::string> lineToListOfStrings(std::string Currline)
//{
//	char* cstr = new char[Currline.size() + 1];
//	strcpy_s(cstr, sizeof cstr, Currline.c_str());
//
//	char* token = strtok(cstr, " ");
//	std::vector<std::string> words;
//	while (token != NULL)
//	{
//		words.push_back(token);
//		token = strtok(NULL, " ");
//	}
//	return words;
//}

std::vector<std::string> lineToListOfStrings(const std::string &Currline)
{
	std::istringstream iss(Currline);
	std::vector<std::string> words;
	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(words));

	return words;
}




// A function to determine if a vector is in a list of vectors
bool isInVecofVec(std::vector<std::vector<int>> MainVec, std::vector<int> myVec)
{
	std::vector<std::vector<int>>::iterator itr;
	bool numOcc{};
	//std::sort(begin(myVec), end(myVec));
	for (int i = 0; i < MainVec.size(); i++)
	{
		// Check if myVec is in the corresponding element of the MainVec
		//int s = 0;
		//for (int j=0;j< myVec.size();j++)
		//{
		//	//std::find(MainVec[i].begin(), MainVec[i].end(), myVec[j]) != MainVec[i].end()
		//	if (std::binary_search(MainVec[i].begin(), MainVec[i].end(), myVec[j]))
		//	{
		//		s += 1;
		//	}
		//}
		//if (s == myVec.size())
		//{
		//	// There is a vector identical to myVec inside the MainVec
		//	numOcc = true;
		//	break;
		//}
		//std::sort(begin(MainVec[i]), end(MainVec[i]));
		std::vector<int> PlaneOverlap(std::min(MainVec[i].size(), myVec.size()));
		auto it = std::set_intersection(MainVec[i].begin(), MainVec[i].end(), myVec.begin(), myVec.end(), PlaneOverlap.begin());
		PlaneOverlap.resize(it - PlaneOverlap.begin());
		if (PlaneOverlap.size()== myVec.size())
		{
			numOcc = true;
			break;
		}
	}
	return numOcc;
}


// A function to remove duplicate vectors from a vector of vectors
void RemoveDuplicatesVecOfVec(std::vector<std::vector<int>> MainVec)
{

}

// A function to find the number of shared elements between two vectors and write it out
std::vector<int> NumSharedElements(std::vector<int> vec1, std::vector<int> vec2)
{
	std::vector<int> SharedElements;
	for (std::vector<int>::iterator i = vec1.begin(); i != vec1.end(); i++)
	{
		if (std::find(vec2.begin(), vec2.end(), *i) != vec2.end())
		{
			SharedElements.push_back(*i);
		}
	}
	return SharedElements;

}

// A function to determine how many planes in the sequence of planes share edges with an arbitrary plane
size_t NumSharedPlanesInSequence(std::vector<int> seqPlanes, std::vector<std::vector<int>> sharedList, int myPlane)
{
	size_t num = 0;
	for (int plane: seqPlanes) 
	{
		if (std::count(sharedList[plane].begin(), sharedList[plane].end(), myPlane) != 0)
		{
			num += 1;
		}
	}
	return num;
}