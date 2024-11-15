#pragma once

// This header file entail the functions to locate the cages within hydrate
#include "Auxilaryfun.h"
#include <algorithm>
#include <unordered_map>


class CageDetectionAlgorithms
{
private:
	// Declare a Vector(Vector<int>) type
	typedef  std::vector<std::vector<int> > VecOfVec;
	
	typedef  std::vector< std::unordered_map<int, int> > VecOfUnorMap;

	inline bool CheckLateral(int p, VecOfVec AllPlanes, VecOfVec SharedPlanes)
	{
		bool isLateral;
		if (SharedPlanes[p].size() >= AllPlanes[p].size())
		{
			isLateral = true;
		}
		else
		{
			isLateral = false;
		}

		return isLateral;
	}


public:

	// Overload the output operator
	/*std::ostream& operator<< (std::ostream& stm, VecOfVec v1)
	{
		// write one vector per line

	}*/
	

	class Cage
	{
	public:
		std::vector<int> planes;
		std::string label;
		vector<double> centroid;
		std::vector<std::string> guestMolIds; // a guest molecule is trapped inside the cage if its coordinates overlaps enough with the centroid of the cage
		void set_values(std::vector<int> pl, std::string cglbl,vector<double> cntr, std::vector<std::string> gMols)
		{
			planes = pl;
			label = cglbl;
			centroid = cntr;
			guestMolIds = gMols;
		}
	};


	typedef std::vector<CageDetectionAlgorithms::Cage> VecOfCages;

	VecOfVec Detect02PlaneCages(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes);

	VecOfVec Detect03PlaneCages(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec& NumOccEd3Planes);

	VecOfVec Detect04PlaneCages(VecOfVec AllPlanes, VecOfVec Sharedplanes, VecOfVec NeighboringList);

	VecOfVec Plane2XfromXPlanes(VecOfVec planesX, VecOfVec planesXSorted, VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NumOccEdPlane, std::vector<int> NumOccEdGoal, VecOfVec& NumOccEd6Planes);

	VecOfVec Detect2XPlaneCagesfromXPlanes(VecOfVec planesX, VecOfVec planesY, VecOfVec planesXYSorted,  VecOfVec AllPlanes,  VecOfVec SharedPlanes, VecOfVec NumOccEdPlane);

	std::vector<CageDetectionAlgorithms::Cage> RemoveDuplicatesCages(std::vector<CageDetectionAlgorithms::Cage> InputVec, std::vector<CageDetectionAlgorithms::Cage>& InputVecRemDupUnsorted);

	int DFSatisifed(std::vector<int> planesX, std::vector<int> planesY, VecOfVec SharedPlanes, VecOfVec AllPlanes, std::vector<int> NumEdOccPlaneX, std::vector<int> NumEdOccGoal, std::vector<int>& NumOccEdUpdated);

	/*VecOfUnorMap Detect2XPlanefromXPlanesUnordMap(VecOfUnorMap planesX, VecOfUnorMap planesY, VecOfVec AllPlanes, VecOfUnorMap SharedPlanes);*/

	VecOfVec CheckCageConstraint(VecOfVec planesX, VecOfVec AllPlanes, VecOfVec SharedPlanes);

	VecOfVec Detect06PlaneCages(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec& Cages6PlaneUnsorted, VecOfVec& DFAllPlanes);

	VecOfVec Detect06PlaneCagesUnorderMap(VecOfVec AllPlanes, VecOfUnorMap SharedPlanes);

	VecOfVec Detect07PlaneCages(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes);

	VecOfVec Detect12PlaneCagesfrom06Planes(VecOfVec planes06, VecOfVec AllPlanes, VecOfVec SharedPlanes);

	VecOfVec Detect12PlaneCages(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes);


	// This function returns the list of the 14plane cages
	VecOfVec Detect14PlaneCages(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes);

	// This function removes the duplicates from the vector of vector object
	VecOfVec RemoveDuplicates(VecOfVec InputVector, VecOfVec& InputVecRemDupUnsorted, VecOfVec& NumOccEdPlanesAll, VecOfVec& NumOccEdPlanesUniqueUnsorted);

	int OverlapWithPlanes(std::vector<int> Plast, std::vector<int> Pcur, VecOfVec SharedPlanes);

	// This function returns the list of the 14plane cages using the probability move methhod
	VecOfVec Detect14PlaneCagesProbMove(VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes);


	class GeneralCageAlg 
	{

	private:

	public:
		void    CalculateCageCentroid(std::vector<CageDetectionAlgorithms::Cage>& Cages, std::vector<vector<double> > PlanesCentroids, std::vector<double> PlanesAreas);

	};


	class CageIdentViaCup
	{

	private:


	public:

		int CheckMutualConnection(std::vector<int> PlanesListTmp, VecOfVec SharedPlanes);

		std::vector<CageDetectionAlgorithms::Cage> cup5_6(VecOfVec AllPlanes, VecOfVec SharedPlanes);


		int CheckCupOverlap5_12(std::vector<int> cup1, std::vector<int> cup2, VecOfVec SharedPlanes);

		std::vector<CageDetectionAlgorithms::Cage> twoCupCagesSym(std::vector<CageDetectionAlgorithms::Cage> Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes,std::string cageType);

		int FindCupsOverlap(std::vector<int> cup1, std::vector<int> cup2);

		std::vector<CageDetectionAlgorithms::Cage> FourCupCagesSym(std::vector<CageDetectionAlgorithms::Cage> Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes, std::string cageType);

		std::vector<CageDetectionAlgorithms::Cage> ThreeCupCagesSym(std::vector<CageDetectionAlgorithms::Cage> Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes,std::string cageType);

		std::vector<CageDetectionAlgorithms::Cage> cup5_6_6_1(VecOfVec AllPlanes, VecOfVec SharedPlanes);

		VecOfVec FourCupCages(VecOfVec Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes);

	};


};