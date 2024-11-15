#pragma once
#include <vector>
#include "HBondedMol.h"
#include "Auxilaryfun.h"
#include <unordered_map>
#include <map>
#include <math.h>
#include <cmath>

class Planes
{
private:
	typedef  std::vector<std::vector<int> > VecOfVec;
	typedef  std::vector< std::unordered_map<int, int> > VecOfUnorMap;
	

public:

	class plane 
	{
	public:
		std::vector<int> WaterMols;
		std::string type;
		vector<double> centroid;
		void set_values(std::vector<int> wtr, std::string tp, vector<double> cntr)
		{
			WaterMols = wtr;
			type = tp;
			centroid = cntr;
		}
	};

	// A method to identify the quadrilaterals formed by the molecules
	VecOfVec FindQuadrilateral(size_t numWaters,  VecOfVec HbondNeigh);
	

	// A method to identify the pentagons formed by the molecules
	VecOfVec FindPentagons(size_t numWaters,  VecOfVec HbondNeigh);
	

	// A method to identify the hexagons formed by the molecules
	VecOfVec FindHexagonals(size_t numWaters,  VecOfVec HbondNeigh);
	
	class CheckPlaneConstraints 
	{

	public:
		// A method to fliter out the the planes that do not satisfy the coplanarity requirement
		VecOfVec  CheckCoplanarityConstraint(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Waters, const double Coplanaritory_thr);

		// A method to filter the regular planes
		VecOfVec CheckRegularPlaneConstraint(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Waters, const double Lth);

	};
	// A method to append all different planes into one single list
	VecOfVec CollectAllPlanes(const VecOfVec quad, const VecOfVec pentagon,  VecOfVec hexagon, std::vector<int>& AllPlaneType);
	
	// A method to find the edge-sharing planes for each plane in the AllPlane list
	VecOfVec FindSharingEdgePlanes(const VecOfVec AllPlanes);
	

	
	

	

	VecOfVec FindNeighboringPlanes(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Waters, double Rth);

	std::vector<vector<double> > PlaneCentroid(VecOfVec AllPlanes, std::vector<HBondedWaters::Molecule> Water);


	class PolygonAreas 
	{
	private:
		typedef  std::vector<double> VecDouble;

		double AreaQuad(double L);
		double AreaPentagon(double L);
		double AreaHex(double L);

	public:
		VecDouble CalcAreaPolygons(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Water);

	};
};