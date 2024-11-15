#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <list>

#include "Auxilaryfun.h"

class HBondedWaters
{
private:
	// Declare a Vector(Vector) type
	//typedef  std::vector<std::vector<int>> VecOfVec;
	
	
	

public:

	
	// Define a class for molecules
	class Molecule
	{
	public:
		vector<double> ro, rh1, rh2;
		std::string WaterID;
		std::vector<std::string>  CageLable;
		void set_values(vector<double> a, vector<double> b, vector<double> c, std::string id, std::vector<std::string> cglbl)
		{
			ro = a;
			rh1 = b;
			rh2 = c;
			WaterID = id;
			CageLable= cglbl;
		}

	};

	// A function to find the dihedral angle between two molecules
	double TorsionAngle(Molecule M1, Molecule M2);

	// A function to find the H-bonded water molecules
	void FindHbondedWaterMolecules(std::vector<std::string> gro_list, int lbStep, int upStep, std::string water_Substr, double  dH,double thetaH, std::vector<Molecule>& Water, std::vector<std::vector<int> >& HbondNeigh);


	
};