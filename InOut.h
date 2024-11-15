#pragma once
#include <vector>
#include <string>
#include "Auxilaryfun.h"
#include <fstream>
#include <cstdlib> 
#include <iostream>
#include <cmath>
#include <iomanip>

#include "HBondedMol.h"
#include "AlgCage.h"

#include <span>


class InOutfunctions
{
private:
	typedef std::vector<std::vector<int> >	VecOfVec;
	typedef std::vector<vector<double> > VecOfVecDouble;
	typedef std::vector<std::string>    VecString;

	// Get the number of digits
	int GetNumDigits(int N);

public:

	

	VecString ReadInputfile(std::string dir, std::string fileName);
	void CageCentroidToGRO(std::string filename, VecOfVecDouble CageCentroids, VecString CageTypes, std::string BoxDimAngle);

	void CageMoleculesToGRO(std::string filename, std::vector<CageDetectionAlgorithms::Cage> Cage, VecOfVec AllPlanes, std::vector<HBondedWaters::Molecule> Water, std::string BoxDimAngle, std::vector<HBondedWaters::Molecule>& WatersInCage);

	void WaterMoleculesCageLabelled(std::string filename, std::vector<HBondedWaters::Molecule>& WaterMol, std::vector<CageDetectionAlgorithms::Cage> Cages, VecOfVec AllPlanes, std::string BoxDimAngle);
	
	void WriteToExcel(std::string filename, std::span<std::vector<CageDetectionAlgorithms::Cage> >  data, std::string colName);

	std::vector<std::vector<std::string> > ExtractDatafromInputStringVector(std::vector<std::string> InputData, std::vector<std::string> MolcIDs);

	void CageAssociatedWaterMoleculesTrajectory(std::string Outputfilename, std::vector<std::string> InputData, std::vector<CageDetectionAlgorithms::Cage> Cages, VecOfVec AllPlanes, std::vector<HBondedWaters::Molecule> WaterMol);


	std::vector<std::string> WriteInStringVector(std::string BoxDimAngle, VecString CageTypes, VecOfVecDouble CageCentroids);


};