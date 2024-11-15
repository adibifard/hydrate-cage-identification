
#include "AlgCage.h"



void    CageDetectionAlgorithms::GeneralCageAlg::CalculateCageCentroid(std::vector<CageDetectionAlgorithms::Cage>& Cages, std::vector<vector<double>> PlanesCentroids, std::vector<double> PlanesAreas)
{

	for (int i = 0; i < Cages.size(); i++) 
	{
		vector<double> s = { 0,0,0 };
		double Asum = 0;
		std::vector<int>::iterator itr;
		for (int j = 0; j < Cages[i].planes.size(); j++)
		{
			s = s +  PlanesCentroids[Cages[i].planes[j]] * PlanesAreas[Cages[i].planes[j]] ;
			Asum += PlanesAreas[Cages[i].planes[j]];
		}
		Cages[i].centroid = s / Asum;
	}
}