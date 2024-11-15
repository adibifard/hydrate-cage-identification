#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <fstream>

#include "HBondedMol.h"
#include "Planes.h"
#include "AlgCage.h"
#include "InOut.h"

using namespace std::chrono;


int main(int argc, char *argv[]) {
    char *inputfile;
    int argInit = 1;
    while (argInit < argc) {
        const char *args = argv[argInit++];
        switch (*args) {
            case '-':
                if (strcmp(args, "-i") == 0) {
                    inputfile = argv[argInit];
                }
        }
    }

    // Declare a Vector(Vector) type
    typedef std::vector<std::vector<int> > VecOfVec;
    typedef std::vector<std::unordered_map<int, int> > VecOfUnorMap;

    // Read the .gro file
    std::string dir =
            "/Users/unconvrs/CLionProjects/crystal-cage-identification/data/cg_model/co2_hydrate/water_co2_sw_116/Tm_250K/postprocess/postprocess/gro";
    std::string fileName = "Tm250K_sliced_new_formed_hydrate_only_h2o_fr360.gro";

    std::vector<std::string> InputData;
    InOutfunctions InOutWrite;
    InputData = InOutWrite.ReadInputfile(dir, fileName);
    unsigned int numAtoms = int(std::stoi(InputData[1])); // this includes h, o, and ch4
    // Assuming that the number of atoms do not change over time
    unsigned int NlinePerStep = numAtoms + 3;

    unsigned int NtimeSteps = 2;
    if (InputData.size() > NlinePerStep) {
        // there are multiple time - steps
        NtimeSteps = int(InputData.size() / NlinePerStep);
    } else {
        NtimeSteps = 2;
    }
    // Define water identifier
    std::string water_Substr = "SOL";

    // Set Hbond criteria
    double dH = 0.35; // in nm
    double dTheta = 45; // in degrees

    // instantiate an object for the class H-bonded molecules
    HBondedWaters hbond;
    Planes pln;
    CageDetectionAlgorithms cageAlg;

    CageDetectionAlgorithms::CageIdentViaCup CageAlgCups;

    std::vector<std::string> OutputCages_5_12;

    std::vector<std::vector<CageDetectionAlgorithms::Cage> > cage5_12UniqueCagesSorted(NtimeSteps - 1);
    std::vector<std::vector<CageDetectionAlgorithms::Cage> > cage5_12_6_2UniqueCagesSorted(NtimeSteps - 1);
    std::vector<std::vector<CageDetectionAlgorithms::Cage> > cage5_12_6_3UniqueCagesSorted(NtimeSteps - 1);
    std::vector<std::vector<CageDetectionAlgorithms::Cage> > cage5_12_6_4UniqueCagesSorted(NtimeSteps - 1);

    std::vector<HBondedWaters::Molecule> Water;
    std::vector<int> AllPlaneType;
    VecOfVec AllPlanesForCage;
    // Set up csv file to write cage-frequency per time data
    std::string csv_filename = fileName + "_cage_data.csv";

    std::ofstream outFile(csv_filename);
    outFile << "Time-Step,5(12) Cages,5(12)6(2) Cages,5(12)6(3) Cages,5(12)6(4) Cages\n";

    // here the main function over time steps begins
    for (unsigned int i = 0; i < NtimeSteps - 1; i++) {
        // empty the previously filled containers
        Water.clear();
        AllPlaneType.clear();
        AllPlanesForCage.clear();

        // Parse the inputData into  another function to detect the Hbonded water moleclues
        unsigned int lbStep = NlinePerStep * i;
        unsigned int upStep = NlinePerStep * (i + 1);


        std::vector<std::vector<int> > HbondNeigh;
        hbond.FindHbondedWaterMolecules(InputData, lbStep, upStep, water_Substr, dH, dTheta, Water, HbondNeigh);

        std::string BoxDimAngleStr = InputData[upStep - 1];

        //HbondNeigh.erase(HbondNeigh.begin(), HbondNeigh.begin()+600);
        size_t numWaters = Water.size();

        VecOfVec quad = pln.FindQuadrilateral(numWaters, HbondNeigh);
        VecOfVec pent = pln.FindPentagons(numWaters, HbondNeigh);
        VecOfVec hex = pln.FindHexagonals(numWaters, HbondNeigh);

        VecOfVec AllPlanes = pln.CollectAllPlanes(quad, pent, hex, AllPlaneType);

        Planes::CheckPlaneConstraints PlaneChecker;

        VecOfVec AllPlanes_Coplanar = PlaneChecker.CheckCoplanarityConstraint(AllPlanes, Water, 0.01);
        VecOfVec AllPlanes_CoplanarRegular = PlaneChecker.CheckRegularPlaneConstraint(AllPlanes_Coplanar, Water, 0.2);

        AllPlanesForCage = AllPlanes_Coplanar;

        // Determine Areas of the planes
        Planes::PolygonAreas polygonAreaCalculator;

        std::vector<double> PlaneAreas = polygonAreaCalculator.CalcAreaPolygons(AllPlanesForCage, Water);

        std::vector<vector<double> > PlanesCentroid = pln.PlaneCentroid(AllPlanesForCage, Water);
        //VecOfUnorMap Sharedplanes=pln.FindSharingEdgePlanes(AllPlanes);

        VecOfVec Sharedplanes = pln.FindSharingEdgePlanes(AllPlanesForCage);

        /////// 5(12) cage identification
        std::vector<CageDetectionAlgorithms::Cage> cup5_6 = CageAlgCups.cup5_6(AllPlanesForCage, Sharedplanes);

        std::vector<CageDetectionAlgorithms::Cage> cup5_6UniqueUnsorted;
        std::vector<CageDetectionAlgorithms::Cage> cup5_6UniqueCagesSorted = cageAlg.RemoveDuplicatesCages(
            cup5_6, cup5_6UniqueUnsorted);

        std::vector<CageDetectionAlgorithms::Cage> cage5_12 = CageAlgCups.twoCupCagesSym(
            cup5_6UniqueCagesSorted, AllPlanesForCage, Sharedplanes, "5_12");
        std::vector<CageDetectionAlgorithms::Cage> cage5_12UniqueUnsorted;
        cage5_12UniqueCagesSorted[i] = cageAlg.RemoveDuplicatesCages(cage5_12, cage5_12UniqueUnsorted);

        /////// 5(12)6(2) cage identification
        std::vector<CageDetectionAlgorithms::Cage> cup6_1_5_6 = CageAlgCups.cup5_6_6_1(AllPlanesForCage, Sharedplanes);

        std::vector<CageDetectionAlgorithms::Cage> cup6_1_5_6UniqueUnsorted;
        std::vector<CageDetectionAlgorithms::Cage> cup6_1_5_6UniqueCagesSorted = cageAlg.RemoveDuplicatesCages(
            cup6_1_5_6, cup6_1_5_6UniqueUnsorted);

        std::vector<CageDetectionAlgorithms::Cage> cage5_12_6_2 = CageAlgCups.twoCupCagesSym(
            cup6_1_5_6UniqueCagesSorted, AllPlanesForCage, Sharedplanes, "5_12_6_2");
        std::vector<CageDetectionAlgorithms::Cage> cage5_12_6_2UniqueUnsorted;
        cage5_12_6_2UniqueCagesSorted[i] = cageAlg.RemoveDuplicatesCages(cage5_12_6_2, cage5_12_6_2UniqueUnsorted);

        ///////// 5(12)6(3) cage identification
        std::vector<CageDetectionAlgorithms::Cage> cage5_12_6_3 = CageAlgCups.ThreeCupCagesSym(
            cup6_1_5_6UniqueCagesSorted, AllPlanesForCage, Sharedplanes, "5_12_6_3");
        std::vector<CageDetectionAlgorithms::Cage> cage5_12_6_3UniqueUnsorted;
        cage5_12_6_3UniqueCagesSorted[i] = cageAlg.RemoveDuplicatesCages(cage5_12_6_3, cage5_12_6_3UniqueUnsorted);

        ///////// 5(12)6(4) cage identification
        std::vector<CageDetectionAlgorithms::Cage> cage5_12_6_4 = CageAlgCups.FourCupCagesSym(
            cup6_1_5_6UniqueCagesSorted, AllPlanesForCage, Sharedplanes, "5_12_6_4");
        std::vector<CageDetectionAlgorithms::Cage> cage5_12_6_4UniqueUnsorted;
        cage5_12_6_4UniqueCagesSorted[i] = cageAlg.RemoveDuplicatesCages(cage5_12_6_4, cage5_12_6_4UniqueUnsorted);

        std::cout << "Time-Step: " << i << "\n" << "\n";
        std::cout << "5(12) cages: " << cage5_12UniqueCagesSorted[i].size() << "\n";
        std::cout << "5(12)6(2) cages: " << cage5_12_6_2UniqueCagesSorted[i].size() << "\n";
        std::cout << "5(12)6(3) cages: " << cage5_12_6_3UniqueCagesSorted[i].size() << "\n";
        std::cout << "5(12)6(4) cages: " << cage5_12_6_4UniqueCagesSorted[i].size() << "\n";


        // Write data for current time-step to the CSV file
        outFile << i << ","
                << cage5_12UniqueCagesSorted[i].size() << ","
                << cage5_12_6_2UniqueCagesSorted[i].size() << ","
                << cage5_12_6_3UniqueCagesSorted[i].size() << ","
                << cage5_12_6_4UniqueCagesSorted[i].size() << "\n";
    } // end of the iteration over the time-steps

    // Close the csv file we opened to write the temporal cage-frequency data.
    outFile.close();

    //////////////////////////////// Track the cage-associated molecules within the original input .gro file ////////////////////////////////////////
    std::string fileNameTemp = "MainRunNodrag_rest0_rest1_frs1_8007_str10.gro";
    //std::string fileName = inputfile;
    std::vector<std::string> InputDataTemporal;
    InputDataTemporal = InOutWrite.ReadInputfile(dir, fileNameTemp);
    //std::vector<CageDetectionAlgorithms::Cage> subCages5_12_6_3{ cage5_12_6_3UniqueCagesSorted.begin(),cage5_12_6_3UniqueCagesSorted.begin() + 1 };

    InOutWrite.CageAssociatedWaterMoleculesTrajectory(
        "MainRunNoDrag_rest0_rest1_CageAssociatedMols_5_12Cages_str10.gro", InputDataTemporal,
        cage5_12UniqueCagesSorted[cage5_12UniqueCagesSorted.size() - 1], AllPlanesForCage, Water);
    InOutWrite.CageAssociatedWaterMoleculesTrajectory(
        "MainRunNoDrag_rest0_rest1_CageAssociatedMols_5_12_6_2Cages_str10.gro", InputDataTemporal,
        cage5_12_6_2UniqueCagesSorted[cage5_12_6_2UniqueCagesSorted.size() - 1], AllPlanesForCage, Water);
    InOutWrite.CageAssociatedWaterMoleculesTrajectory(
        "MainRunNoDrag_rest0_rest1_CageAssociatedMols_5_12_6_3Cages_str10.gro", InputDataTemporal,
        cage5_12_6_3UniqueCagesSorted[cage5_12_6_3UniqueCagesSorted.size() - 1], AllPlanesForCage, Water);
    InOutWrite.CageAssociatedWaterMoleculesTrajectory(
        "MainRunNoDrag_rest0_rest1_CageAssociatedMols_5_12_6_4Cages_str10.gro", InputDataTemporal,
        cage5_12_6_4UniqueCagesSorted[cage5_12_6_4UniqueCagesSorted.size() - 1], AllPlanesForCage, Water);
}
