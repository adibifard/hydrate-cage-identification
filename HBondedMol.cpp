// This header file contain the classes and functions that seek to find the hydrogen-bonded water moelcules
#include <string>
#include <vector>
#include <algorithm>
#include <list>
#include <math.h>
#include "Auxilaryfun.h"

#include "HBondedMol.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Construct FindHbondedWaterMolecules function
void HBondedWaters::FindHbondedWaterMolecules(std::vector<std::string> gro_list, int lbStep, int upStep,
                                              std::string water_Substr, double dH, double thetaH,
                                              std::vector<HBondedWaters::Molecule> &Water,
                                              std::vector<std::vector<int> > &HbondNeigh) {
    int nWat = 0;
    int j = 0;
    int atom_incr = 1;
    for (int i = lbStep + 2; i < upStep - 2; i += atom_incr) {
        // Split the line into a list
        std::vector<std::string> tmpLine = lineToListOfStrings(gro_list[i]);

        if (tmpLine[0].find(water_Substr) != -1) {
            // its a water molecule
            nWat += 1;
            HBondedWaters::Molecule tmpMol;
            // Get O1 position
            tmpMol.ro = {std::stod(tmpLine[3]), std::stod(tmpLine[4]), std::stod(tmpLine[5])};
            // Get H1 position
            std::vector<std::string> l2 = lineToListOfStrings(gro_list[i + 1]);
            tmpMol.rh1 = {std::stod(l2[3]), std::stod(l2[4]), std::stod(l2[5])};
            // Get H2 position
            std::vector<std::string> l3 = lineToListOfStrings(gro_list[i + 2]);
            tmpMol.rh2 = {std::stod(l3[3]), std::stod(l3[4]), std::stod(l3[5])};

            // Get water lable(id)
            tmpMol.WaterID = tmpLine[0];

            Water.push_back(tmpMol);
        }
    }
    //Water.erase(Water.begin(), Water.begin() + 200);
    // Find the list of the H-bonded Molecules
    /*std::vector<std::vector<int>> HbondNeigh;*/
    for (int i = 0; i < Water.size(); i += 1) {
        std::vector<int> tmpindexOfMolecules;
        for (int j = 0; j < Water.size(); j += 1) {
            // Determine the distance between molecules(the distance between the oxygen atoms of the molecules)
            double do1o2 = (Water[i].ro - Water[j].ro).norm();
            // Torsion angle calculator
            //double DIHangle = TorsionAngle(Water[i], Water[j]);

            if (do1o2 <= dH) // The water molecules are hydrogen-bonded
            {
                tmpindexOfMolecules.push_back(j);
            }
        }
        HbondNeigh.push_back(tmpindexOfMolecules);
    }
}

// Torsion Angle calcluator method
double HBondedWaters::TorsionAngle(HBondedWaters::Molecule Mol1, HBondedWaters::Molecule Mol2) {
    double d1 = (Mol1.ro - Mol2.rh1).norm();
    double d2 = (Mol1.ro - Mol2.rh2).norm();
    double d3 = (Mol2.ro - Mol1.rh1).norm();
    double d4 = (Mol2.ro - Mol1.rh2).norm();
    vector<double> Mol1_Hfar;
    vector<double> Mol2_Hfar;
    // Determine which hydrogen atom is involved in the hydrogen bond
    if (d1 > d2) // that means h2 is the outermost hydrogen
    {
        Mol2_Hfar = Mol2.rh1;
    } else {
        Mol2_Hfar = Mol2.rh2;
    }

    if (d3 > d4) // that means h3 is the outermost hydrogen
    {
        Mol1_Hfar = Mol1.rh1;
    } else {
        Mol1_Hfar = Mol1.rh2;
    }

    vector<double> u1 = Mol1.ro - Mol1_Hfar;
    vector<double> u2 = Mol2.ro - Mol1.ro;
    vector<double> u3 = Mol2_Hfar - Mol2.ro;

    vector<double> n1 = u1 * u2;
    vector<double> n2 = u2 * u3;
    double x = u2.DotProduct(n1 * n2);

    double y = u2.norm() * n1.DotProduct(n2);

    double thetaRad = atan2(y, x); // theta in radians
    double thetaDeg = thetaRad * 180 / M_PI;
    //theta = TorsionAngle(Water[i].o, wi_Hfar, Water[j].o, wj_Hfar);
    return thetaDeg;
}
