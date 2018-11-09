/*----------------------------------------------------------------------------
 ADOL-C -- Automatic Differentiation by Overloading in C++
 File:     cpp_example.cpp
 Revision: $$
 Contents: example for class myADOLC_sparseNPL for interfacing with Ipopt
 
 Copyright (c) Andrea Walther
   
 This file is part of ADOL-C. This software is provided as open source.
 Any use, reproduction, or distribution of the software constitutes 
 recipient's acceptance of the terms of the accompanying license file.
 
 This code is based on the file corresponding file cpp_example.cpp contained 
 in the Ipopt package with the authors:  Carl Laird, Andreas Waechter   
----------------------------------------------------------------------------*/

//*************************************************************************
//
//
//         Nothing has to be changed in this file !!
//
//
//*************************************************************************

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "ADOL-C_sparseNLP.hpp"

using namespace Ipopt;

int main(int argv, char* argc[])
{
  // Create an instance of your nlp...
  SmartPtr<TNLP> myadolc_nlp = new MyADOLC_sparseNLP();

  // Create an instance of the IpoptApplication
  SmartPtr<IpoptApplication> app = new IpoptApplication();

  // Initialize the IpoptApplication and process the options
  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    printf("\n\n*** Error during initialization!\n");
    return (int) status;
  }

  status = app->OptimizeTNLP(myadolc_nlp);

  if (status == Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Index iter_count = app->Statistics()->IterationCount();
    printf("\n\n*** The problem solved in %d iterations!\n", iter_count);

    Number final_obj = app->Statistics()->FinalObjective();
    printf("\n\n*** The final value of the objective function is %e.\n", final_obj);
  }

  return (int) status;
}
