#!/bin/bash
# Script to clean the tree from all compiled files.
# You can rebuild them afterwards using "build.sh".
#
# Written by Tiffany Huang, 12/14/2016
#

# Remove the dedicated output directories
cd C:\Users\mmasa\Udacity\SelfDrivingCarEngineerND\Projects\P6_KidnappedVehicleProject

rm -rf build

# We're done!
echo Cleaned up the project!
