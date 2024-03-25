#!/bin/bash
# Environment Variables
export HUSKY_URDF_EXTRAS=$(catkin_find ncat19_husky_description urdf/husky_description.urdf.xacro  --first-only)
export HUSKY_LARGE_TOP_PLATE=1
export HUSKY_FRONT_BUMPER_EXTEND=0
export HUSKY_REAR_BUMPER_EXTEND=0
export HUSKY_USER_RAIL_ENABLED=0

#comment out the arm you are not using here
export HUSKY_KINOVA_XYZ="-0.106 0 0.006"
