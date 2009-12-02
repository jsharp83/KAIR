PROJECT(SICK_TEST)
 
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
 
# --------------------------------------------------------------------------
#   The list of "libs" which can be included is:
#     - core: The main library. It will be included by default if the 
#              assignment to MRPT_USE_LIBS is omitted.
#     - hmtslam        --> libmrpt-hmtslam
#     - hwdrivers      --> libmrpt-hwdrivers
#     - reactivenav    --> libmrpt-reactivenav
#     - aria           --> libmrpt-aria
#     - xsens          --> libmrpt-xsens
# --------------------------------------------------------------------------
SET(MRPT_USE_LIBS core hwdrivers)
FIND_PACKAGE( MRPT REQUIRED )
 
# Declare the target (an executable)
ADD_EXECUTABLE(SICK_TEST 
	SickTest.cpp SickLMS200.cpp
	)
 
TARGET_LINK_LIBRARIES(SICK_TEST ${MRPT_LIBS})
