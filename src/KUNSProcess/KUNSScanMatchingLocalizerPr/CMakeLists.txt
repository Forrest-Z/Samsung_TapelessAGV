#-----------------------------------------------------------------------------------------------
# CMake file for the KUNS library:  /KuScanMatchingLocalizerPr
#
#  Run with "ccmake ." at the root directory, or use it as a template for 
#   starting your own programs
#-----------------------------------------------------------------------------------------------
SET(LibraryName KuScanMatchingLocalizerPr)
SET(PRJ_NAME "${LibraryName}")

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
PROJECT(${PRJ_NAME})

# These commands are needed by modern versions of CMake:
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# ---------------------------------------------------------------------------
# Set the output directory 
# ---------------------------------------------------------------------------
SET(LIBRARY_OUTPUT_PATH "${KUNS_LIB_DIR}")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the library target:
ADD_LIBRARY(${LibraryName}			  
									  KuScanMatchingLocalizerPr.h
							   		  KuScanMatchingLocalizerPr.cpp
									  ${ALGORITHM}/KuICP/KuICP.h
									  ${ALGORITHM}/KuICP/KuICP.cpp

									 
									  
		   ) 

SET_TARGET_PROPERTIES(
	${LibraryName} 
	PROPERTIES 
	PROJECT_LABEL "(KUNS_PROCESS_LIB) ${LibraryName}"
	DEBUG_POSTFIX "_dbg"	
					
	)

# Add special defines needed by this example, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${LibraryName} -lpthread)


# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
