PROJECT(Enose_Simul)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED maps obs )

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

FILE(GLOB list_srcs GasMapModel*.cpp GasMapModel*.h)

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CEnoseSimulApp.h CEnoseSimulApp.cpp
	${list_srcs}
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )
