# create an executable
ADD_EXECUTABLE (pp_project
	${CS225A_COMMON_SOURCE}
	pp_project.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (pp_project
	${CS225A_COMMON_LIBRARIES}
)

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/pp_project)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa_pp.urdf world_pp.urdf DESTINATION ${APP_RESOURCE_DIR})
