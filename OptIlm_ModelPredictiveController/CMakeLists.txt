cmake_minimum_required(VERSION 3.10.0)

project(OptIlm_ModelPredictiveController)

include_directories("/usr/ipopt31210_all/include/coin")
include_directories("/usr/ipopt31210_all/include/coin/Thirdparty")
include_directories("/home/aadc/thirdparty_libs/eigen335")


set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -O3")


set (SOURCEFILES   	
	stdafx.h 
	OptIlm_ModelPredictiveController.h
	OptIlm_ModelPredictiveController.cpp


	Nmpc_Algorithm/SOP_NMPC.cpp
        Nmpc_Algorithm/SOP_NMPC.h
        Nmpc_Algorithm/SOP_nlp.cpp
        Nmpc_Algorithm/SOP_nlp.h
        Nmpc_Algorithm/MPC_weighting_parameter.h
    	Nmpc_Algorithm/ExtendedKalmanFilter/rungekutta.h
    	Nmpc_Algorithm/ExtendedKalmanFilter/rungekutta.cpp
    	Nmpc_Algorithm/ExtendedKalmanFilter/ExtendedKalmanFilter.h
    	Nmpc_Algorithm/ExtendedKalmanFilter/ExtendedKalmanFilter.cpp)


adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC -L/usr/ipopt31210_all/lib -lipopt -lm  -ldl)


#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER OptIlmDriveX)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER OptIlmDriveX)
