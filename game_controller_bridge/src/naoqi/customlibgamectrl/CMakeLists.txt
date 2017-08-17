cmake_minimum_required(VERSION 2.8)
project(customlibgamectrl)

find_package(qibuild)

include_directories(../../include)

set(_srcs GameCtrl.cpp UdpComm.h UdpComm.cpp)

qi_create_lib(gamectrl SHARED ${_srcs} SUBFOLDER naoqi)

qi_use_lib(gamectrl ALCOMMON BOOST)
