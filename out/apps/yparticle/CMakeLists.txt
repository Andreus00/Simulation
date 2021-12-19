add_executable(yparticle  yparticle.cpp)

set_target_properties(yparticle  PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yparticle  PRIVATE ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yparticle  yocto yocto_particle)

if(YOCTO_OPENGL)
target_link_libraries(yparticle  yocto_gui)
endif(YOCTO_OPENGL)
