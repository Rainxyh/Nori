cmake_minimum_required (VERSION 3.8)
project(nori)

add_subdirectory(ext ext_build)

include_directories(
  # Nori include files
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # tinyformat string formatting library
  ${TFM_INCLUDE_DIR}
  # Eigen linear algebra library
  SYSTEM ${EIGEN_INCLUDE_DIR}
  # OpenEXR high dynamic range bitmap library
  SYSTEM ${OPENEXR_INCLUDE_DIRS}
  # Intel Thread Building Blocks
  SYSTEM ${TBB_INCLUDE_DIR}
  # Pseudorandom number generator
  ${PCG32_INCLUDE_DIR}
  # PugiXML parser
  ${PUGIXML_INCLUDE_DIR}
  # Helper functions for statistical hypothesis tests
  ${HYPOTHESIS_INCLUDE_DIR}
  # GLFW library for OpenGL context creation
  SYSTEM ${GLFW_INCLUDE_DIR}
  # GLEW library for accessing OpenGL functions
  SYSTEM ${GLEW_INCLUDE_DIR}
  # NanoVG drawing library
  SYSTEM ${NANOVG_INCLUDE_DIR}
  # NanoGUI user interface library
  SYSTEM ${NANOGUI_INCLUDE_DIR}
  SYSTEM ${NANOGUI_EXTRA_INCS}
  # Portable filesystem API
  SYSTEM ${FILESYSTEM_INCLUDE_DIR}
  # STB Image Write
  SYSTEM ${STB_IMAGE_WRITE_INCLUDE_DIR}
)

# The following lines build the main executable. If you add a source
# code file to Nori, be sure to include it in this list.
add_executable(nori

  # Header files
  include/nori/accel.h
  include/nori/bbox.h
  include/nori/bitmap.h
  include/nori/block.h
  include/nori/bsdf.h
  include/nori/bbugbox.h
  include/nori/bsphere.h
  include/nori/bstructure.h
  include/nori/bvh.h
  include/nori/camera.h
  include/nori/color.h
  include/nori/common.h
  include/nori/dpdf.h
  include/nori/emitter.h
  include/nori/frame.h
  include/nori/gui.h
  include/nori/texture.h
  include/nori/integrator.h
  include/nori/mesh.h
  include/nori/object.h
  include/nori/parser.h
  include/nori/proplist.h
  include/nori/ray.h
  include/nori/rfilter.h
  include/nori/sampler.h
  include/nori/scene.h
  include/nori/stb_image.h
  include/nori/timer.h
  include/nori/transform.h
  include/nori/vector.h
  include/nori/warp.h
  
  # BSDF
  src/bsdf/dielectric.cpp
  src/bsdf/diffuse.cpp
  src/bsdf/microfacet.cpp
  src/bsdf/mirror.cpp
  
  # Emitters
  src/emitter/arealight.cpp
  src/emitter/disklight.cpp
  #src/emitter/environmentlight.cpp
  src/emitter/pointlight.cpp
  
  # Integrators
   # direct
    src/integrator/direct/direct_ems.cpp
    src/integrator/direct/direct_mats.cpp
    src/integrator/direct/direct_mis.cpp
    src/integrator/direct/direct.cpp
   # path
    src/integrator/path/path_ems.cpp
    src/integrator/path/path_mats.cpp
    #src/integrator/path/path_mis_ite.cpp
    src/integrator/path/path_mis.cpp
    src/integrator/path/whitted.cpp
   # toy
    src/integrator/toy/ambient_occlusion.cpp
    src/integrator/toy/average_visibility.cpp
    src/integrator/toy/normals.cpp
    src/integrator/toy/nprintegrator.cpp
    src/integrator/toy/simple.cpp
   # volume
    

  # Source code files
  src/accel.cpp
  src/bitmap.cpp
  src/block.cpp
  src/bvh.cpp
  src/chi2test.cpp
  src/common.cpp
  src/gui.cpp
  src/independent.cpp
  src/main.cpp
  src/mesh.cpp
  src/texture_image.cpp
  src/wavefrontobj.cpp
  src/object.cpp
  src/parser.cpp
  src/perspective.cpp
  src/proplist.cpp
  src/rfilter.cpp
  src/scene.cpp
  src/ttest.cpp
  src/warp.cpp
)

add_definitions(${NANOGUI_EXTRA_DEFS})

# The following lines build the warping test application
add_executable(warptest
  include/nori/warp.h
  src/warp.cpp
  src/warptest.cpp
  src/bsdf/microfacet.cpp
  src/object.cpp
  src/proplist.cpp
  src/common.cpp
)

# The following lines build the tonemapper
# add_executable(tonemapper
#         src/bitmap.cpp
#         src/common.cpp
#         src/hdrToLdr.cpp)

if (WIN32)
  target_link_libraries(nori tbb_static pugixml IlmImf nanogui ${NANOGUI_EXTRA_LIBS} zlibstatic)
else()
  target_link_libraries(nori tbb_static pugixml IlmImf nanogui ${NANOGUI_EXTRA_LIBS})
endif()

target_link_libraries(warptest tbb_static nanogui ${NANOGUI_EXTRA_LIBS})
# target_link_libraries(tonemapper IlmImf)

# Force colored output for the ninja generator
if (CMAKE_GENERATOR STREQUAL "Ninja")
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcolor-diagnostics")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
  elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
  endif()
endif()

target_compile_features(warptest PRIVATE cxx_std_17)
target_compile_features(nori PRIVATE cxx_std_17)
# target_compile_features(tonemapper PRIVATE cxx_std_17)

# vim: set et ts=2 sw=2 ft=cmake nospell:
