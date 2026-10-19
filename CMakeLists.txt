cmake_minimum_required(VERSION 3.28)

project(fmjpeg2k)

set (fmjpeg2k_VERSION_MAJOR 1)
set (fmjpeg2k_VERSION_MINOR 0)
set (fmjpeg2k_VERSION_BUILD 3)
set(fmjpeg2k_VERSION "${fmjpeg2k_VERSION_MAJOR}.${fmjpeg2k_VERSION_MINOR}.${fmjpeg2k_VERSION_BUILD}")

option(BUILD_SHARED_LIBS "Build fmjpeg2k shared library and link executables against it." ON)

set (CMAKE_CXX_STANDARD 20)

if(NOT BUILD_SHARED_LIBS)
add_definitions(-DOPJ_STATIC)
endif()

find_package(DCMTK REQUIRED CONFIG)
find_package(OpenJPEG REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include ${DCMTK_INCLUDE_DIRS} ${OPENJPEG_INCLUDE_DIRS})
LINK_DIRECTORIES(${FMJPEG2K}/lib)

add_definitions(-Dfmjpeg2k_EXPORTS)

set(fmjpeg2k_HEADERS
    include/fmjpeg2k/cxx_config.h
	include/fmjpeg2k/djcodecd.h
	include/fmjpeg2k/djcodece.h
	include/fmjpeg2k/djcparam.h
	include/fmjpeg2k/djdecode.h
	include/fmjpeg2k/djencode.h
	include/fmjpeg2k/djerror.h
	include/fmjpeg2k/djlsutil.h
	include/fmjpeg2k/djrparam.h
	include/fmjpeg2k/dldefine.h
	include/fmjpeg2k/memory_file.h
)

set(fmjpeg2k_SRCS
    ${fmjpeg2k_HEADERS}
	djcodecd.cc
	djcodece.cc
	djcparam.cc
	djdecode.cc
	djencode.cc
	djrparam.cc
	djutils.cc
	memory_file.cpp
)

if(WIN32)
	add_definitions(-D_BIND_TO_CURRENT_VCLIBS_VERSION=1)
endif()

ADD_EXECUTABLE(dcmcjp2k dcmcjp2k.cc)
TARGET_LINK_LIBRARIES(dcmcjp2k
	DCMTK::DCMTK
	${OPENJPEG_LIBRARIES}
	fmjpeg2k
	)

ADD_EXECUTABLE(dcmdjp2k dcmdjp2k.cc)
TARGET_LINK_LIBRARIES(dcmdjp2k
	DCMTK::DCMTK
	${OPENJPEG_LIBRARIES}
	fmjpeg2k
	)


add_library(fmjpeg2k ${fmjpeg2k_SRCS})
set(fmjpeg2k_LIBRARY_NAME fmjpeg2k)
TARGET_LINK_LIBRARIES(fmjpeg2k
	DCMTK::DCMTK
	${OPENJPEG_LIBRARIES}
	)

include(CTest)
if(BUILD_TESTING)
  # A consumer must explicitly select a compatible language mode. Test the
  # header itself, independently of DCMTK's own compiler/version checks.
  foreach(standard IN ITEMS 17 20)
    try_compile(FMJPEG2K_CXX${standard}_ACCEPTED
      SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tests/cxx_standard_probe.cc"
      NO_CACHE
      CXX_STANDARD ${standard}
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
      OUTPUT_VARIABLE FMJPEG2K_CXX${standard}_OUTPUT)
  endforeach()
  if(FMJPEG2K_CXX17_ACCEPTED OR
     NOT FMJPEG2K_CXX17_OUTPUT MATCHES "fmjpeg2k requires C[+][+]20 or newer")
    message(FATAL_ERROR "The public header must reject C++17 with the fmjpeg2k diagnostic.\n${FMJPEG2K_CXX17_OUTPUT}")
  endif()
  if(NOT FMJPEG2K_CXX20_ACCEPTED)
    message(FATAL_ERROR "The public header must accept C++20.\n${FMJPEG2K_CXX20_OUTPUT}")
  endif()
  message(STATUS "fmjpeg2k consumer language check: C++17 rejected, C++20 accepted")

  add_executable(test_decode_sample_width tests/decode_sample_width.cc memory_file.cpp)
  target_link_libraries(test_decode_sample_width PRIVATE fmjpeg2k)
  add_test(NAME decode_sample_width COMMAND test_decode_sample_width)
  add_executable(test_decode_color tests/decode_color.cc memory_file.cpp)
  target_link_libraries(test_decode_color PRIVATE fmjpeg2k)
  add_test(NAME decode_color COMMAND test_decode_color)
endif()

include(GenerateExportHeader)
generate_export_header(fmjpeg2k)
set_property(TARGET fmjpeg2k PROPERTY VERSION ${fmjpeg2k_VERSION})
set_property(TARGET fmjpeg2k PROPERTY SOVERSION 1)
set_property(TARGET fmjpeg2k PROPERTY
  INTERFACE_fmjpeg2k_MAJOR_VERSION 1)
set_property(TARGET fmjpeg2k APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING fmjpeg2k_MAJOR_VERSION
)

install(TARGETS dcmdjp2k EXPORT fmjpeg2kTargets
  RUNTIME DESTINATION bin
)

install(TARGETS dcmcjp2k EXPORT fmjpeg2kTargets
  RUNTIME DESTINATION bin
)

install(TARGETS fmjpeg2k EXPORT fmjpeg2kTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${fmjpeg2k_HEADERS}
  DESTINATION
    include/fmjpeg2k
  COMPONENT
    Devel
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/fmjpeg2k/fmjpeg2kConfigVersion.cmake"
  VERSION ${fmjpeg2k_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(TARGETS fmjpeg2k
  FILE "${CMAKE_CURRENT_BINARY_DIR}/fmjpeg2kExports.cmake"
)
configure_file( ${CMAKE_SOURCE_DIR}/cmake/fmjpeg2kConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/fmjpeg2kConfig.cmake
  @ONLY
)
#configure_file(cmake/fmjpeg2kConfig.cmake
#  "${CMAKE_CURRENT_BINARY_DIR}/fmjpeg2k/fmjpeg2kConfig.cmake"
#  COPYONLY
#)

set(ConfigPackageLocation lib/cmake/fmjpeg2k)
install(EXPORT fmjpeg2kTargets
  FILE
    fmjpeg2kTargets.cmake
  DESTINATION
    ${ConfigPackageLocation}
)


install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/fmjpeg2kConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/fmjpeg2k/fmjpeg2kConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
