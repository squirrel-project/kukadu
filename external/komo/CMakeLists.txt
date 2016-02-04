project(komo)
cmake_minimum_required(VERSION 2.6)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_definitions(-fPIC)
add_definitions(-DMT_LAPACK)
add_definitions(-DMT_Core)
add_definitions(-DMT_GTKGL)
add_definitions(-DMT_GL)
add_definitions(-DMT_GTK)
add_definitions(-DMT_QHULL)
add_definitions(-DMT_Algo)
add_definitions(-DMT_Gui)
add_definitions(-DMT_extern_SWIFT)
add_definitions(-DSWIFT_DECOMP)
add_definitions(-DNO_CYCLE_DETECTION)

find_package(Boost 1.54 REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(GTK REQUIRED gtkglext-1.0)

include_directories(
    share/src
    share/include
    share/src/extern/SWIFT_decomposer/RAPID
    ${Qt5Gui_INCLUDE_DIRS}
    ${QT5Core_INCLUDE_DIRS}
    ${QT5CoreWidgets_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${GTK_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(share/src/Algo ALGOSRC)
AUX_SOURCE_DIRECTORY(share/src/Core CORESRC)
AUX_SOURCE_DIRECTORY(share/src/extern/SWIFT SWIFTSRC)
AUX_SOURCE_DIRECTORY(share/src/extern/SWIFT_decomposer SWIFTDECOSRC)
AUX_SOURCE_DIRECTORY(share/src/extern/SWIFT_decomposer/RAPID RAPIDSRC)
AUX_SOURCE_DIRECTORY(share/src/Gui GUISRC)
AUX_SOURCE_DIRECTORY(share/src/Motion MOTIONSRC)
AUX_SOURCE_DIRECTORY(share/src/Optim OTPIMSRC)
AUX_SOURCE_DIRECTORY(share/src/Ors ORSSRC)

list(REMOVE_ITEM CORESRC "share/src/Core/array_instantiate.cxx")

list(REMOVE_ITEM GUISRC "share/src/Gui/opengl_void.cxx")
list(REMOVE_ITEM GUISRC "share/src/Gui/opengl_qt.cxx")
list(REMOVE_ITEM GUISRC "share/src/Gui/opengl_qt_moc.cxx")
list(REMOVE_ITEM GUISRC "share/src/Gui/opengl_freeglut.cxx")
list(REMOVE_ITEM GUISRC "share/src/Gui/opengl_gtk.cxx")
list(REMOVE_ITEM GUISRC "share/src/Gui/opengl_fltk.cxx")

list(REMOVE_ITEM SWIFTDECOSRC "share/src/extern/SWIFT_decomposer/main.cxx")

list(REMOVE_ITEM ORSSRC "share/src/Ors/ors_tutorial.cxx")

add_library(core ${CORESRC})
add_library(algo ${ALGOSRC})
add_library(swift ${SWIFTSRC} ${SWIFTDECOSRC} ${RAPIDSRC})
add_library(gui ${GUISRC})
add_library(motion ${MOTIONSRC})
add_library(optim ${OTPIMSRC})
add_library(ors ${ORSSRC})

add_library(komo share/src/Komo/komo.cpp)

target_link_libraries(core rt lapack blas)
target_link_libraries(algo core rt lapack blas ann)
target_link_libraries(gui rt glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull)
target_link_libraries(ors core algo gui swift rt glut GLU GL X11)
target_link_libraries(optim core rt)
target_link_libraries(motion core ors optim algo rt)

target_link_libraries(komo core algo gui ors optim motion rt lapack blas ann glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull)
