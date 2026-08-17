# This file is intentionally not a standalone CMake project.
# It is included from the top-level STARK CMakeLists.txt after Python has
# already been found exactly once.
#
# Do not call find_package(Python ...) here. Doing so can poison the CMake
# cache with a different interpreter than the one selected by the user.

if(NOT STARK_BUILD_PYTHON_BINDINGS)
    return()
endif()

if(NOT Python_FOUND OR NOT TARGET Python::Module)
    message(FATAL_ERROR
        "pystark/cpp was added before Python was configured.\n"
        "Configure from the top-level STARK CMakeLists.txt and let the parent "
        "find Python with components Interpreter and Development.Module."
    )
endif()

include(FetchContent)
set(STARK_NANOBIND_GIT_TAG "v2.12.0"
    CACHE STRING "nanobind git tag used to build pystark"
)

FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG        ${STARK_NANOBIND_GIT_TAG}
    GIT_SHALLOW    TRUE
)

# nanobind caches NB_SUFFIX (the .pyd/.so extension tag) in CMakeCache.txt.
# When the active Python interpreter changes (e.g. cp312 → cp311) the cached
# value becomes stale because nanobind uses "set(... CACHE INTERNAL)" without
# FORCE, so it never overwrites an existing entry.  We query the interpreter
# here and force-write the cache entry so that the generated library always
# carries the correct tag regardless of previous configure runs.
execute_process(
    COMMAND "${Python_EXECUTABLE}" "-c"
        "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
    OUTPUT_VARIABLE _pystark_ext_suffix
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(NB_SUFFIX  "${_pystark_ext_suffix}" CACHE INTERNAL "" FORCE)
set(NB_SUFFIX_S "${_pystark_ext_suffix}" CACHE INTERNAL "" FORCE)

FetchContent_MakeAvailable(nanobind)

# Source files
set(CORE_FILES
    core/pystark_core.h
    core/pystark_Logger.cpp
    core/pystark_Console.cpp
    core/pystark_Settings.cpp
)

set(UTILS_FILES
    utils/pystark_utils.cpp
    utils/pystark_utils.h
)

set(MODELS_FILES
    # General
    models/pystark_models.h
    models/pystark_Simulation.cpp

    # Deformables
    models/deformables/pystark_PointDynamics.cpp
    models/deformables/pystark_PointSetHandler.cpp
    models/deformables/pystark_Deformables.cpp
    models/deformables/pystark_DeformablesMeshOutput.cpp

    models/deformables/point/pystark_EnergyLumpedInertia.cpp
    models/deformables/point/pystark_EnergyPrescribedPositions.cpp
    models/deformables/line/pystark_EnergySegmentStrain.cpp
    models/deformables/surface/pystark_EnergyTriangleStrain.cpp
    models/deformables/surface/pystark_EnergyDiscreteShells.cpp
    models/deformables/volume/pystark_EnergyTetStrain.cpp

    # Rigid bodies
    models/rigidbodies/pystark_RigidBodyDynamics.cpp
    models/rigidbodies/pystark_RigidBodies.cpp
    models/rigidbodies/pystark_RigidBodyHandler.cpp
    models/rigidbodies/pystark_RigidBodiesMeshOutput.cpp
    models/rigidbodies/pystark_rigidbody_constraints_ui.cpp

    # Interactions
    models/interactions/pystark_Interactions.cpp
    models/interactions/pystark_EnergyFrictionalContact.cpp
    models/interactions/pystark_EnergyAttachments.cpp

    # Presets
    models/presets/pystark_Presets.cpp
    models/presets/pystark_DeformablesPresets.cpp
    models/presets/pystark_deformables_preset_types.cpp
    models/presets/pystark_RigidBodyPresets.cpp
)

set(PYSTARK_FILES
    ${CORE_FILES}
    ${MODELS_FILES}
    ${UTILS_FILES}

    nanobind_stark_include_all.h
    pystark_main.cpp
)

nanobind_add_module(pystark ${PYSTARK_FILES})

# Output the extension module directly into the Python package directory so that
# `import pystark` from the source tree picks up the latest build.
set_target_properties(pystark PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../pystark"
)

target_link_libraries(pystark PRIVATE stark)

target_include_directories(pystark PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../stark/include"
)

# Local development convenience:
# build/pystark target writes the extension into pystark/pystark/
# so `PYTHONPATH=/path/to/stark/pystark python -c "import pystark"` works.
set_target_properties(pystark PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../pystark"
)

# Packaging:
# scikit-build-core builds the wheel from the CMake install tree, not from
# LIBRARY_OUTPUT_DIRECTORY. Therefore the extension and pure Python files must
# be installed explicitly into the Python package directory.
install(TARGETS pystark
    LIBRARY DESTINATION pystark
    RUNTIME DESTINATION pystark
    ARCHIVE DESTINATION pystark
)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../pystark/__init__.py"
    DESTINATION pystark
)

# Misc
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${PYSTARK_FILES})
message(STATUS "STARK: pystark uses Python executable: ${Python_EXECUTABLE}")
message(STATUS "STARK: pystark uses nanobind tag:     ${STARK_NANOBIND_GIT_TAG}")
