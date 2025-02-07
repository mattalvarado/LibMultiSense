#
# MultiSense - Makefile
#

add_subdirectory(Wire)

if (${BUILD_LEGACY_API})
    add_subdirectory(Legacy)
else ()
    add_subdirectory(LibMultiSense)
endif()

if (${MULTISENSE_BUILD_UTILITIES})
    add_subdirectory(Utilities)
endif ()

if (NOT BUILD_LEGACY_API AND BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()

