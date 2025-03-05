set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)

pybind11_add_module(libmultisense MODULE bindings.cc)

target_link_libraries(
    libmultisense
    PRIVATE
    MultiSense)

if (BUILD_JSON_SERIALIZATION)
    if(nlohmann_json_FOUND)
        target_compile_definitions(libmultisense PRIVATE BUILD_JSON="True")
    endif()
endif()
