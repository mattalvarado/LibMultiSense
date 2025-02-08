set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)

pybind11_add_module(libmultisense MODULE bindings.cc)

target_link_libraries(
    libmultisense
    PRIVATE
    MultiSense)

if (USE_OPENCV)
  if(OpenCV_FOUND)
    target_compile_definitions(libmultisense PRIVATE HAVE_OPENCV)

    target_include_directories(libmultisense PUBLIC ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(libmultisense PRIVATE ${OpenCV_LIBS})
  endif()
endif()


set_target_properties(libmultisense PROPERTIES libmultisense _core)
