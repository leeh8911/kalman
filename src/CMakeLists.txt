file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} STATIC
${SRC_FILES})

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} PUBLIC 
${CMAKE_SOURCE_DIR}
${CMAKE_SOURCE_DIR}/src
${CMAKE_SOURCE_DIR}/includes
)