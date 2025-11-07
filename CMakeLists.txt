cmake_minimum_required(VERSION 3.10)
project(UndoFatigue)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add your executable (ONLY your source file)
add_executable(UndoFatigue backend/Nearest_Path_Finder.cpp)

# Include directories for header-only libraries
target_include_directories(UndoFatigue PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/backend
        ${CMAKE_CURRENT_SOURCE_DIR}/backend/cpp-httplib
        ${CMAKE_CURRENT_SOURCE_DIR}/backend/json/single_include
)

# Windows specific settings
if(WIN32)
    target_compile_definitions(UndoFatigue PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(UndoFatigue PRIVATE ws2_32)
endif()

# Find and link threads
find_package(Threads REQUIRED)
target_link_libraries(UndoFatigue PRIVATE Threads::Threads)

# Copy CSV files to build directory
file(COPY
        ${CMAKE_CURRENT_SOURCE_DIR}/backend/uf_edges.csv
        ${CMAKE_CURRENT_SOURCE_DIR}/backend/uf_nodes.csv
        ${CMAKE_CURRENT_SOURCE_DIR}/backend/uf_places.csv
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)