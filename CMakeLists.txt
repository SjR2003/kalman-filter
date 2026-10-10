cmake_minimum_required(VERSION 3.5)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 11)

# --- Eigen ---
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# --- SDL2 ---
set(SDL2_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SDL2-2.32.10/x86_64-w64-mingw32")
set(SDL2TTF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SDL2_ttf-2.24.0/x86_64-w64-mingw32")

set(SDL2_INCLUDE    "${SDL2_ROOT}/include")
set(SDL2_LIB        "${SDL2_ROOT}/lib/libSDL2.dll.a")
set(SDL2MAIN_LIB    "${SDL2_ROOT}/lib/libSDL2main.a")

set(SDL2TTF_INCLUDE "${SDL2TTF_ROOT}/include")
set(SDL2TTF_LIB     "${SDL2TTF_ROOT}/lib/libSDL2_ttf.dll.a")

set(sources
    main.cpp
    src/simulation/display.cpp
    src/simulation/beacons.cpp
    src/simulation/simulation.cpp

    src/utils/sensors.cpp
    src/utils/utils.cpp

    src/math/kalmanfilter_capstone.cpp
)

add_executable(${PROJECT_NAME} ${sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SDL2_INCLUDE}
    ${SDL2_INCLUDE}/SDL2
    ${SDL2TTF_INCLUDE}
    ${SDL2TTF_INCLUDE}/SDL2
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    mingw32
    ${SDL2MAIN_LIB}
    ${SDL2_LIB}
    ${SDL2TTF_LIB}
    Eigen3::Eigen
)

# Copy data folder
add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/third_party/roboto $<TARGET_FILE_DIR:${PROJECT_NAME}>)

# Copy only the two DLLs that actually exist
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL2_ROOT}/bin/SDL2.dll"
        "${SDL2TTF_ROOT}/bin/SDL2_ttf.dll"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
)