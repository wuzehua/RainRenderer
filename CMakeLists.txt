cmake_minimum_required(VERSION 3.15)
project(RainRenderer)

set(CMAKE_CXX_STANDARD 20)


aux_source_directory(src/main MAIN_SRC)
aux_source_directory(src/math MATH_SRC)
aux_source_directory(src/media MEDIA_SRC)
aux_source_directory(src/utils UTILS_SRC)

set(SRCS
        ${MAIN_SRC}
        ${MATH_SRC}
        ${MEDIA_SRC}
        ${UTILS_SRC})


include_directories(${PROJECT_SOURCE_DIR}/include)


add_executable(RainRenderer ${SRCS})