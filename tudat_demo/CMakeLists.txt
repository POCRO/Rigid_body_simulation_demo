# 设置 CMake 最低版本
cmake_minimum_required(VERSION 3.10)

# 定义项目名称和版本
project(tudat_demo VERSION 0.1.0 LANGUAGES C CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加源文件目录
add_executable(tudat_demo src/main.cpp)

# Tudat 库路径
# 假设 Tudat 的安装路径是 ../tudatBundle/build
set(TUDAT_DIR /home/yy/tudatBundle)
include_directories(${TUDAT_DIR})

include_directories(${TUDAT_DIR}/tudat)
# include_directories(${TUDAT_DIR}/tudat/sofa)
include_directories(${TUDAT_DIR}/tudat/Tudat/Astrodynamics/Aerodynamics)

target_include_directories(tudat_demo PRIVATE
    ${TUDAT_DIR}/tudat/Tudat/Astrodynamics/Aerodynamics
    ${TUDAT_DIR}/tudat/Tudat/SimulationSetup
    # 添加其他必要的路径
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 引入 Tudat 的库和头文件
# target_include_directories(tudat_demo PRIVATE ${TUDAT_DIR}/include)
target_link_directories(tudat_demo PRIVATE ${TUDAT_DIR}/tudat/lib)

file(GLOB TUDAT_LIBRARIES ${TUDAT_DIR}/tudat/lib/*.a)

target_link_libraries(tudat_demo PRIVATE ${TUDAT_LIBRARIES})

# target_link_libraries(tudat_demo PRIVATE ${TUDAT_BUNDLE}/tudat/libtudat.a)

# 链接 Tudat 所需的库
target_link_libraries(tudat_demo PRIVATE
    tudat_simulation
    tudat_basic_mathematics
    tudat_aerodynamics
    tudat_astrodynamics
    tudat_input_output
    tudat_numerical_integrators
    tudat_gravitation
    tudat_reference_frames
    tudat_propagators
    tudat_system_models
)
