# 指定 CMake 的最低版本要求。
cmake_minimum_required(VERSION 4.1)

# 声明当前工程名称，后面可以通过 PROJECT_NAME 等变量引用。
project(server)

# 设置可执行文件输出目录为当前工程下的 bin 目录。
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 自动收集 src 目录下的所有源文件，保存到 SRC_LIST 变量中。
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)

# 把 include 目录加入头文件搜索路径。
include_directories(${PROJECT_SOURCE_DIR}/include)

# 生成服务器主程序可执行文件，使用 SRC_LIST 中的所有源文件。
add_executable(server ${SRC_LIST})

# 链接数学库，供代码中的数学函数使用。
target_link_libraries(server m)

# 启用 CTest 测试能力，后面才能注册和运行测试。
enable_testing()

# 生成单元测试可执行文件，编译测试代码和被测的 unit_conversion.c。
add_executable(test_unit_conversion
    test/test_unit_conversion.cpp
    src/unit_conversion.c
)

# 给测试目标单独指定头文件搜索路径，确保能找到项目头文件。
target_include_directories(test_unit_conversion PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

# 链接 GoogleTest 和数学库；gtest_main 提供默认 main 函数。
target_link_libraries(test_unit_conversion gtest gtest_main m)

# 注册一个 CTest 测试项，执行 test_unit_conversion 这个测试程序。
add_test(NAME UnitConversionTest COMMAND test_unit_conversion)