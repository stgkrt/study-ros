# 2.Hello World
パッケージ作成の[公式ページ](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)があります。
git cloneしているとsrcがすでにあるので削除してから開始すると良いかもしれません。

```sh
mkdir src
cd src
ros2 pkg create --build-type ament_cmake my_first_package
```
> 以下の出力とともにパッケージが生成されます。
> ```
> going to create a new package
> package name: my_first_package
> destination directory: /workspaces/study-ros/src
> package format: 3
> version: 0.0.0
> description: TODO: Package description
> maintainer: ['root <taro.stst@gmai.l.com>']
> licenses: ['TODO: License declaration']
> build type: ament_cmake
> dependencies: []
> creating folder ./my_first_package
> creating ./my_first_package/package.xml
> creating source and include folder
> creating folder ./my_first_package/src
> creating folder ./my_first_package/include/my_first_package
> creating ./my_first_package/CMakeLists.txt
> ```

先ほどbuildしたsrcのdirectoryにいる状態でbuildコマンドを実行します。
```bash
colcon build
```
> buildが成功すると、以下のように出力されます。
> ```
> Starting >>> my_first_package
> Finished <<< my_first_package [1.52s]                  
> 
> Summary: 1 package finished [1.85s]
> ```

それでは自分でhello worldを書いてみましょう。
/workspaces/study-ros/src/my_first_package/srcのdirectoryにhello.cppを作成し、以下のコードを書き込みます。

```cpp
# include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello");
    RCLCPP_INFO(node->get_logger(), "Hello, ROS2 world!");

    rclcpp::shutdown();
    return 0;
}
```

/workspaces/study-ros/src/my_first_package/CMakeLists.txtの内容に追記します。

```CMake
cmake_minimum_required(VERSION 3.5)
project(my_first_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello src/hello.cpp)
ament_target_dependencies(hello rclcpp)
install(TARGETS hello DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

追記したのは以下の部分です。
> ```diff
> + find_package(rclcpp REQUIRED)
> + 
> + add_executable(hello src/hello.cpp)
> + ament_target_dependencies(hello rclcpp)
> + install(TARGETS hello DESTINATION lib/${PROJECT_NAME})
> ```

自分の作ったpackageをinstallします。
```bash
. install/setup.bash
```
次にROS2で自分の作ったpackageであるmy_first_packageのhelloを実行します。
```sh
ros2 run my_first_package hello
```
> 以下のように書いたコードのHello worldが表示されます。
> ```
> [INFO] [1718952982.607302300] [hello]: Hello, ROS2 world!
> ```