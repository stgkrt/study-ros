# はじめに
chapter2のHello worldをpythonでやってみます。

# ROS2ノード作成の流れ
C++の場合と同じ流れですが、以下の5stepでノードを作成します。
1. ワークスペースの作成
2. パッケージの作成
3. ノードの作成
4. ビルド
5. 実行


# ワークスペースの作成
まずはworkspaceとなるdirectoryを作成します。
ここでは`python_ws`という名前で次のパスのディレクトリを作成しました。(`/workspaces/study-ros/python_ws`)

# パッケージの作成
`/workspaces/study-ros/python_ws`このディレクトリにいる状態で、パッケージ作成用のコマンドを実行します。
```bash
ros2 pkg create --build-type ament_python my_python_pkg
```
> 以下のように出力され、パッケージの構成が作成されます。 
> ```
> going to create a new package
> package name: my_python_pkg
> destination directory: /workspaces/study-ros/python_ws
> package format: 3
> version: 0.0.0
> description: TODO: Package description
> maintainer: ['root <taro.stst@gmail.com>']
> licenses: ['TODO: License declaration']
> build type: ament_python
> dependencies: []
> creating folder ./my_python_pkg
> creating ./my_python_pkg/package.xml
> creating source folder
> creating folder ./my_python_pkg/my_python_pkg
> creating ./my_python_pkg/setup.py
> creating ./my_python_pkg/setup.cfg
> creating folder ./my_python_pkg/resource
> creating ./my_python_pkg/resource/my_python_pkg
> creating ./my_python_pkg/my_python_pkg/__init__.py
> creating folder ./my_python_pkg/test
> creating ./my_python_pkg/test/test_copyright.py
> creating ./my_python_pkg/test/test_flake8.py
> creating ./my_python_pkg/test/test_pep257.py
> ```

# ノードの作成(プログラムの準備)
以下のように標準出力をするだけのプログラムを準備してみます。
```python
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("hello")
    node.get_logger().info("Hello, ROS2 world!")

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
```

# ビルド
コードが準備できたらビルドします。
`/workspaces/study-ros/python_ws`にいる状態で、ビルドコマンドを実行します。
```bash
colcon build
```

> 次のように表示されたらビルド完了です。
> ```
> Starting >>> my_python_pkg
> Finished <<< my_python_pkg [2.54s]          
> 
> Summary: 1 package finished [6.16s]
> ```

シェルを開く度に読み込んでくれるよう次のコマンドを実行しておきましょう。
```bash
echo "source /workspaces/study-ros/python_ws/install/setup.bash" >> ~/.bashrc
```

# 動作確認
それでは次のコマンドを実行して、動作確認してみましょう。

package.xmlを次のように変更します。
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_python_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="taro.stst@gmail.com">root</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

```diff
+ <exec_depend>rclpy</exec_depend>
+ <exec_depend>std_msgs</exec_depend>
```

setup.pyに以下のように変更します。
```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        "my_python_pkg.hello"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='taro.stst@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello=my_python_pkg.hello:main',
        ],
    },
)
```

```diff
+ py_modules=[
+     "my_python_pkg.hello"
+ ],
+ entry_points={
+     'console_scripts': [
+         'hello=my_python_pkg.hello:main',
+     ],
+ },
```

```bash
ros2 run my_python_pkg hello
```

> 次のように標準出力されたら成功です。 
> ```
> [INFO] [1719056866.984911637] [hello]: Hello, ROS2 world!
> ```

# おわりに
これでpythonでもROSのノードを作って標準出力することができました。