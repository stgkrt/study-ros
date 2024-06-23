# はじめに
ここでは、メインループありの処理をHello worldに追加します。  
rclpyのsleepさせる関数がNodeクラスのcallback methodになってそうで、ほかの書き方がわからなかったのでsleepはrosではないものを使っています。

# プログラムの準備
それでは以下のようなコードを`/workspaces/study-ros/python_ws/my_python_pkg/my_python_pkg/hello_loop.py`に追加します。  
基本的にあ第二章のhello worldにループを追加しただけのものです。

```python
import rclpy
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("hello_loop")

    # keyboard 操作があるまでループ
    while rclpy.ok():
        node.get_logger().info("Hello, ROS2 world!")
        #rosのsleepがわからなかった。クラスならcallbackがあるっぽい
        sleep(1)

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
```

# ビルドと動作確認
py_modulesとentry_pointsにそれぞれ追加します。
```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        "my_python_pkg.hello",
        "my_python_pkg.hello_loop",
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
            'hello_loop=my_python_pkg.hello_loop:main',
        ],
    },
)
```

```diff
py_modules=[
    "my_python_pkg.hello",
+     "my_python_pkg.hello_loop",
],
entry_points={
    'console_scripts': [
        'hello=my_python_pkg.hello:main',
+        'hello_loop=my_python_pkg.hello_loop:main',
    ],
},
```
`/workspaces/study-ros/python_ws`にいる状態で`colcon build`でビルドし、以下のコマンドで実行します。
```bash
ros2 run my_python_pkg hello_loop
```
> このように1secごとに標準出力されれば成功です。
> ```
> [INFO] [1719114208.495501933] [hello_loop]: Hello, ROS2 world!
> [INFO] [1719114209.498904120] [hello_loop]: Hello, ROS2 world!
> [INFO] [1719114210.500664607] [hello_loop]: Hello, ROS2 world!
> [INFO] [1719114211.502377493] [hello_loop]: Hello, ROS2 world!
> ```


# さいごに
今回はchapter4のループ処理をpythonで実行しました。同時にパッケージ追加部分なども触ってみることができましたね。