# はじめに
ここではlaunchファイルを作って、複数のノードを実行してみます。

# プログラムの作成
`/workspaces/study-ros/python_ws/my_python_pkg/launch`のディレクトリに`pubsub_launch.py`を作成します。
中身は次のようにしてみました。
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
        package='my_python_pkg',
        executable='my_publisher'
    )
    node2 = Node(
        package='my_python_pkg',
        executable='my_subscriber'
    )
    return LaunchDescription([
        node1,
        node2
    ])
```

# 動作確認
`/workspaces/study-ros/python_ws/my_python_pkg/launch`のディレクトリにいる状態で次のコマンドを実行します。
```bash
ros2 launch pubsub_launch.py
```
> 次のように出力されると成功です。
> ```
> [INFO] [launch]: All log files can be found below /root/.ros/log/2024-06-23-10-53-08-462236-7d9761ed3164-74022
> [INFO] [launch]: Default logging verbosity is set to INFO
> [INFO] [my_publisher-1]: process started with pid [74064]
> [INFO] [my_subscriber-2]: process started with pid [74066]
> [my_publisher-1] [INFO] [1719139991.393115971] [my_publisher]: Publishing: "Hello World: 0"
> [my_subscriber-2] [INFO] [1719139991.403826171] [my_subscriber]: I heard: "Hello World: 0"
> [my_publisher-1] [INFO] [1719139992.378367574] [my_publisher]: Publishing: "Hello World: 1"
> [my_subscriber-2] [INFO] [1719139992.379303974] [my_subscriber]: I heard: "Hello World: 1"
> [my_publisher-1] [INFO] [1719139993.378355077] [my_publisher]: Publishing: "Hello World: 2"
> [my_subscriber-2] [INFO] [1719139993.379271177] [my_subscriber]: I heard: "Hello World: 2"
> ```


# おわりに
ここではlaunchファイルで複数のノードを実行しました。
