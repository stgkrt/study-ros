# はじめに
ここでは、chapter5のpub/subをやってみます。

# パブリッシャー
それでは次のようにpublisherのプログラムを書いてみます。  
公式のexampleにある[簡単なpublisher](https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_local_function.py)の例を参考に書いてみました。

```python
import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("my_publisher")
    publisher = node.create_publisher(String, "greeting", 10)

    msg = String()
    i = 0
    def timer_callback():
        nonlocal i
        msg.data = 'Hello World: %d' %i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
    
    while rclpy.ok():
        timer_period = 1.0  # seconds
        timer = node.create_timer(timer_period, timer_callback)
        timer  # Quiet flake8 warnings about unused variable
        rclpy.spin(node)
    
    rclpy.shutdown()

if __name__=="__main__":
    main()
```

# サブスクライバー
それではサブスクライバーのプログラムを書いていきます。  
サブスクライバーも[公式の例](https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_lambda.py)を参考に書いてみました。  


```python
import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("my_subscriber")
    subscription = node.create_subscription(
        String,
        "greeting",
        lambda msg: node.get_logger().info('I heard: "%s"' % msg.data),
        10
    )
    subscription
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
```

# 動作確認
ビルドのためにsetup.pyにpy_modulesとentrypoitsを追加します。
```diff
py_modules=[
    "my_python_pkg.hello",
    "my_python_pkg.hello_loop",
+    "my_python_pkg.my_publisher",
+    "my_python_pkg.my_subscriber",
],
'console_scripts': [
    'hello=my_python_pkg.hello:main',
    'hello_loop=my_python_pkg.hello_loop:main',
+    'my_publisher=my_python_pkg.my_publisher:main',
+    'my_subscriber=my_python_pkg.my_subscriber:main',
],
```
それでは`/workspaces/study-ros/python_ws`にいる状態で`colcon build`して、2つのターミナルでつぎのコマンドを実行して動作確認してみましょう。

```bash
ros2 run my_python_pkg my_subscriber 
```

```bash
ros2 run my_python_pkg my_publisher
```
> 次のように表示されれば成功です。
> subscriber側のターミナルでは、以下のように標準出力されます。
> ```
> [INFO] [1719128070.274723870] [my_subscriber]: I heard: "Hello World: 0"
> [INFO] [1719128071.061999973] [my_subscriber]: I heard: "Hello World: 1"
> [INFO] [1719128072.062100577] [my_subscriber]: I heard: "Hello World: 2"
> [INFO] [1719128073.057555881] [my_subscriber]: I heard: "Hello World: 3"
> ```
> publisher側のターミナルでは、以下のように標準出力されます。
> ```
> [INFO] [1719128070.218916270] [my_publisher]: Publishing: "Hello World: 0"
> [INFO] [1719128071.058637773] [my_publisher]: Publishing: "Hello World: 1"
> [INFO] [1719128072.058669377] [my_publisher]: Publishing: "Hello World: 2"
> [INFO] [1719128073.056669581] [my_publisher]: Publishing: "Hello World: 3"
> ```


# おわりに
ここではpub/subを簡単に書いてみました。