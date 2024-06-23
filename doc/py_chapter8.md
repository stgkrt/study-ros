# はじめに
ここでは、Nodeをclassで書いてみます。  
ここで書いたコードは[公式のexamples](https://github.com/ros2/examples/tree/rolling)にあるものそのままです。  
- publisherの[プログラム](https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py)
- subscriberの[プログラム](https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py)

# プログラムの準備
公式そのままですが、以下のプログラムです。  

```python
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    try:
        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
```

```python
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.get_logger().info(f"I heard: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    try:
        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
```

> 2つのターミナルを開いてそれぞれ実行すると以下のような標準出力されます。
> ``` 
> [INFO] [1719140686.477419988] [minimal_publisher]: Publishing: Hello World: 0
> [INFO] [1719140686.960985389] [minimal_publisher]: Publishing: Hello World: 1
> [INFO] [1719140687.460952590] [minimal_publisher]: Publishing: Hello World: 2
> [INFO] [1719140687.960933190] [minimal_publisher]: Publishing: Hello World: 3
> [INFO] [1719140688.461599191] [minimal_publisher]: Publishing: Hello World: 4
> ```
> ```
> [INFO] [1719140686.477328488] [minimal_subscriber]: I heard: Hello World: 0
> [INFO] [1719140686.961346089] [minimal_subscriber]: I heard: Hello World: 1
> [INFO] [1719140687.461308790] [minimal_subscriber]: I heard: Hello World: 2
> [INFO] [1719140687.961349190] [minimal_subscriber]: I heard: Hello World: 3
> [INFO] [1719140688.462128691] [minimal_subscriber]: I heard: Hello World: 4
> ```

# さいごに
Nodeをclassで書いてみました。  
実用的にはこういったclassをカスタマイズしていくことになりそうですね。  
chapter9のGazeboはプログラム言語は関係ないのでこれでおわりになります。  
おつかれさまでした～！