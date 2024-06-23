# はじめに
ここではchapter6の亀の制御をpythonで行います。  
topicの確認方法などはchapter6を見てください。  

# プログラム
それでは亀を制御するプログラムを書いていきます。  
プログラムのパスは次のようにしてみました。`/workspaces/study-ros/python_ws/my_python_pkg/my_python_pkg/my_turtle_controller.py`
publisherのプログラムとcppで書いたものを双方参考に組み合わせて書いてみました。  

```python
import rclpy
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("turtle_controller")
    publisher = node.create_publisher(Twist, "/turtle1/cmd_vel", 1)

    msg = Twist()
    timer_period = 1.0
    def timer_callback():
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        node.get_logger().info('Publishing linear x:"%s"' % msg.linear.x)
        node.get_logger().info('Publishing angular z:"%s"' % msg.angular.z)
        publisher.publish(msg)

    while rclpy.ok():
        timer = node.create_timer(timer_period, timer_callback)
        timer
        rclpy.spin(node)
    
    rclpy.shutdown()

if __name__=="__main__":
    main()
```

# 動作確認
それでは`setup.py`をこれまで同様に更新して、ビルドし動作確認してみましょう。  
```diff
py_modules=[
    "my_python_pkg.hello",
    "my_python_pkg.hello_loop",
    "my_python_pkg.my_publisher",
    "my_python_pkg.my_subscriber",
+    "my_python_pkg.my_turtle_controller",
],
entry_points={
    'console_scripts': [
        'hello=my_python_pkg.hello:main',
        'hello_loop=my_python_pkg.hello_loop:main',
        'my_publisher=my_python_pkg.my_publisher:main',
        'my_subscriber=my_python_pkg.my_subscriber:main',
+        "my_turtle_controller=my_python_pkg.my_turtle_controller:main"
    ],
},
```
2つのターミナルを開いてそれぞれ次のコマンドを実行します。

```bash
ros2 run turtlesim turtlesim_node
```

```bash
ros2 run my_python_pkg my_turtle_controller
```

亀が円を描くように移動してくれれば成功です。


# おわりに
ここでは亀の制御をpythonで書いてみました。既存のトピックを介してpublishした結果を反映させることができましたね。