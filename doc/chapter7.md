# はじめに
これまでの振り返りとLauchファイルについてです。


# メインループ
chapter4では、`Ctrl+C`を押すまで終わらないノードを作りました。  
`rclcpp`ライブラリをうまく使うことが重要であることがわかりましたね。

# パブリッシャーとサブスクライバー
ROSでは複数のノードを協調させたアプリケーションを実現しています。  
メッセージを送るノードをパブリッシャー、メッセージを受け取るノードをサブスクライバーと呼びます。  
chapter5ではpub/subの簡単なプログラムを書いてみましたね。  

# 亀を制御する
chapter6ではデモパッケージ`turtlesim`の亀を動かしてみました。  
トピックを介してメッセージを送る時、それをどのようなデータで扱うかも確認しました。  
ここでは亀を動かしましたが、適切なタイミングで適切なメッセージを送ることで、様々な動作が実現可能になりましたね。  

# Lauchファイル
これまで「複数のノードを一斉に同時起動する」ということをしていませんでした。  
これをLauchファイルを使うと実現することができます。  
複数のノードが連携する実用的なアプリケーションを作るときに便利なので試してみましょう。
作り方は[Lauch Tutorials](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)の[Creating a launch file](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)が参考になります。  
Launchファイルはpython, XML, yamlで記述する方法があるようですが、ここではpythonで書いてみましょう。  

## 準備
`/workspaces/study-ros/src/my_first_package/launch`のディレクトリを作成して、`pubsub_launch.py`ファイルを作成します。  
プログラムは以下のように書いてみましょう。
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
        package='my_first_package',
        executable='my_publisher'
    )
    node2 = Node(
        package='my_first_package',
        executable='my_subscriber'
    )
    return LaunchDescription([
        node1,
        node2
    ])
```
それではディレクトリを`/workspaces/study-ros/src/my_first_package/launch`に移動して、次のコマンドを実行します。
```bash
ros2 launch pubsub_launch.py
```
> 以下のように出力され、pub/subが動いていることがわかります。
> ```
> [INFO] [launch]: All log files can be found below /root/.ros/log/2024-06-22-00-14-21-583922-b811bec10261-19885
> [INFO] [launch]: Default logging verbosity is set to INFO
> [INFO] [my_publisher-1]: process started with pid [19890]
> [INFO] [my_subscriber-2]: process started with pid [19892]
> [my_subscriber-2] [INFO] [1719015261.683156085] [my_subscriber]: Hello, world 0
> [my_subscriber-2] [INFO] [1719015262.683590993] [my_subscriber]: Hello, world 1
> [my_subscriber-2] [INFO] [1719015263.683551205] [my_subscriber]: Hello, world 2
> ```
今回使用した方法以外でも、C++とpythonでlaunchさせる方法があります。  
方法については、[Launching/monitoring multiple node with Launch](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html)に書いてあります。  
この方法で`launch`ディレクトリにいなくても`ros2 launch`できるようになります。  

# おわりに
今回は振り返りを行いました。  
さらにlaunchを使って複数のノードを同時実行する方法についても確認しました。