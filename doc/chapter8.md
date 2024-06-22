# はじめに
コールバック関数を使用したサブスクライバーは別にして、これまで作ったプログラムはいずれも`main()`関数にすべての処理がありました。  
`main()`関数はプログラムのエントリーポイントなので、なるべく短い方が良いと思います。  
そのため今回は、ノードをクラスにして`main()`関数ではそのインスタンス化だけを行う、ということをやってみましょう。

# パブリッシャーとサブスクライバー
chapter5で作ったpub/subのプログラムをクラスを使った形に書き直しみます。

## パブリッシャーのヘッダファイル
まずはパブリッシャーをクラスにしてみます。
ヘッダファイルを次のように書いてみました。クラス名は`SimplePublisher`です。
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher();

private:
  void timer_callback();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```
1つ目は、ノードをクラスにするため`rclcpp::Node`クラスを継承しているということです。この継承は必須になります。  
2つ目は、メンバ変数の宣言で具体的な型を使っているということです。この理由は `auto`による型推論が使えないためです。  
また、周期的に呼び出すためのコールバック関数`timer_callback()`を宣言しています。  

## パブリッシャーのソースファイル
次にヘッダファイルに対応するソースファイルを書いてみます。
```cpp
#include "my_first_package/simple_publisher.hpp"

SimplePublisher::SimplePublisher() : Node("simple_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("greeting", 1);

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, std::bind(&SimplePublisher::timer_callback, this));
}

void SimplePublisher::timer_callback()
{
  static int count = 0;
  auto msg = std_msgs::msg::String();
  msg.data = "Hello, world " + std::to_string(count++);
  publisher_->publish(msg);
}
```
はじめに`SimplePublisher`のコンストラクタ初期化子で基底クラス Node のコンストラクタを呼び出し、ノード名を指定する必要があります。  
`publisher_`の初期化は以前と同様ですね。  
続く`timer_`の初期化では`create_wall_timer()`関数を使い、一定間隔で実行されるコールバック関数を登録しています。これで`timer_callback()`関数が1秒毎に呼ばれるようになります。  
`using namespace std::chrono_literals;`では時間単位のリテラル、例えば`1s`や`500ms`が使えるようになります。

## パブリッシャーのインスタンス化
パブリッシャーをインスタンス化するソースファイルも必要です。これは次のようになります。
```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_first_package/simple_publisher.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## サブスクライバーのヘッダファイル
次にサブスクライバーのヘッダファイルを書いていきます。クラス名は`SimpleSubscriber`です。
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber();

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```
トピックをサブスクライブした時に呼び出すためのコールバック関数`topic_callback()`と、サブスクライブを登録するための`subscription_`を宣言しています。

## サブスクライバーのソースファイル
ヘッダファイルに対応するソースファイルは次のようになります。
```cpp
#include "my_first_package/simple_subscriber.hpp"

SimpleSubscriber::SimpleSubscriber() : Node("simple_subscriber")
{
  using std::placeholders::_1;
  subscription_ = this->create_subscription<std_msgs::msg::String>("greeting", 1,
    std::bind(&SimpleSubscriber::topic_callback, this, _1));
}

void SimpleSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
}
```
パブリッシャーと同じく、コンストラクタ初期化子で基底クラス Node のコンストラクタを呼び出し、ノード名を指定します。  
次の`subscription_`の初期化では`std::bind()`や`using std::placeholders::_1;`といった見慣れないものが登場します。  
実はパブリッシャーで`timer_`を初期化する時も使っていますが、`std::bind()`は引数を束縛した関数オブジェクトを返す関数です。そして、引数の`_1`は`bind()`のプレースホルダーオブジェクトです。  
今のところはクラスを使う際に必要な「おまじない」と思っておいて良いかも知れません。

## サブスクライバーのインスタンス化
サブスクライバーをインスタンス化するソースファイルは次のようになります。
```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_first_package/simple_subscriber.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

# 動作確認
ビルド設定ファイルに次の行を追加します。
```diff
+ unset(NODES)
+ set(NODES simple_publisher;simple_subscriber)
+ foreach(target IN LISTS NODES)
+   add_executable(${target}_node src/${target}_main.cpp src/${target}.cpp)
+   ament_target_dependencies(${target}_node rclcpp std_msgs)
+   install(TARGETS ${target}_node DESTINATION lib/${PROJECT_NAME})
+   target_include_directories(${target}_node
+     PUBLIC
+       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
+       $<INSTALL_INTERFACE:include>
+   )
+ endforeach()
```
それでは、`/workspaces/study-ros/src`のディレクトリで`colcon build`して、ターミナルを2つ用意して次のコマンドを実行します。
```bash
ros2 run my_first_package simple_subscriber_node 
```

```bash
ros2 run my_first_package simple_publisher_node
```
> subscriberのターミナルで次のような標準出力になります。
> ```
> [INFO] [1719017146.598067438] [simple_subscriber]: Hello, world 0
> [INFO] [1719017147.597158888] [simple_subscriber]: Hello, world 1
> [INFO] [1719017148.597043739] [simple_subscriber]: Hello, world 2
> [INFO] [1719017149.597116389] [simple_subscriber]: Hello, world 3
> [INFO] [1719017150.597193940] [simple_subscriber]: Hello, world 4
> ```

# おわりに
今回はノードをクラスにしてみました。
C++の知識不足でインスタンス化する部分などはあまりわかっていませんが、動作確認まで実行することができました。