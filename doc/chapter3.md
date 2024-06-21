# アプリケーションの作り方
1. パッケージの作成
2. ソースファイルの作成
3. ビルド設定ファイルの編集
4. ビルド
5. 実行

[Tutorials](https://docs.ros.org/en/foxy/Tutorials.html)からC++, pythonでソースを記述するのが基本らしいです。今回はC++メインで行います。

自作パッケージを実行できるように.bashrcに以下のコードを追加しておきます。
```bash
echo "source /workspaces/study-ros/src/install/setup.bash" >> ~/.bashrc
```

# 基礎知識
ROSの動作を理解する上でNodeの理解が重要になります。公式の[Understanding nodes](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)を見て、概要を理解していきましょう。

ROSでは複数のノードを協調させることでアプリケーションを実現します。
公式でもServiceやTopicを経由して複数のノードが協調している様子が見て取れます。
![Understanding ROS2 nodes](./images/Nodes-TopicandService.gif)

## トピックとメッセージ
あるノードが別のノードへデータを送るためのパスをトピックと呼び、トピックを介して送られるデータをメッセージと呼びます。

## パブリッシャーとサブスクライバー
- メッセージの送り手：パブリッシャー
- メッセージの受けて：サブスクライバー

このような送受信の形式を「パブリッシュ・スブスクライブモデル」、略して「パブサブモデル」と呼びます。
![RO2 publish subscribe model](./images/Topic-MultiplePublisherandMultipleSubscriber.gif)


## プログラミングスタイル
[Code style and language versions](https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)に詳細がありますが、なるべくこの規約に沿ってプログラミングするようにしましょう。
