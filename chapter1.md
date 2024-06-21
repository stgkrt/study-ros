# 1. 環境構築
公式docの[installページ](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)があります。DockerFileに書き込んでいるのでdevcontainerで開くとそのまま実行できるはずです。


## 動作確認
ターミナルにhello-world表示する。通信？なのか次のコマンドで読み出すことができるようになっている。
```bash
ros2 run demo_nodes_cpp talker
```
> 出力
> ```
> [INFO] [1718938336.009226400] [talker]: Publishing: 'Hello World: 1'
> [INFO] [1718938337.009315200] [talker]: Publishing: 'Hello World: 2'
> [INFO] [1718938338.009243100] [talker]: Publishing: 'Hello World: 3'
> [INFO] [1718938339.009241500] [talker]: Publishing: 'Hello World: 4'
> ...
> ```
読み出し側のコマンド
```bash
ros2 run demo_nodes_py listener
```
> 出力
> ```
> [INFO] [1718938338.012768600] [listener]: I heard: [Hello World: 3]
> [INFO] [1718938339.012901200] [listener]: I heard: [Hello World: 4]
> [INFO] [1718938340.012824500] [listener]: I heard: [Hello World: 5]
> [INFO] [1718938341.011854000] [listener]: I heard: [Hello World: 6]
> ...
> ```

