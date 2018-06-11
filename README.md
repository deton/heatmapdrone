# 室温ヒートマップ作成のためドローンを天井灯にそって飛行させる

オフィスの全体空調では、室温が場所によって、暑すぎたり寒すぎたりする場合があり、
[17個程度のセンサを置いて測定](https://github.com/deton/roomtemper#付録grafanaでフロアのヒートマップもどき)をしているのですが、
もっと細かい室温ヒートマップを作成してみたいと思い、
ドローンを自律飛行させれば良さそう思って作成。

ライントレーサ同様に、天井の蛍光灯の明るさをトレースして飛行。
(カメラ等を使ったSLAM(Simultaneous Localization and Mapping)でなく)

![写真](../img/onmambo.jpg)

### 部品
#### ハード
* Parrot Mambo Fly。
  カメラ等を後から載せられる形になっているので、
  上にセンサ等を載せられそうだったので。
  SDKもあるし、node.jsやpythonから使う例もあるので。
* [BLE Nano V2](https://www.switch-science.com/catalog/3445/)
* [VL53L0X](https://www.switch-science.com/catalog/2894/) 2個。前方の壁と天井からの距離測定用(I2C)
* [ADT7410](http://akizukidenshi.com/catalog/g/gM-06675/)。温度センサ(I2C)
* NJL7502L 2個、10kΩ抵抗 2個。左右上方の明るさ測定用。
* CR2032 コイン電池ホルダ
* 細ピンヘッダ(秋月)。
  丸ピンソケットと合わせて使いたかったので。
* ユニバーサル基板。はさみで切れる薄いもの。

配線は行きあたりばったりで作成。

I2Cプルアップ抵抗はADT7410に実装されているものを使用。

![回路図](../img/schematic.png)

#### ソフト
BLE関係は、BLE Nano v2のArduino用ライブラリを使用。

+ 当初、BLE central & peripheralとして作って、
peripheralに対してスマホから接続してデータ取得する形にしようかと思っていた。
が、面倒になったので、データ取得は着陸後Serial接続して行う形に変更。
後からRX/TX接続を追加したのでジャンパ接続。

### 環境
* 広さ: 12m * 40m
* 2本ずつ17列の蛍光灯。1列に7つから4つ。また切れたままになっている所も。

```
 +----------------------+  壁
 |                      |
 | == == == == == == == |  蛍光灯
 |                      |
 | == == == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 | ==    == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 | == == == == == ==    |
 |                      |
 | == == == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 |    == == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 |    == == == == == == |
 |                      |
 | == == == == == == == |
 |                      |
 | ==    ==    ==    == |
 |                      |
 | == == == == == == == |
 |                      |
 | ==    == == == == == |
 |                      |
 +----------------------+
```

## known issue
* 蛍光灯の列を外れて右に流れる。
  1列目をたどるのはなんとかなっても、2列目をたどりきれない。
* 接続完了までに70秒程度かかる

## 苦労した点
* 自律飛行がなかなか安定しない
  * 手で操縦しても、右に流れる場合あり。
    何回か墜落してから、右に流れやすくなった気がしなくもない。
    ホバー状態でも、1m以上流れて移動する場合あり。
    床面がごちゃごちゃしていると発生しやすい?
  * スマホで操縦しても、同じ場所に来た時に流れる。ホバリング状態でも。
    少し風がある? または机があるのが原因?
  * スマホで操縦した感じ、ホバリングをはさみながら少しずつ移動させると、
    流れやすい。
    少しずつ連続的に移動させると比較的なめらかに飛行する。
    が、短時間で飛行コマンドを送るようにしても流れる場合あり。
  * 基坂を貼る位置を少し右寄りや左寄りにしてみたが、明確な変化は見られない。
    進まなくなる場合に、前寄りにしてみたが、明確な変化なし。
  * 横に流れたことの検出が難しい。
    また、前に進んでいるつもりで進んでいなかったり。
    天井灯が切れている場所なのか、右に流れたのかの検出難。
  * 当初は明るさセンシング用に[Adafruit VCNL4010](https://www.switch-science.com/catalog/2640/)を1個使用。
    横に流れるため左右に明るさセンサNJL7502Lを設ける形に変更。
  * 回転や左右移動方向変更の際は、waitを入れないと、直前に移動していた方向に移動し続ける。
  * 飛行コマンドの送信後に飛行完了まで数百msはかかるので、
    短時間に連続してコマンドを送ると、別のコマンドを送ってもすぐには反応しない。

* 物にぶつかって墜落した時に、BLE Nano v2が起動しなくなる現象発生。
  直接電源に接続すると起動するので、どこかの半田付けが外れたかと思って、
  テスタで導通を確認するも問題なさそう。
  テスタで電圧を確認すると出てなかったりする場合も。
  接触が悪くなってるのかと思って、全ての半田付けを再実施。
  * 壁にぶつかって、天井付近1.8m程度から落ちた場合、当たりどころが悪いと、
    CR2032ホルダ接続用丸ピンソケットの足が曲がって折れる。数回発生。
    墜落時はなるべく途中で外箱等で受けとめる方が良い。
  * 飛行中に顔を近付けて唇や頬を切った。手の平ならだいたい大丈夫。

### 飛行安定化対策案
+ 回転しないで後ろにもセンサを付ける。右または左にも要。ななめに付ける?
+ 90度回転はやめて180度回転にする。右または左にセンサ要。
+ 横に流れたことの検出のため、加速度等のセンサを付ける。

## 他の方法案
+ 公式SDKでの制御を試す。
  `機体上センサ+BLE Nano2 ---> Android上アプリ(+SDK) ---> 機体制御`
+ 別の機体で試してみる。墜落後に右に流れやすくなった気がするので。
+ カメラ等を使ってSLAM。蛍光灯の明るい列から外れないようにする程度?
+ tello等別のドローンで試してみる

## TODO
+ 温度センサの反応時間の測定と、それに応じた飛行速度の決定
+ 温度センサを基板にきちんと固定する

## 拡張案
+ 温度以外に、CO2濃度、電波強度等を測定してヒートマップを作成する
+ ドローン飛行による下方向の風の流れを使って、サーキュレータがわりに使用

## 参考
### Parrot Mambo制御
* https://github.com/Mechazawa/minidrone-js
* https://github.com/algolia/pdrone
* https://github.com/fetherston/npm-parrot-minidrone
* https://github.com/amymcgovern/pymambo
* https://github.com/voodootikigod/node-rolling-spider
### droneを使ってWi-Fi heatmap
* [Use Quadcopter to create a heat map of WiFi signal strength](https://diydrones.com/forum/topics/use-quadcopter-to-create-a-heat-map-of-wifi-signal-strength)
* [Bloodhound: Autonomous Radiolocation Drone](https://hackaday.io/project/25995-bloodhound-autonomous-radiolocation-drone)
