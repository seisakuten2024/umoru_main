# 実行手順
```
# 以下はbashrcに書いたので制作展のときは実行せずに大丈夫
source /opt/ros/noetic/setup.bash
source ~/seisakuten_ws/devel/setup.bash
rossetmaster
rossetip
```

## 1. arm関連を起動
```
roslaunch umoru_arm umoru_larm.launch
roslaunch umoru_arm umoru_rarm.launch
```
このとき、一つずつ実行する。腕にある4つのインフレータブルに空気が入って、十分入ったらenterを押すフェーズがある。放置すると空気が入りすぎるので注意。

## 2. カメラとオーディオ
```
roslaunch umoru_main umoru_camera_and_audio.launch
```
D405の起動、顔認識、speech recognitionを立ち上げている
## 3. rosserial
```
roslaunch umoru_main umoru_rosserial.launch
```
両目、インフレータブルにつながるarduinoとのrosserial通信を始めている
## 4. main
```
roslaunch umoru_main umoru_heart_and_main.launch
```
heartの起動、mainのpythonファイルの実行を行う。
このときheartのwindowが現れるとターミナルが見えなくなる。Windows key + tabでターミナルを選択するように。

# 注意点
USBハブはバスパワーを2つつかう。
バスパワー①には、rarm, larm, スピーカを指す。
バスパワー②には、arduino, right-eye, left-eye, cameraを指す。
※カメラとrarmとlarmを同じハブに指したりすると電源供給が足りなくなってハグの瞬間にカメラが切れるので注意。


# トラブルシューティング
## カメラが動かない
rvizを確認する
/camera/color/face_pose_estimation/output/vizのimage(compressed)を選択して、顔認識されているか、カメラが見えているかを確認する
。
USB抜き差ししても、カメラのroslaunchを立ち上げ直してもだめなとき
```
rostopic pub /if_camera_doesnot_work std_msgs/Bool "data: false"
```
これで顔が現れたのと同じ処理が走り出す。

## 腕が動かない
```
ls /dev/umoru*
```
をして /dev/umoru_rarm, /dev/umoru_larmがあるかをチェックする。
もしあるのに、roslaunchでエラーが出ている or 腕が動かない場合は、
腕の電源ケーブルを抜き差ししてkondoのブザー音がなったらもう一度launchを立ち上げ直す

## 瞬きが両目でずれている
umoru_rosserial.launchを立ち上げ直す

