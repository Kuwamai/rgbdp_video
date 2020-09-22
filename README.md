# rgbdp_video

## Description
RGBD映像にカメラ位置を埋め込むパッケージです

## Requirements
下記環境で動作確認しています

* Ubuntu18.04
* ROS Melodic
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* [rtabmap_ros](https://github.com/introlab/rtabmap_ros)
* [Tracking camera T265](https://www.intelrealsense.com/tracking-camera-t265/)
* [Depth Camera D435i](https://www.intelrealsense.com/depth-camera-d435i/)

## Installation
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros)をインストールします
* D435iのみを使用する場合は[SLAM with D435i](https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i)のページを参考に必要なものをインストールします
* 本リポジトリをクローン、ビルドします

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/Kuwamai/rgbdp_video.git
    $ cd ~/catkin_ws && catkin_make
    $ source ~/catkin_ws/devel/setup.bash
    ```

## Usage
* D435iのみで使用する場合は下記コマンドを入力します
    ```
    $ roslaunch rgbdp_video rgbdp_video.launch
    ```

* D435iとT265を使用する場合
    * 下記コマンドでデバイスのシリアル番号を調べます
        ```
        $ rs-enumerate-devices | grep Serial
        ```
    * 下記コマンドに取得したシリアル番号を入力
        ```
        $ roslaunch rgbdp_video rgbdp_video_t265.launch  serial_no1:=<serial number of d435> serial_no2:=<serial number of t265>
        ```

## H.264からVP8に変換
Ubuntu版UnityのVideo playerはH.264に対応していません。ローカルテスト用に下記コマンドで動画をVP8に変換します。

    ```
    $ ffmpeg -i RgbdpVideo.mp4 -r 30 -c:v libvpx RgbdpVideo.webm
    ```

## License
This repository is licensed under the MIT license, see [LICENSE](./LICENSE).