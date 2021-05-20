## **ros_lib**

### 概要

Arduino上でrosと連携する為のライブラリ。
lexxauto_msgsの更新など、Arduinoとの通信で使用するメッセージ形式に更新がある場合、ros_libの再生成が必要となる。

### 生成方法
はじめにLexxAutoディレクトリにて`$ source devel/setup.bash`を行いLexxAuto内のmsgを参照できるようにする。

その上で、
`$ rosrun rosserial_arduino make_libraries.py .`
を行えば良い。

（.bashrcに別のworkspaceのdevel/setup.bashがsourceされていたり、同じターミナル上で上記source 前に別のworkspace のdevel/setup.bashをsourceしていたりすると、余計なものも生成されてしまうことに留意）