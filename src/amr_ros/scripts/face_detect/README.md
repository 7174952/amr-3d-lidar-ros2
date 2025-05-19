# プログラム簡易概要

### Prerequisites
Install prerequisites with:
```
conda create -n face_recog
conda activate face_recog
conda install ***
```
*** are refereed to requirements.txt.

### Step.1 ユーザー登録
カメラでユーザー画像を撮影するプログラム．撮影画像は，"./TARGETIMG/<ユーザー名>"に保存される．
```
$ python capture_target.py -o <ユーザー名>
```
カメラウインドウ表示後，キーボード"p"を押すことで"./TARGETIMG/<ユーザー名>"に画像を保存．
撮影を終了する場合は，キーボード"q"を押す．

### Step.2 カメラでのリアルタイム顔認識
参照したいユーザーは，ディレクトリ"./TARGETIMG/"の直下にある各サブディレクトリから読み込まれる．

e.g., 参照ディレクトリのデータ構造
```
./TARGETIMG
 |
 ├── classA
 |     |
 |     ├── img1.png
 |     ├── img2.png
 |     :
 |
 ├── classB
 |     |
 :     ├── img1.png
 :     ├── img2.png
       :
```

参照ディレクトリを準備後，下記コマンドにてカメラ画像からの顔認識を実行可能である．
```
$ python inference_cam.py
```

### サンプルコード
事前に用意してあるサンプル画像で実行結果を表示する方法．
参照ディレクトリは"./SAMPLE/"，推論時に用いるテスト画像は"./TESTIMG/test.jpg"である．
```
$ python inference_img.py
```
![overview](./result/result.png)  
