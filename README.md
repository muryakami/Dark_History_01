3D-recognition
====

## Overview
点群データやSTLデータを基に3次元物体認識を行うプログラム

## Description
今回はSTLデータを用いた3次元物体認識．
STLデータを点群に変換した後，3次元特徴点を抽出し，物体認識を行う．
扱うSTLデータはA,B,Cの3種．
AとAの一致率，AとBの一致率，AとCの一致率を
.\outputfile\accuracy001.txtに出力する．
プログラムにコマンド等は存在せず，実行するだけでよい．

このプログラムは標準ライブラリ以外に
PCL(Point Cloud Library)
を使用している．

## Author

[ymurakami0509](https://github.com/ymurakami0509)