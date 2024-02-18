# EMU8088_57Q<br>
<br>
PIC18F47Q43はDIP40ピンで、電脳伝説さんがそれを使用したSBCであるEMUZ80を<br>
公開されたのがきっかけで、その後コアな愛好者によって、色々な拡張や<br>
新しいSBCが公開されています。<br>
<br>
PIC18F57Q43は、QFP48ピンのパッケージに収められており、仕様は47Qと変わりませんが、<br>
I/Oが8ピン多いのが特徴です。<br>
<br>
このPIC18F57Q43を使った8088/用のSBCを作成し、CPM-86を移植、公開しています<br>
SBCについての詳細と、Gerberデータは、以下を参照してください。<br>
<br>
＜EMU8088_57Q_CPM86＞<br>
https://github.com/akih-san/EMU8088_57Q_CPM86
<br>
EMU8088_57Q SBCボード
![EMU8088 1](photo/P1020470.JPG)

# MS-DOS V2.11の移植<br>
このSBCにMS-DOS V2.11を移植しました。8088/V20が4.9MHz、または8MHzで動作します。<br>

MS-DOS Ver2.11 Opening Photo
![EMU8088 2](photo/MSDOS_opening.png)

# ファームウェア
@hanyazouさんが作成したZ80で動作しているCP/M-80のFWをベースに、EMU8088_57Q0用の<br>
FWとして動作するように修正を加え、MS-DOSをインプリメントしました。<br>
<br>
MS-DOSのデバイスドライバの本体はPICのFW側で実現し、新設計となります。<br>
DISK I/OとFatFs、及びSPIについては、CPM-86と同様に、ほぼ未修整で使用しています。<br>
<br>
# IO.SYSの開発環境
IO.SYSの開発は、マイクロソフト社マクロアセンブラ（MASM）V5.0Aを使用しています。<br>
開発には、セルフで開発できる環境か、MS-DOSエミュレーターが動作する環境が必要です。<br>
<br>
EMU8088のIO.SYSの開発には、Windows上で動くエミュレーターとして有名なmsdos playerを<br>
使用させていただきました。開発者のtakeda氏に深く感謝致します。<br>
<br>
＜MSDOS Playerの場所＞<br>
http://takeda-toshiya.my.coocan.jp/msdos/
<br>
こちらも、参考になるかと。<br>
http://iamdoingse.livedoor.blog/archives/24144518.html


# その他で使用した開発ツール
FWのソースのコンパイルは、マイクロチップ社の<br>
<br>
「MPLAB® X Integrated Development Environment (IDE)」<br>
<br>
を使っています。（MPLAB X IDE v6.10）コンパイラは、XC8を使用しています。<br>
https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide<br>
<br>
universal moniter 8088/V20は、Macro Assembler AS V1.42を使用してバイナリを<br>
作成しています。<br>
http://john.ccac.rwth-aachen.de:8000/as/<br>
<br>
FatFsはR0.15を使用しています。<br>
＜FatFs - Generic FAT Filesystem Module＞<br>
http://elm-chan.org/fsw/ff/00index_e.html<br>
<br>
SDカード上のMS-DOSイメージファイルの作成は、ImDisk Virtual Disk Driverを利用しています。<br>
ここで、紹介されています。<br>
https://freesoft-100.com/review/imdisk-virtual-disk-driver.html
<br>
本家は、ここです。<br>
http://www.ltr-data.se/opencode.html/#ImDisk
<br>
＜＠hanyazouさんのソース＞<br>
https://github.com/hanyazou/SuperMEZ80/tree/mez80ram-cpm<br>
<br>
＜@electrelicさんのユニバーサルモニタ＞<br>
https://electrelic.com/electrelic/node/1317<br>
<br>
<br>
# 参考
・EMUZ80<br>
EUMZ80はZ80CPUとPIC18F47Q43のDIP40ピンIC2つで構成されるシンプルなコンピュータです。<br>
<br>
＜電脳伝説 - EMUZ80が完成＞  <br>
https://vintagechips.wordpress.com/2022/03/05/emuz80_reference  <br>
＜EMUZ80専用プリント基板 - オレンジピコショップ＞  <br>
https://store.shopping.yahoo.co.jp/orangepicoshop/pico-a-051.html<br>
<br>
・SuperMEZ80<br>
SuperMEZ80は、EMUZ80にSRAMを追加し、Z80をノーウェイトで動かすことができるメザニンボードです<br>
<br>
SuperMEZ80<br>
https://github.com/satoshiokue/SuperMEZ80<br>
<br>
