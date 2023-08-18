function koma_1045;%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  支点をボールジョイントで拘束されたコマ
%
%  回転姿勢にオイラーパラメータを使用。
%  オイラーパラメータの二乗和が１に成るような安定化を計る。
%
%  状態変数は、回転姿勢（オイラーパラメータ）と角速度
%      微分代数型運動方程式を疎行列の解法で解く。⇒　加速度、角加速度（角速度の時間微分）、未定乗数
%      オイラーパラメータの時間微分を、角速度から作る。
%      角速度の時間微分とオイラーパラメータの時間微分だけを積分。⇒　角速度、オイラーパラメータ
%      速度と位置は、角速度と回転姿勢（オイラーパラメータ）から作る。
%
%  単位はセンチメートルとグラムと秒を使用
%  Y軸の負の方向に重力が働く。
%  コマの回転軸もY軸、コマの支点位置はY軸負の部分上の点
%
%  重心の位置 Roa
%  重心の速度 Voa
%  回転姿勢(オイラーパラメータ) --- 一般化座標 Eoa
%  回転行列 Coa
%  角速度 --- 一般化速度 Omoa
%  DEoaは、Eoaの時間微分
%  DOmoaは、Omoaの時間微分
%
%  コマの支点位置 Rap
%  コマの質量 Ma
%  慣性行列(重心まわり) Joa
%  支点Pまわりの慣性行列 pJoa
%  重力加速度 g
%  支点の回転減衰係数(行列) CC
%  作用力(重力) Foa
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  使用する別の関数
%
%  function DY=e_koma_1045(t,Y,SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);%
%            微分方程式の右辺を計算する関数
%
%  function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%            Ｙ軸まわりの回転体を、頂点と多角形の面の集まり（AvertとAface）として作る関数。
%            回転体を近似する分割数（AYrevDevide）、
%                            Ｙ軸上の位置と半径による形状定義（AYrevStep,AYrevYandRadii）
%                            を入力とする。(AYrevStepは半径を指定している位置の段数)
%
%  function [TildeA]=TILDE(A);%
%            ３×１行列 R に外積オペレータを作用させた交代行列を作る関数。
%            TILDE(R)=[0 -R(3) R(2);R(3) 0 -R(1);-R(2) R(1) 0]
%
%  function [C]=EtoC(E);%
%            オイラーパラメータ E から回転行列を作る関数。(E0=E(1),E1=E(2),E2=E(3),E3=E(4)として)
%            E2C(E)=[E1*E1-E2*E2-E3*E3+E0*E0 2(E1*E2-E3*E0)          2(E3*E1+E2*E0);
%                    2(E1*E2+E3*E0)          E2*E2-E3*E3-E1*E1+E0*E0 2(E2*E3-E1*E0);
%                    2(E3*E1-E2*E0)          2(E2*E3+E1*E0)          E3*E3-E1*E1-E2*E2+E0*E0]
%
%  function [S]=EtoS(E);%
%            オイラーパラメータ E からＳ行列を作る関数。(E0=E(1),E1=E(2),E2=E(3),E3=E(4)として)
%            E2S(E)=[-E1  E0  E3 -E2;
%                    -E2 -E3  E0  E1;
%                    -E3  E2 -E1  E0]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all;%                          全ての図を閉じる
clear all;%                          ワークスペースのクリア
%
global OUTPUT;%                     グラフ用出力変数の受渡しに利用する変数
global ANMPUT;%                     アニメ用出力変数の受渡しに利用する変数
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第一段階
%  シミュレーションに用いる数値パラメータの設定
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  コマと環境のパラメータ
%
g=980.;%                            重力加速度
Ma=32*pi;%                          質量
Joa=[128*pi 0      0;%
     0      256*pi 0;%
     0      0      128*pi];%        重心まわりの慣性行列
Rap=[0;%
    -3;%
     0];%　 　　　　　　　　   　　 　支点の位置
Cxz=0;%                             Ｘ軸、Ｚ軸まわりの角速度成分に対する支点まわりの回転ダンピング係数　
Cyy=0;%                             Ｙ軸まわりの角速度成分に対する支点まわりの回転ダンピング係数
%
%  オイラーパラメータ安定化のための時定数
%
TAU=0.1;%                           オイラーパラメータ安定化のための時定数
%
%  初期条件
%
EoaINITIAL=[cos(pi/12);%
            0;%
            0;%
           -sin(pi/12)];%           初期回転姿勢（オイラーパラメータ）
OmoaINITIAL=[  0;%
             113.369;%
               0];%                 初期角速度
%
%  シミュレーション時間
%
t0=0;%                              計算開始時間
dt=0.01;%                           計算結果出力キザミ
tf=6;%                              計算終了時間
%
%  アニメ用の形状　  
%      Ｙ軸まわりの回転体のみを扱う
%      NumDevide,NumStepの要素数は回転体の数
%      NumDevideは回転方向の分割数（３以上）
%      NumStepは回転体形状の段数（１以上）
%      YandRadiusにはすべての回転体回転体のすべての段に対して
%                     Ｙ座標値とその位置での半径が与えられている
%      半径がゼロの場合はＹ軸上に一つの点が取られる
%      半径がゼロ以外の場合は円周上に分割数だけの点が取られる
%      回転体端部の半径が正数の場合、その端面にもパッチを張る
%      回転体端部の半径が負数の場合、半径としてはその絶対値を用いる
%      回転体端部の半径が負数の場合、その端面にはパッチを張らない
%      回転体端部以外の半径の正負は無意味である
%
AYrevnumDevide =[16];%             Ｙ軸まわり一回転の分割数
AYrevnumStep   =[7];%              Ｙ軸に沿って、半径を指定する段数
AYrevYandRadii =[-3   0;%
                 -2.5 0.5;%
                 -1   0.5;%
                 -0.5 3.5;%
                  0.5 3.5;%
                  0.5 0.5;%
                  2   0.5];%        Ｙ軸に沿った位置と、その位置での半径
%
%  グラフ用出力に対し、ANMSKIPに一回の割合でアニメ出力を作る。
%
ANMaxis=[-5 5 -1 5 -5 5];%         アニメ表示領域
ANMskip = 3;%                      グラフ出力に対するアニメ出力の削減割合
AfaceColor = 'yellow';%            形状を表す多角形の面の色
AedgeColor = 'red';%               形状を表す多角形のエッジの色
ANMfps = 33;%                      一秒間当たりのアニメ画面表示数
ANMview = [1 0 0];%                アニメ表示用カメラの方向
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第二段階
%  初期処理
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
YINITIAL=[ EoaINITIAL;%
          OmoaINITIAL];%           状態変数初期値（初期オイラーパラメータと初期角速度）
Mag=Ma*g;%                         重力
Foa=[0;%
    -Mag;%
     0];%                          作用力（重力）
TILDERap=TILDE(Rap);%              Rapから作った交代行列
pJoa=Joa+TILDERap'*Ma*TILDERap;%   支点Ｐまわりの慣性行列
CC=[Cxz 0   0;%
    0   Cyy 0;%
    0   0   Cxz];%                 支点まわりの回転ダンピング係数行列
%
%  疎行列の準備
%
AM=zeros(9,9);%                    ９×９行列
AM(1,1)=Ma;%                       左上の３×３部分は、質量によるスカラー行列
AM(2,2)=Ma;%
AM(3,3)=Ma;%
AM(4:6,4:6)=Joa;%                  中央の３×３部分は、慣性行列
AM(7,1)=1;%                        左下は、拘束条件の定数部分（単位行列）
AM(8,2)=1;%
AM(9,3)=1;%
AM(1,7)=1;%                        右上（対称位置）は、左下部分の転置
AM(2,8)=1;%
AM(3,9)=1;%
SPAM=sparse(AM);%                  定数部分以外はゼロのままとして、疎行列に変換
BM=zeros(9,1);%                    ９×１行列
BM(1:3)=Foa;%                      上部の３×１には、重力（定数）
SPBM=sparse(BM);%                  定数部分以外はゼロのままとして、疎行列に変換
%                                  乗数以外の部分は、動計算の中で各時間ごとに与えられる。
%                                  SPAMを係数行列とし、SPBMを右辺として
%                                      連立一次方程式を解くと、自動的に疎行列用の解法が実行される。
%
%  アニメ用初期処理
%
[Avert,Aface]=ANMYrev(AYrevnumDevide,AYrevnumStep,AYrevYandRadii);%
%                                  アニメ用形状を構成する多角形頂点の位置、面を構成する頂点の組
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第三段階
%  微分方程式の積分計算
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
[tt,YY]=ode45(@e_koma_1045,[t0:dt:tf],YINITIAL,[],SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);%
%                                　　コマの運動の計算
%                                    出力ttは、t0からtfまでdtキザミの時間が入った列行列（n行1列）
%                                    出力YYは、各時間に対応したコマのオイラー角,
%                                                                      角速度を含む行列（n行7列）
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第四段階
%  出力処理
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
numOUTtable=size(tt,1);%                 出力時間の数
OUTtable=[];%                            グラフ用出力テーブルの準備
ANMtable=[];%                            アニメ用出力テーブルの準備
ANMcount=0;%                             アニメ出力制御のためのカウンター
for i=1:numOUTtable;%                    出力時間の順に
    t=tt(i);%                            時間と
    Y=YY(i,:)';%                         状態変数を準備
    DY=e_koma_1045(t,Y,...;%
        SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);% もう一度微分方程式の右辺の呼び出し
    OUTtable=[OUTtable;OUTPUT];%         出力テーブルに出力を追加
    ANMcount=ANMcount+1;%                アニメ出力制御カウンターを１だけ増加
    if mod(ANMcount-1,ANMskip)==0;%      アニメ出力制御カウンターから１を指し引いた値がANMskipの倍数のとき
        ANMtable=[ANMtable;ANMPUT];%         アニメ用出力テーブルに出力を追加
    end;%
end;%
%
%  グラフ出力
%
figure(1);%                          計算結果出力図１を準備
plot(tt,OUTtable(:,1));%                       時間－コマ重心位置のX座標を描画
xlabel('時間（秒）');%
ylabel('コマ重心位置のX座標（cm）');%
figure(2);%                          計算結果出力図２を準備
plot(tt,OUTtable(:,2));%                       時間－コマ重心位置のY座標を描画
xlabel('時間（秒）');%
ylabel('コマ重心位置のY座標（cm）');%
figure(3);%                          計算結果出力図３を準備
plot(tt,OUTtable(:,3));%                       時間－コマ重心位置のZ座標を描画
xlabel('時間（秒）');%
ylabel('コマ重心位置のZ座標（cm）');%
%
figure(4);%                          計算結果出力図４を準備
plot(OUTtable(:,3),OUTtable(:,1));%            コマ重心のＺ座標－コマ重心のＸ座標を描画
xlabel('コマ重心のＺ座標（cm）');%
ylabel('コマ重心のＸ座標（cm）');%
%
figure(5);%                          計算結果出力図５を準備
plot(tt,YY(:,1));%                              時間－オイラーパラメータ１を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ１');%
figure(6);%                          計算結果出力図６を準備
plot(tt,YY(:,2));%                              時間－オイラーパラメータ２を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ２');%
figure(7);%                          計算結果出力図７を準備
plot(tt,YY(:,3));%                              時間－オイラーパラメータ３を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ３');%
figure(8);%                          計算結果出力図８を準備
plot(tt,YY(:,4));%                              時間－オイラーパラメータ４を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ４');%
%
figure(9);%                          計算結果出力図９を準備
plot(tt,OUTtable(:,4));%                       時間－コマ運動量のX座標成分を描画
xlabel('時間（秒）');%
ylabel('コマ運動量のX座標成分（kg*cm/s）');%
figure(10);%                          計算結果出力図１０を準備
plot(tt,OUTtable(:,5));%                       時間－コマ運動量のY座標成分を描画
xlabel('時間（秒）');%
ylabel('コマ運動量のY座標成分（kg*cm/s）');%
figure(11);%                          計算結果出力図１１を準備
plot(tt,OUTtable(:,6));%                       時間－コマ運動量のZ座標成分を描画
xlabel('時間（秒）');%
ylabel('コマ運動量のZ座標成分（kg*cm/s）');%
%
figure(12);%                          計算結果出力図１２を準備
plot(tt,OUTtable(:,7));%                       時間－コマ角運動量のX座標成分を描画
xlabel('時間（秒）');%
ylabel('コマ角運動量のX座標成分（kg*cm^2/s）');%
figure(13);%                          計算結果出力図１３を準備
plot(tt,OUTtable(:,8));%                       時間－コマ角運動量のY座標成分を描画
xlabel('時間（秒）');%
ylabel('コマ角運動量のY座標成分（kg*cm^2/s）');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     縦軸は、自動スケール最小値の0.9倍～自動スケールの最大値の1.1倍
figure(14);%                          計算結果出力図１４を準備
plot(tt,OUTtable(:,9));%                       時間－コマ角運動量のZ座標成分を描画
xlabel('時間（秒）');%
ylabel('コマ角運動量のZ座標成分（kg*cm^2/s）');%
%
figure(15);%                          計算結果出力図１５を準備
plot(tt,OUTtable(:,10));%                       時間－コマ運動エネルギーを描画
xlabel('時間（秒）');%
ylabel('コマ運動エネルギー（kg*cm^2/s^2）');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     縦軸は、自動スケール最小値の0.9倍～自動スケールの最大値の1.1倍
figure(16);%                          計算結果出力図１６を準備
plot(tt,OUTtable(:,11));%                       時間－コマポテンシャルエネルギーを描画
xlabel('時間（秒）');%
ylabel('コマポテンシャルエネルギー（kg*cm^2/s^2）');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     縦軸は、自動スケール最小値の0.9倍～自動スケールの最大値の1.1倍
figure(17);%                          計算結果出力図１７を準備
plot(tt,OUTtable(:,12));%                       時間－コマ全エネルギーを描画
xlabel('時間（秒）');%
ylabel('コマ全エネルギー（kg*cm^2/s^2）');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     縦軸は、自動スケール最小値の0.9倍～自動スケールの最大値の1.1倍
figure(18);%                          計算結果出力図１８を準備
plot(tt,OUTtable(:,13));%                       時間－オイラーパラメータの拘束を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータの拘束');%
%
%  アニメ出力
%
numANMtable = size(ANMtable,1);%     アニメ用出力テーブルのステップ数
numAvert  = size(Avert,1);%          コマの形状の頂点の数
figure(19);%                         出力図１７を準備
axis(ANMaxis);%                      　  アニメ用に大きさの設定
axis equal;%                             三軸を均等な長さにする
view(ANMview);%                          コマを見る（カメラを設置する）向き
camup([0,1,0]);%                     カメラの上を示す方向
grid on;%
%
    ANMPUT=ANMtable(1,:);%               アニメ用出力テーブルの最初のステップ
    Roa=ANMPUT(1:3)';%                   Roa（重心位置）の取り出し
    Coa=reshape(ANMPUT(4:12)',3,3);%     Coa（回転行列）の取り出し
    Avertex=[];%                         座標系Ｏで表したコマの頂点の位置を作りこむ行列の準備
    for j=1:numAvert;%                   すべての形状節点について
        Roj=Roa+Coa*Avert(j,:)';%　　　　　　 その位置を計算し、
        Avertex=[Avertex;Roj'];%             準備した行列に並べる。
    end;%
    h=patch('Vertices',Avertex,'Faces',Aface,'EdgeColor',AedgeColor,'FaceColor',AfaceColor);%
    M(1)=getframe;%　　　　　　　　　 　　パッチを張って、フレームを取り込む
%
for i=2:numANMtable;%                    同じ作業の繰り返し
    ANMPUT=ANMtable(i,:);%               アニメ用出力テーブルからの取り出し
    Roa=ANMPUT(1:3)';%                   Roa（重心位置）の取り出し
    Coa=reshape(ANMPUT(4:12),3,3);%      Coa（回転行列）の取り出し
    Avertex=[];%                         座標系Ｏで表したコマの頂点の位置を作りこむ行列の準備
    for j=1:numAvert;%                   すべての形状節点について
        Roj=Roa+Coa*Avert(j,:)';%　　　　　　 その位置を計算し、
        Avertex=[Avertex;Roj'];%             準備した行列に並べる。
    end;%
    set(h,'Vertices',Avertex);%          頂点情報の変更
    drawnow;%                            描画
    M(i)=getframe;%                      画面の取り込み
end;%
movie(M,0,ANMfps);%                      取り込んだ全画面の再生
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  微分方程式
%  右辺を計算する関数
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function DY=e_koma_1045(t,Y,SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);%
%
global OUTPUT;%
global ANMPUT;%
%
Eoa  = Y(1:4);%                                状態量からオイラーパラメータを抽出
Omoa = Y(5:7);%                                状態量から角速度を抽出
%
Coa = EtoC(Eoa);%                              回転行列
Soa = EtoS(Eoa);%                              オイラーパラメータの時間微分と角速度の関係の３×４係数行列
%
Nop = -CC*Omoa;%                               支点Ｐに働く回転減衰トルク
Noa = Nop;%                                    重心Ａへ等価換算しても同じ値
TILDEOmoa = TILDE(Omoa);%                      角速度Omoaから作った交代行列
%
SPAM(7:9,4:6) = -Coa*TILDERap;%                疎行列SPAMの動的に変動するの部分
SPAM(4:6,7:9) = SPAM(7:9,4:6)';%               その対称部分
SPBM(4:6) = Noa-TILDEOmoa*(Joa*Omoa);%         疎行列SPBMの動的に変動するの部分
SPBM(7:9) = Coa*(TILDEOmoa*(TILDERap*Omoa));%  疎行列SPBMの動的に変動するの部分
SPXM = SPAM\SPBM;%
%
DEoa  = Soa'*Omoa*0.5-Eoa*(0.5*(1-1/(Eoa'*Eoa))/TAU);%  オイラーパラメータの時間微分（＋拘束安定化）
DOmoa = full(SPXM(4:6));%                      角速度の時間微分
DY = [DEoa;DOmoa];%                            状態量の時間微分
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  出力のための計算
%
Roa = -Coa*Rap;%                               重心位置
Voa = Coa*(TILDERap*Omoa);%                    重心速度
Poa = Ma*Voa;%                                 運動量
PIoa = Coa*(pJoa*Omoa);%                       角運動量（慣性座標系表現）
Ta = Omoa'*pJoa*Omoa*0.5;%                     運動エネルギー
Ua = Roa(2)*Mag;%                              位置エネルギー
TUa = Ta+Ua;%                                  全エネルギー
%
OUTPUT = [Roa(1),Roa(2),Roa(3),...,%
          Poa(1),Poa(2),Poa(3),PIoa(1),PIoa(2),PIoa(3),Ta,Ua,TUa,...;%
          Eoa'*Eoa-1];%                        グラフ用出力のOUTPUTへの格納
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
ANMPUT = [Roa',Coa(:)'];%                      グラフ用出力（重心位置と回転行列）のANMPUTへの格納
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Ｙ軸まわり回転体の頂点と面の情報を作る関数
%
function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%
%  このプログラムの詳しい説明は省略する。
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
Vert = [];%
Face = [];%
if length(NumDevide)~=length(NumStep);%
    error('user input error at 1');%
end;%
MaxNumDevide=0;%
SumNumStep=0;%
for i=1:length(NumDevide);%
    if NumDevide(i)<=2;%
        error('user input error at 2');%
    end;%
    if NumStep(i)<=0;%
        error('user input error at 3');%
    end;%
    MaxNumDevide=max(MaxNumDevide,NumDevide(i));%
    SumNumStep=SumNumStep+NumStep(i);%
end;%
if size(YandRadii)~=[SumNumStep 2];%
    error('user input error at 4');%
end;%
StepSum=0;%
VertexSum=0;%
Face0=ones(1,max(MaxNumDevide,4))*NaN;%
Face1=Face0;%
OneVertexFlag=0;%
for i=1:length(NumDevide);%
    for j=1:NumStep(i);%
        StepSum=StepSum+1;%
        Y = YandRadii(StepSum,1);%
        Radius = YandRadii(StepSum,2);%
        if Radius==0;%
            if OneVertexFlag==1;%
                error('user input error at 5');%
            end;%
            OneVertexFlag=1;%
            Vert=[Vert;[0 Y 0]];%
            VertexSum=VertexSum+1;%
            if j~=1;%
                Face1(3)=VertexSum;%
                for k=1:NumDevide(i);%
                    Face1(1)=VertexSum-NumDevide(i)+mod(k-1,NumDevide(i));%
                    Face1(2)=VertexSum-NumDevide(i)+mod(k,NumDevide(i));%
                    Face=[Face;Face1];%
                    Face1=Face0;%
                end;%
            end;%
        else;%
            for k=1:NumDevide(i);%
                X=abs(Radius)*sin(2*pi*(k-1)/NumDevide(i));%
                Z=abs(Radius)*cos(2*pi*(k-1)/NumDevide(i));%
                Vert=[Vert;[X Y Z]];%
                VertexSum=VertexSum+1;%
            end;%
            if j~=1;%
                if OneVertexFlag==0;%
                    for k=1:NumDevide(i);%
                        Face1(3)=VertexSum+1-NumDevide(i)+mod(k,NumDevide(i));%
                        Face1(4)=VertexSum+1-NumDevide(i)+mod(k-1,NumDevide(i));%
                        Face1(1)=VertexSum+1-2*NumDevide(i)+mod(k-1,NumDevide(i));%
                        Face1(2)=VertexSum+1-2*NumDevide(i)+mod(k,NumDevide(i));%
                        Face=[Face;Face1];%
                        Face1=Face0;%
                    end;%
                else;%
                    for k=1:NumDevide(i);%
                        Face1(3)=VertexSum-NumDevide(i);%
                        Face1(1)=VertexSum+1-NumDevide(i)+mod(k-1,NumDevide(i));%
                        Face1(2)=VertexSum+1-NumDevide(i)+mod(k,NumDevide(i));%
                        Face=[Face;Face1];%
                        Face1=Face0;%
                    end;%
                end;%
                OneVertexFlag=0;%
            end;%
            if j==1 | j==NumStep(i);%
                if Radius>0;%
                    for n=1:NumDevide(i);%
                        Face1(n)=VertexSum+1-NumDevide(i)+mod(n-1,NumDevide(i));%
                    end;%
                    Face=[Face;Face1];%
                    Face1=Face0;%
                end;%
            end;%
        end;%
    end;%
end;%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function [TildeA]=TILDE(A);%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
A1=A(1);%
A2=A(2);%
A3=A(3);%
TildeA=[ 0 -A3  A2;...;%
        A3   0 -A1;...;%
       -A2  A1   0];%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function [C]=EtoC(E);%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
E1=E(1);%
E2=E(2);%
E3=E(3);%
E4=E(4);%
%
C=[E2*E2-E3*E3-E4*E4+E1*E1 (E2*E3-E4*E1)*2          (E4*E2+E3*E1)*2;...;%
  (E2*E3+E4*E1)*2           E3*E3-E4*E4-E2*E2+E1*E1 (E3*E4-E2*E1)*2;...;%
  (E4*E2-E3*E1)*2          (E3*E4+E2*E1)*2           E4*E4-E2*E2-E3*E3+E1*E1];%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function [S]=EtoS(E);%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
E0=E(1);%
E1=E(2);%
E2=E(3);%
E3=E(4);%
%
S=[-E1  E0  E3 -E2;...;%
   -E2 -E3  E0  E1;...;%
   -E3  E2 -E1  E0];%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
