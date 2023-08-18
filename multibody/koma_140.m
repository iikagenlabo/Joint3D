function koma_140;%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  支点を近似ボールジョイントで近似拘束されたコマ。
%  ただし、ここではボールジョイントを拘束によって実現するのではない。
%  バネとダンパー的な機能による、近似的なボールジョイントを用いる。
%  コマの自由度は３ではなく、６である。
%
%  回転姿勢にはオイラーパラメータを使用。
%  オイラーパラメータの二乗和が１になるような管理を行わない場合を調べる。
%
%  単位はセンチメートル（長さ）とグラム（質量）と秒（時間）を使用。
%  慣性座標系ＯのＹ軸の負の方向に重力が働く。
%  コマに働く作用力は、この重力のほかに、支点に働くバネ力ダンピング力ある。
%  コマの回転軸も座標系ＡのＹ軸、コマの支点もＹ軸上にある。（支点のＹ座標値 Rap(2)は負）
%
%  一般化座標は、支点位置 Roa とオイラーパラメータ Eoa 。
%　一般化速度は、支点速度 Voa と角速度Omoa（Ａ座標系表現）。
%
%  回転行列 Coa
%  コマの座標系から見た支点位置 Rap（定数）
%  DEoaは、Eoaの時間微分
%  DOmoaは、Omoaの時間微分
%  DRoaは、Roaの時間微分
%  DVoaは、Voaの時間微分
%  コマの質量 Ma（定数）
%  慣性行列(重心まわり) Joa（Ａ座標系表現、定数）
%  Joaの逆行列 invJoa（定数）
%  重力加速度 g（定数）
%  支点のバネ定数 KKp（定数）
%  支点の減衰係数 CCp（定数）
%  支点の回転減衰係数 Cxz、Cyy（定数）
%  支点の回転減衰係数行列 CC（定数）
%  支点に働くバネ力とダンピング力 Fop
%  支点に働く減衰トルク Nop （Ａ座標系表現）
%  コマに働く作用力（Fop、Nop、重力を重心位置に等価換算した値） Foa
%  コマに働く作用トルク（Fop、Nop、重力を重心位置に等価換算した値） Noa （Ａ座標系表現）
%
%  状態変数 Y
%  状態変数の時間微分 DY
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  使用する別の関数
%
%  function DY=e_koma_140(t,Y,Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%            微分方程式の右辺を計算する関数
%
%  function [TildeA]=TILDE(A);%
%            ３×１行列 A に外積オペレータを作用させた交代行列を作る関数。
%            TILDE(A)=[0 -A(3) A(2);A(3) 0 -A(1);-A(2) A(1) 0]
%
%  function [C]=EtoC(E);%
%            オイラーパラメータ E から回転行列 C を作る関数。
%            E0=E(1),E1=E(2),E2=E(3),E3=E(4)として、
%            EtoC(E)=[E1*E1-E2*E2-E3*E3+E0*E0 2*(E1*E2-E3*E0)         2*(E3*E1+E2*E0);
%                     2*(E1*E2+E3*E0)         E2*E2-E3*E3-E1*E1+E0*E0 2*(E2*E3-E1*E0);
%                     2*(E3*E1-E2*E0)         2*(E2*E3+E1*E0)         E3*E3-E1*E1-E2*E2+E0*E0]
%
%  function [S]=EtoS(E);%
%            オイラーパラメータ E から S 行列を作る関数。
%            E0=E(1),E1=E(2),E2=E(3),E3=E(4)として、
%            EtoS(E)=[-E1  E0  E3 -E2;
%                     -E2 -E3  E0  E1;
%                     -E3  E2 -E1  E0]
%
%  function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%            Ｙ軸まわりの回転体を、頂点と多角形の面の集まり（VertとFace）として作る関数。
%            回転体を近似する分割数（NumDevide）、
%                      Ｙ軸上の位置と半径による形状定義（NumStep,YandRadii）
%                                を入力とする。(NumStepは半径を指定している位置の段数)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all;%                         全ての図を閉じる
clear all;%                         ワークスペースのクリア
%
global OUTPUT;%                     出力変数の受渡しにグローバル変数を利用する。
global ANMPUT;%                     アニメ用出力変数の受渡しにグローバル変数を利用する。
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第一段階
%  シミュレーションデータの設定
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  コマと環境のパラメータ
%
g=980.;%                       重力加速度
Ma=32*pi;%                     質量
Joa=[128*pi 0      0;%
     0      256*pi 0;%
     0      0      128*pi];%   重心まわりの慣性行列
Rap=[0;%
    -3;%
     0];% 　　　　　　　　　  　Ａ座標系から見た支点の位置
KKp=10000000;%                 支点の変位に対するバネ定数　
CCp=100000;%                   支点の速度に対するダンピング帰依数
Cxz=0;%                        Ｘ軸Ｚ軸まわりの角速度成分に対する支点まわりの回転ダンピング係数
Cyy=0;%                        Ｙ軸まわりの角速度成分に対する支点まわりの回転ダンピング係数
%
%  初期条件
%  支点の位置、速度とコマの回転姿勢、角速度を入力
%  初期処理でコマの重心位置と重心速度の初期値を求める。
%
RopINITIAL=[0;%
            0;%
            0];%                     初期の支点位置
VopINITIAL=[0;%
            0;%
            0];%                     初期の支点速度
EoaINITIAL=[cos(pi/12);%
            0;%
            0;%
           -sin(pi/12)*1.1];%        初期回転姿勢（オイラーパラメータ）
%                                        故意に初期のオイラーパラメータの第4成分を1.1倍して、
%                                                二乗和が１にならないようにしてある。
%
% EoaINITIAL=[cos(pi/12);%
%             0;%
%             0;%
%            -sin(pi/12)];%            初期回転姿勢（オイラーパラメータ）
%
OmoaINITIAL=[  0;%
             113.369;%
               0];%                  初期角速度
%
%  シミュレーション時間
%
t0=0;%                               計算開始時間
dt=0.01;%                            計算結果出力キザミ
tf=6;%                               計算終了時間
%
%  アニメ用の形状　
%      Y軸まわりの回転体のみを扱う
%      AYrevDevide,AYrevStepの要素数は回転体の数（通常１）
%      AYrevDevideは回転方向の分割数（３以上）
%      AYrevStepは回転体形状（AYrevYandRadii）の段数（１以上）
%      AYrevYandRadiiにはすべての回転体のすべての段に対して
%                     Y座標値とその位置での半径が与えられている
%      半径がゼロの場合はY軸上に一つの点が取られる
%      半径がゼロ以外の場合は円周上に分割数だけの点が取られる
%      回転体端部の半径が正数の場合、その端面にもパッチを張る
%      回転体端部の半径が負数の場合、半径としてはその絶対値を用いる
%      回転体端部の半径が負数の場合、その端面にはパッチを張らない
%      回転体端部以外の半径の正負は無意味である
%
AYrevDevide =[16];%           回転方向の分割数（３以上）
AYrevStep   =[7];%            回転体形状（AYrevYandRadii）の段数（１以上）
AYrevYandRadii =[-3   0;%     第一段 Ｙ座標値とその位置での半径
                 -2.5 0.5;%   第二段 Ｙ座標値とその位置での半径
                 -1   0.5;%   第三段 Ｙ座標値とその位置での半径
                 -0.5 3.5;%   第四段 Ｙ座標値とその位置での半径
                  0.5 3.5;%   第五段 Ｙ座標値とその位置での半径
                  0.5 0.5;%   第六段 Ｙ座標値とその位置での半径
                  2   0.5];%  第七段 Ｙ座標値とその位置での半径
%
%  グラフ用出力に対し、ANMskipに一回の割合でアニメ出力を作る。
%
ANMaxis=[-5 5 -1 5 -5 5];%        表示範囲
ANMskip = 3;%                     グラフ用出力に対するスキップ数
AfaceColor = 'yellow';%           パッチの面の色
AedgeColor = 'red';%              パッチのエッジの色
ANMfps = 33;%                     秒間の表示コマ数
ANMview = [1 0 0];%               カメラの向き
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第二段階
%  初期処理
%  定数と初期状態量
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
Dy=[0;1;0];%                         定数（Ｙ軸成分を取り出すため、または、Ｙ軸成分を加え合わせるために使用）
TILDERap=TILDE(Rap);%                Rapから作った交代行列
CC=[Cxz 0  0;%
     0 Cyy 0;%
     0  0 Cxz];%                     支点まわりの回転ダンピング係数行列
DyMag=Dy*Ma*g;%                      重力
invJoa=inv(Joa);%                    Joaの逆行列
%
CoaINITIAL=EtoC(EoaINITIAL);%                               回転行列初期値
RoaINITIAL=RopINITIAL-CoaINITIAL*Rap;%                      重心位置初期値
VoaINITIAL=VopINITIAL+CoaINITIAL*TILDERap*OmoaINITIAL;%     重心速度初期値
%
YINITIAL=[EoaINITIAL;%
          OmoaINITIAL;%
          RoaINITIAL;%
          VoaINITIAL];%              状態量初期値
%
%  アニメ用初期処理
%
[Avert,Aface]=ANMYrev(AYrevDevide,AYrevStep,AYrevYandRadii);%      コマの形状を表す頂点と面
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第三段階
%  微分方程式の積分計算
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
[tt,YY]=ode45(@e_koma_140,[t0:dt:tf],YINITIAL,[],...;%
        Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%                      コマの運動の計算
%                      出力ttは、t0からtfまでdtキザミの時間が入った列行列（n行1列）
%                      出力YYは、各時間に対応したコマのオイラー角、角速度、
%                                                     重心位置、重心速度を含む行列（n行12列）
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  第四段階
%  出力処理
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
numOUTtable=size(tt,1);%             出力時間の数
OUTtable=[];%                        グラフ出力テーブルの準備
ANMtable=[];%                        アニメ出力テーブルの準備
ANMcount=0;%                         アニメ出力テーブル作成のためのカウンター初期値
for i=1:numOUTtable;%                出力時間の順に
    t=tt(i);%                        時間と
    Y=YY(i,:)';%                     状態変数を準備
    DY=e_koma_140(t,Y,Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%                                    もう一度微分方程式右辺の計算を呼び出す
    OUTtable=[OUTtable;OUTPUT];%     グラフ出力テーブルに出力を追加
    ANMcount=ANMcount+1;%            アニメ出力テーブル作成のためのカウンターを一つアップ
    if mod(ANMcount-1,ANMskip)==0;%  カウンターの値から１引いた値がANMskipの倍数になったとき、
        ANMtable=[ANMtable;ANMPUT];% アニメ出力テーブルに出力を追加
    end;%
end;%
%
%  グラフ出力
%
figure(1);%                          計算結果出力図１を準備
plot(tt,YY(:,1));%                              時間－オイラーパラメータ（１）を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ（１）');%
figure(2);%                          計算結果出力図２を準備
plot(tt,YY(:,2));%                              時間－オイラーパラメータ（２）を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ（２）');%
figure(3);%                          計算結果出力図３を準備
plot(tt,YY(:,3));%                              時間－オイラーパラメータ（３）を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ（３）');%
figure(4);%                          計算結果出力図４を準備
plot(tt,YY(:,4));%                              時間－オイラーパラメータ（４）を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータ（４）');%
%
figure(5);%                          計算結果出力図５を準備
plot(YY(:,10),YY(:,8));%                        コマ重心のＺ座標－コマ重心のＸ座標を描画
xlabel('コマ重心のＺ座標（cm）');%
ylabel('コマ重心のＸ座標（cm）');%
%
figure(6);%                          計算結果出力図６を準備
plot(tt,YY(:,8));%                              時間－コマ重心のＸ座標を描画
xlabel('時間（秒）');%
ylabel('コマ重心のＸ座標（cm）');%
figure(7);%                          計算結果出力図７を準備
plot(tt,YY(:,9));%                              時間－コマ重心のＹ座標を描画
xlabel('時間（秒）');%
ylabel('コマ重心のＹ座標（cm）');%
figure(8);%                          計算結果出力図８を準備
plot(tt,YY(:,10));%                             時間－コマ重心のＺ座標を描画
xlabel('時間（秒）');%
ylabel('コマ重心のＺ座標（cm）');%
%
figure(9);%                          計算結果出力図９を準備
plot(tt,OUTtable(:,1));%                       時間－点ＯＰ間の距離の二乗を描画
xlabel('時間（秒）');%
ylabel('点ＯＰ間の距離の二乗（cm^2）');%
figure(10);%                         計算結果出力図１０を準備
plot(tt,OUTtable(:,2));%                       時間－オイラーパラメータの拘束を描画
xlabel('時間（秒）');%
ylabel('オイラーパラメータの拘束');%
tmpaxis=axis; axis([tmpaxis(1),tmpaxis(2),-0.001,0.016]);%
%                このグラフの縦軸は、自動スケールではなく、上限と下限を与えられている。
%
%  アニメ出力
%
numANMtable = size(ANMtable,1);%     アニメ用出力テーブルのステップ数
numAvert  = size(Avert,1);%          コマの形状の頂点の数
figure(11);%                         出力図１１を準備
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
    M(1)=getframe;%　　　　　　　　　　　パッチを張って、フレームを取り込む
%
for i=2:numANMtable;%                    同じ作業の繰り返し
    ANMPUT=ANMtable(i,:);%
    Roa=ANMPUT(1:3)';%
    Coa=reshape(ANMPUT(4:12),3,3);%
    Avertex=[];%
    for j=1:numAvert;%
        Roj=Roa+Coa*Avert(j,:)';%
        Avertex=[Avertex;Roj'];%
    end;%
    set(h,'Vertices',Avertex);%          頂点情報の変更
    drawnow;%                            描画
    M(i)=getframe;%                      画面の取り込み
end;%
movie(M,0,ANMfps);%                      取り込んだ全画面の再生
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  微分方程式
%  右辺を計算する関数
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function DY=e_koma_140(t,Y,Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
global OUTPUT;%
global ANMPUT;%
%
Eoa =Y( 1:4);%                               状態量からオイラーパラメータを抽出
Omoa=Y( 5:7);%                               状態量から角速度を抽出
Roa =Y( 8:10);%                              状態量から重心位置を抽出
Voa =Y(11:13);%                              状態量から重心速度を抽出
%
Coa=EtoC(Eoa);%                              オイラーパラメータから回転行列を作る。
Soa=EtoS(Eoa);%                              オイラーパラメータからＳ行列を作る。
%
TILDEOmoa=TILDE(Omoa);%                      角速度Omoaから作った交代行列
%
DEoa=0.5*Soa'*Omoa;%                         オイラーパラメータの時間微分
DRoa=Voa;%                                   重心位置の時間微分
%
Rop=Roa+Coa*Rap;%                            支点の位置
Vop=Voa-Coa*TILDERap*Omoa;%                  支点の速度
%
Fop=-Rop*KKp-Vop*CCp;%                       支点に働くバネ力とダンピング力
Nop=-CC*Omoa;%                               支点に働く減衰トルク
Foa=-DyMag+Fop;%                             FopとNopの等価換算＋重力
Noa=Nop+TILDERap*Coa'*Fop;%                  FopとNopの等価換算
%
DOmoa=invJoa*(Noa-TILDEOmoa*Joa*Omoa);%      角速度の時間微分
DVoa=Foa/Ma;%                                速度の時間微分
%
DY=[DEoa;%
    DOmoa;%
    DRoa;%
    DVoa];%                                  状態変数の時間微分
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
OUTPUT = [Rop'*Rop Eoa'*Eoa-1];%             グラフ用出力への格納
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
ANMPUT = [Roa',Coa(:)'];%                    アニメ用出力への格納（重心位置、回転行列）
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Ｙ軸まわりの回転体の節点と面を作る関数
%
function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%
%  このプログラムの詳しい説明は省略する。
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
