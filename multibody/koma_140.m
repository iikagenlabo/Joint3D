function koma_140;%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �x�_���ߎ��{�[���W���C���g�ŋߎ��S�����ꂽ�R�}�B
%  �������A�����ł̓{�[���W���C���g���S���ɂ���Ď�������̂ł͂Ȃ��B
%  �o�l�ƃ_���p�[�I�ȋ@�\�ɂ��A�ߎ��I�ȃ{�[���W���C���g��p����B
%  �R�}�̎��R�x�͂R�ł͂Ȃ��A�U�ł���B
%
%  ��]�p���ɂ̓I�C���[�p�����[�^���g�p�B
%  �I�C���[�p�����[�^�̓��a���P�ɂȂ�悤�ȊǗ����s��Ȃ��ꍇ�𒲂ׂ�B
%
%  �P�ʂ̓Z���`���[�g���i�����j�ƃO�����i���ʁj�ƕb�i���ԁj���g�p�B
%  �������W�n�n�̂x���̕��̕����ɏd�͂������B
%  �R�}�ɓ�����p�͂́A���̏d�͂̂ق��ɁA�x�_�ɓ����o�l�̓_���s���O�͂���B
%  �R�}�̉�]�������W�n�`�̂x���A�R�}�̎x�_���x����ɂ���B�i�x�_�̂x���W�l Rap(2)�͕��j
%
%  ��ʉ����W�́A�x�_�ʒu Roa �ƃI�C���[�p�����[�^ Eoa �B
%�@��ʉ����x�́A�x�_���x Voa �Ɗp���xOmoa�i�`���W�n�\���j�B
%
%  ��]�s�� Coa
%  �R�}�̍��W�n���猩���x�_�ʒu Rap�i�萔�j
%  DEoa�́AEoa�̎��Ԕ���
%  DOmoa�́AOmoa�̎��Ԕ���
%  DRoa�́ARoa�̎��Ԕ���
%  DVoa�́AVoa�̎��Ԕ���
%  �R�}�̎��� Ma�i�萔�j
%  �����s��(�d�S�܂��) Joa�i�`���W�n�\���A�萔�j
%  Joa�̋t�s�� invJoa�i�萔�j
%  �d�͉����x g�i�萔�j
%  �x�_�̃o�l�萔 KKp�i�萔�j
%  �x�_�̌����W�� CCp�i�萔�j
%  �x�_�̉�]�����W�� Cxz�ACyy�i�萔�j
%  �x�_�̉�]�����W���s�� CC�i�萔�j
%  �x�_�ɓ����o�l�͂ƃ_���s���O�� Fop
%  �x�_�ɓ��������g���N Nop �i�`���W�n�\���j
%  �R�}�ɓ�����p�́iFop�ANop�A�d�͂��d�S�ʒu�ɓ������Z�����l�j Foa
%  �R�}�ɓ�����p�g���N�iFop�ANop�A�d�͂��d�S�ʒu�ɓ������Z�����l�j Noa �i�`���W�n�\���j
%
%  ��ԕϐ� Y
%  ��ԕϐ��̎��Ԕ��� DY
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �g�p����ʂ̊֐�
%
%  function DY=e_koma_140(t,Y,Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%            �����������̉E�ӂ��v�Z����֐�
%
%  function [TildeA]=TILDE(A);%
%            �R�~�P�s�� A �ɊO�σI�y���[�^����p���������s������֐��B
%            TILDE(A)=[0 -A(3) A(2);A(3) 0 -A(1);-A(2) A(1) 0]
%
%  function [C]=EtoC(E);%
%            �I�C���[�p�����[�^ E �����]�s�� C �����֐��B
%            E0=E(1),E1=E(2),E2=E(3),E3=E(4)�Ƃ��āA
%            EtoC(E)=[E1*E1-E2*E2-E3*E3+E0*E0 2*(E1*E2-E3*E0)         2*(E3*E1+E2*E0);
%                     2*(E1*E2+E3*E0)         E2*E2-E3*E3-E1*E1+E0*E0 2*(E2*E3-E1*E0);
%                     2*(E3*E1-E2*E0)         2*(E2*E3+E1*E0)         E3*E3-E1*E1-E2*E2+E0*E0]
%
%  function [S]=EtoS(E);%
%            �I�C���[�p�����[�^ E ���� S �s������֐��B
%            E0=E(1),E1=E(2),E2=E(3),E3=E(4)�Ƃ��āA
%            EtoS(E)=[-E1  E0  E3 -E2;
%                     -E2 -E3  E0  E1;
%                     -E3  E2 -E1  E0]
%
%  function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%            �x���܂��̉�]�̂��A���_�Ƒ��p�`�̖ʂ̏W�܂�iVert��Face�j�Ƃ��č��֐��B
%            ��]�̂��ߎ����镪�����iNumDevide�j�A
%                      �x����̈ʒu�Ɣ��a�ɂ��`���`�iNumStep,YandRadii�j
%                                ����͂Ƃ���B(NumStep�͔��a���w�肵�Ă���ʒu�̒i��)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all;%                         �S�Ă̐}�����
clear all;%                         ���[�N�X�y�[�X�̃N���A
%
global OUTPUT;%                     �o�͕ϐ��̎�n���ɃO���[�o���ϐ��𗘗p����B
global ANMPUT;%                     �A�j���p�o�͕ϐ��̎�n���ɃO���[�o���ϐ��𗘗p����B
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ���i�K
%  �V�~�����[�V�����f�[�^�̐ݒ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �R�}�Ɗ��̃p�����[�^
%
g=980.;%                       �d�͉����x
Ma=32*pi;%                     ����
Joa=[128*pi 0      0;%
     0      256*pi 0;%
     0      0      128*pi];%   �d�S�܂��̊����s��
Rap=[0;%
    -3;%
     0];% �@�@�@�@�@�@�@�@�@  �@�`���W�n���猩���x�_�̈ʒu
KKp=10000000;%                 �x�_�̕ψʂɑ΂���o�l�萔�@
CCp=100000;%                   �x�_�̑��x�ɑ΂���_���s���O�A�ː�
Cxz=0;%                        �w���y���܂��̊p���x�����ɑ΂���x�_�܂��̉�]�_���s���O�W��
Cyy=0;%                        �x���܂��̊p���x�����ɑ΂���x�_�܂��̉�]�_���s���O�W��
%
%  ��������
%  �x�_�̈ʒu�A���x�ƃR�}�̉�]�p���A�p���x�����
%  ���������ŃR�}�̏d�S�ʒu�Əd�S���x�̏����l�����߂�B
%
RopINITIAL=[0;%
            0;%
            0];%                     �����̎x�_�ʒu
VopINITIAL=[0;%
            0;%
            0];%                     �����̎x�_���x
EoaINITIAL=[cos(pi/12);%
            0;%
            0;%
           -sin(pi/12)*1.1];%        ������]�p���i�I�C���[�p�����[�^�j
%                                        �̈ӂɏ����̃I�C���[�p�����[�^�̑�4������1.1�{���āA
%                                                ���a���P�ɂȂ�Ȃ��悤�ɂ��Ă���B
%
% EoaINITIAL=[cos(pi/12);%
%             0;%
%             0;%
%            -sin(pi/12)];%            ������]�p���i�I�C���[�p�����[�^�j
%
OmoaINITIAL=[  0;%
             113.369;%
               0];%                  �����p���x
%
%  �V�~�����[�V��������
%
t0=0;%                               �v�Z�J�n����
dt=0.01;%                            �v�Z���ʏo�̓L�U�~
tf=6;%                               �v�Z�I������
%
%  �A�j���p�̌`��@
%      Y���܂��̉�]�݂̂̂�����
%      AYrevDevide,AYrevStep�̗v�f���͉�]�̂̐��i�ʏ�P�j
%      AYrevDevide�͉�]�����̕������i�R�ȏ�j
%      AYrevStep�͉�]�̌`��iAYrevYandRadii�j�̒i���i�P�ȏ�j
%      AYrevYandRadii�ɂ͂��ׂẲ�]�̂̂��ׂĂ̒i�ɑ΂���
%                     Y���W�l�Ƃ��̈ʒu�ł̔��a���^�����Ă���
%      ���a���[���̏ꍇ��Y����Ɉ�̓_�������
%      ���a���[���ȊO�̏ꍇ�͉~����ɕ����������̓_�������
%      ��]�̒[���̔��a�������̏ꍇ�A���̒[�ʂɂ��p�b�`�𒣂�
%      ��]�̒[���̔��a�������̏ꍇ�A���a�Ƃ��Ă͂��̐�Βl��p����
%      ��]�̒[���̔��a�������̏ꍇ�A���̒[�ʂɂ̓p�b�`�𒣂�Ȃ�
%      ��]�̒[���ȊO�̔��a�̐����͖��Ӗ��ł���
%
AYrevDevide =[16];%           ��]�����̕������i�R�ȏ�j
AYrevStep   =[7];%            ��]�̌`��iAYrevYandRadii�j�̒i���i�P�ȏ�j
AYrevYandRadii =[-3   0;%     ���i �x���W�l�Ƃ��̈ʒu�ł̔��a
                 -2.5 0.5;%   ���i �x���W�l�Ƃ��̈ʒu�ł̔��a
                 -1   0.5;%   ��O�i �x���W�l�Ƃ��̈ʒu�ł̔��a
                 -0.5 3.5;%   ��l�i �x���W�l�Ƃ��̈ʒu�ł̔��a
                  0.5 3.5;%   ��ܒi �x���W�l�Ƃ��̈ʒu�ł̔��a
                  0.5 0.5;%   ��Z�i �x���W�l�Ƃ��̈ʒu�ł̔��a
                  2   0.5];%  �掵�i �x���W�l�Ƃ��̈ʒu�ł̔��a
%
%  �O���t�p�o�͂ɑ΂��AANMskip�Ɉ��̊����ŃA�j���o�͂����B
%
ANMaxis=[-5 5 -1 5 -5 5];%        �\���͈�
ANMskip = 3;%                     �O���t�p�o�͂ɑ΂���X�L�b�v��
AfaceColor = 'yellow';%           �p�b�`�̖ʂ̐F
AedgeColor = 'red';%              �p�b�`�̃G�b�W�̐F
ANMfps = 33;%                     �b�Ԃ̕\���R�}��
ANMview = [1 0 0];%               �J�����̌���
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ���i�K
%  ��������
%  �萔�Ə�����ԗ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
Dy=[0;1;0];%                         �萔�i�x�����������o�����߁A�܂��́A�x���������������킹�邽�߂Ɏg�p�j
TILDERap=TILDE(Rap);%                Rap�����������s��
CC=[Cxz 0  0;%
     0 Cyy 0;%
     0  0 Cxz];%                     �x�_�܂��̉�]�_���s���O�W���s��
DyMag=Dy*Ma*g;%                      �d��
invJoa=inv(Joa);%                    Joa�̋t�s��
%
CoaINITIAL=EtoC(EoaINITIAL);%                               ��]�s�񏉊��l
RoaINITIAL=RopINITIAL-CoaINITIAL*Rap;%                      �d�S�ʒu�����l
VoaINITIAL=VopINITIAL+CoaINITIAL*TILDERap*OmoaINITIAL;%     �d�S���x�����l
%
YINITIAL=[EoaINITIAL;%
          OmoaINITIAL;%
          RoaINITIAL;%
          VoaINITIAL];%              ��ԗʏ����l
%
%  �A�j���p��������
%
[Avert,Aface]=ANMYrev(AYrevDevide,AYrevStep,AYrevYandRadii);%      �R�}�̌`���\�����_�Ɩ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ��O�i�K
%  �����������̐ϕ��v�Z
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
[tt,YY]=ode45(@e_koma_140,[t0:dt:tf],YINITIAL,[],...;%
        Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%                      �R�}�̉^���̌v�Z
%                      �o��tt�́At0����tf�܂�dt�L�U�~�̎��Ԃ���������s��in�s1��j
%                      �o��YY�́A�e���ԂɑΉ������R�}�̃I�C���[�p�A�p���x�A
%                                                     �d�S�ʒu�A�d�S���x���܂ލs��in�s12��j
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ��l�i�K
%  �o�͏���
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
numOUTtable=size(tt,1);%             �o�͎��Ԃ̐�
OUTtable=[];%                        �O���t�o�̓e�[�u���̏���
ANMtable=[];%                        �A�j���o�̓e�[�u���̏���
ANMcount=0;%                         �A�j���o�̓e�[�u���쐬�̂��߂̃J�E���^�[�����l
for i=1:numOUTtable;%                �o�͎��Ԃ̏���
    t=tt(i);%                        ���Ԃ�
    Y=YY(i,:)';%                     ��ԕϐ�������
    DY=e_koma_140(t,Y,Rap,TILDERap,KKp,CCp,CC,DyMag,Joa,invJoa,Ma);%
%                                    ������x�����������E�ӂ̌v�Z���Ăяo��
    OUTtable=[OUTtable;OUTPUT];%     �O���t�o�̓e�[�u���ɏo�͂�ǉ�
    ANMcount=ANMcount+1;%            �A�j���o�̓e�[�u���쐬�̂��߂̃J�E���^�[����A�b�v
    if mod(ANMcount-1,ANMskip)==0;%  �J�E���^�[�̒l����P�������l��ANMskip�̔{���ɂȂ����Ƃ��A
        ANMtable=[ANMtable;ANMPUT];% �A�j���o�̓e�[�u���ɏo�͂�ǉ�
    end;%
end;%
%
%  �O���t�o��
%
figure(1);%                          �v�Z���ʏo�͐}�P������
plot(tt,YY(:,1));%                              ���ԁ|�I�C���[�p�����[�^�i�P�j��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�i�P�j');%
figure(2);%                          �v�Z���ʏo�͐}�Q������
plot(tt,YY(:,2));%                              ���ԁ|�I�C���[�p�����[�^�i�Q�j��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�i�Q�j');%
figure(3);%                          �v�Z���ʏo�͐}�R������
plot(tt,YY(:,3));%                              ���ԁ|�I�C���[�p�����[�^�i�R�j��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�i�R�j');%
figure(4);%                          �v�Z���ʏo�͐}�S������
plot(tt,YY(:,4));%                              ���ԁ|�I�C���[�p�����[�^�i�S�j��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�i�S�j');%
%
figure(5);%                          �v�Z���ʏo�͐}�T������
plot(YY(:,10),YY(:,8));%                        �R�}�d�S�̂y���W�|�R�}�d�S�̂w���W��`��
xlabel('�R�}�d�S�̂y���W�icm�j');%
ylabel('�R�}�d�S�̂w���W�icm�j');%
%
figure(6);%                          �v�Z���ʏo�͐}�U������
plot(tt,YY(:,8));%                              ���ԁ|�R�}�d�S�̂w���W��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�d�S�̂w���W�icm�j');%
figure(7);%                          �v�Z���ʏo�͐}�V������
plot(tt,YY(:,9));%                              ���ԁ|�R�}�d�S�̂x���W��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�d�S�̂x���W�icm�j');%
figure(8);%                          �v�Z���ʏo�͐}�W������
plot(tt,YY(:,10));%                             ���ԁ|�R�}�d�S�̂y���W��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�d�S�̂y���W�icm�j');%
%
figure(9);%                          �v�Z���ʏo�͐}�X������
plot(tt,OUTtable(:,1));%                       ���ԁ|�_�n�o�Ԃ̋����̓���`��
xlabel('���ԁi�b�j');%
ylabel('�_�n�o�Ԃ̋����̓��icm^2�j');%
figure(10);%                         �v�Z���ʏo�͐}�P�O������
plot(tt,OUTtable(:,2));%                       ���ԁ|�I�C���[�p�����[�^�̍S����`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�̍S��');%
tmpaxis=axis; axis([tmpaxis(1),tmpaxis(2),-0.001,0.016]);%
%                ���̃O���t�̏c���́A�����X�P�[���ł͂Ȃ��A����Ɖ�����^�����Ă���B
%
%  �A�j���o��
%
numANMtable = size(ANMtable,1);%     �A�j���p�o�̓e�[�u���̃X�e�b�v��
numAvert  = size(Avert,1);%          �R�}�̌`��̒��_�̐�
figure(11);%                         �o�͐}�P�P������
axis(ANMaxis);%                      �@  �A�j���p�ɑ傫���̐ݒ�
axis equal;%                             �O�����ϓ��Ȓ����ɂ���
view(ANMview);%                          �R�}������i�J������ݒu����j����
camup([0,1,0]);%                     �J�����̏����������
grid on;%
%
    ANMPUT=ANMtable(1,:);%               �A�j���p�o�̓e�[�u���̍ŏ��̃X�e�b�v
    Roa=ANMPUT(1:3)';%                   Roa�i�d�S�ʒu�j�̎��o��
    Coa=reshape(ANMPUT(4:12)',3,3);%     Coa�i��]�s��j�̎��o��
    Avertex=[];%                         ���W�n�n�ŕ\�����R�}�̒��_�̈ʒu����肱�ލs��̏���
    for j=1:numAvert;%                   ���ׂĂ̌`��ߓ_�ɂ���
        Roj=Roa+Coa*Avert(j,:)';%�@�@�@�@�@�@ ���̈ʒu���v�Z���A
        Avertex=[Avertex;Roj'];%             ���������s��ɕ��ׂ�B
    end;%
    h=patch('Vertices',Avertex,'Faces',Aface,'EdgeColor',AedgeColor,'FaceColor',AfaceColor);%
    M(1)=getframe;%�@�@�@�@�@�@�@�@�@�@�@�p�b�`�𒣂��āA�t���[������荞��
%
for i=2:numANMtable;%                    ������Ƃ̌J��Ԃ�
    ANMPUT=ANMtable(i,:);%
    Roa=ANMPUT(1:3)';%
    Coa=reshape(ANMPUT(4:12),3,3);%
    Avertex=[];%
    for j=1:numAvert;%
        Roj=Roa+Coa*Avert(j,:)';%
        Avertex=[Avertex;Roj'];%
    end;%
    set(h,'Vertices',Avertex);%          ���_���̕ύX
    drawnow;%                            �`��
    M(i)=getframe;%                      ��ʂ̎�荞��
end;%
movie(M,0,ANMfps);%                      ��荞�񂾑S��ʂ̍Đ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ����������
%  �E�ӂ��v�Z����֐�
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
Eoa =Y( 1:4);%                               ��ԗʂ���I�C���[�p�����[�^�𒊏o
Omoa=Y( 5:7);%                               ��ԗʂ���p���x�𒊏o
Roa =Y( 8:10);%                              ��ԗʂ���d�S�ʒu�𒊏o
Voa =Y(11:13);%                              ��ԗʂ���d�S���x�𒊏o
%
Coa=EtoC(Eoa);%                              �I�C���[�p�����[�^�����]�s������B
Soa=EtoS(Eoa);%                              �I�C���[�p�����[�^����r�s������B
%
TILDEOmoa=TILDE(Omoa);%                      �p���xOmoa�����������s��
%
DEoa=0.5*Soa'*Omoa;%                         �I�C���[�p�����[�^�̎��Ԕ���
DRoa=Voa;%                                   �d�S�ʒu�̎��Ԕ���
%
Rop=Roa+Coa*Rap;%                            �x�_�̈ʒu
Vop=Voa-Coa*TILDERap*Omoa;%                  �x�_�̑��x
%
Fop=-Rop*KKp-Vop*CCp;%                       �x�_�ɓ����o�l�͂ƃ_���s���O��
Nop=-CC*Omoa;%                               �x�_�ɓ��������g���N
Foa=-DyMag+Fop;%                             Fop��Nop�̓������Z�{�d��
Noa=Nop+TILDERap*Coa'*Fop;%                  Fop��Nop�̓������Z
%
DOmoa=invJoa*(Noa-TILDEOmoa*Joa*Omoa);%      �p���x�̎��Ԕ���
DVoa=Foa/Ma;%                                ���x�̎��Ԕ���
%
DY=[DEoa;%
    DOmoa;%
    DRoa;%
    DVoa];%                                  ��ԕϐ��̎��Ԕ���
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
OUTPUT = [Rop'*Rop Eoa'*Eoa-1];%             �O���t�p�o�͂ւ̊i�[
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
ANMPUT = [Roa',Coa(:)'];%                    �A�j���p�o�͂ւ̊i�[�i�d�S�ʒu�A��]�s��j
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �x���܂��̉�]�̂̐ߓ_�Ɩʂ����֐�
%
function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%
%  ���̃v���O�����̏ڂ��������͏ȗ�����B
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
