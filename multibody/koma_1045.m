function koma_1045;%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �x�_���{�[���W���C���g�ōS�����ꂽ�R�}
%
%  ��]�p���ɃI�C���[�p�����[�^���g�p�B
%  �I�C���[�p�����[�^�̓��a���P�ɐ���悤�Ȉ��艻���v��B
%
%  ��ԕϐ��́A��]�p���i�I�C���[�p�����[�^�j�Ɗp���x
%      �����㐔�^�^����������a�s��̉�@�ŉ����B�ˁ@�����x�A�p�����x�i�p���x�̎��Ԕ����j�A����搔
%      �I�C���[�p�����[�^�̎��Ԕ������A�p���x������B
%      �p���x�̎��Ԕ����ƃI�C���[�p�����[�^�̎��Ԕ���������ϕ��B�ˁ@�p���x�A�I�C���[�p�����[�^
%      ���x�ƈʒu�́A�p���x�Ɖ�]�p���i�I�C���[�p�����[�^�j������B
%
%  �P�ʂ̓Z���`���[�g���ƃO�����ƕb���g�p
%  Y���̕��̕����ɏd�͂������B
%  �R�}�̉�]����Y���A�R�}�̎x�_�ʒu��Y�����̕�����̓_
%
%  �d�S�̈ʒu Roa
%  �d�S�̑��x Voa
%  ��]�p��(�I�C���[�p�����[�^) --- ��ʉ����W Eoa
%  ��]�s�� Coa
%  �p���x --- ��ʉ����x Omoa
%  DEoa�́AEoa�̎��Ԕ���
%  DOmoa�́AOmoa�̎��Ԕ���
%
%  �R�}�̎x�_�ʒu Rap
%  �R�}�̎��� Ma
%  �����s��(�d�S�܂��) Joa
%  �x�_P�܂��̊����s�� pJoa
%  �d�͉����x g
%  �x�_�̉�]�����W��(�s��) CC
%  ��p��(�d��) Foa
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �g�p����ʂ̊֐�
%
%  function DY=e_koma_1045(t,Y,SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);%
%            �����������̉E�ӂ��v�Z����֐�
%
%  function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%            �x���܂��̉�]�̂��A���_�Ƒ��p�`�̖ʂ̏W�܂�iAvert��Aface�j�Ƃ��č��֐��B
%            ��]�̂��ߎ����镪�����iAYrevDevide�j�A
%                            �x����̈ʒu�Ɣ��a�ɂ��`���`�iAYrevStep,AYrevYandRadii�j
%                            ����͂Ƃ���B(AYrevStep�͔��a���w�肵�Ă���ʒu�̒i��)
%
%  function [TildeA]=TILDE(A);%
%            �R�~�P�s�� R �ɊO�σI�y���[�^����p���������s������֐��B
%            TILDE(R)=[0 -R(3) R(2);R(3) 0 -R(1);-R(2) R(1) 0]
%
%  function [C]=EtoC(E);%
%            �I�C���[�p�����[�^ E �����]�s������֐��B(E0=E(1),E1=E(2),E2=E(3),E3=E(4)�Ƃ���)
%            E2C(E)=[E1*E1-E2*E2-E3*E3+E0*E0 2(E1*E2-E3*E0)          2(E3*E1+E2*E0);
%                    2(E1*E2+E3*E0)          E2*E2-E3*E3-E1*E1+E0*E0 2(E2*E3-E1*E0);
%                    2(E3*E1-E2*E0)          2(E2*E3+E1*E0)          E3*E3-E1*E1-E2*E2+E0*E0]
%
%  function [S]=EtoS(E);%
%            �I�C���[�p�����[�^ E ����r�s������֐��B(E0=E(1),E1=E(2),E2=E(3),E3=E(4)�Ƃ���)
%            E2S(E)=[-E1  E0  E3 -E2;
%                    -E2 -E3  E0  E1;
%                    -E3  E2 -E1  E0]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all;%                          �S�Ă̐}�����
clear all;%                          ���[�N�X�y�[�X�̃N���A
%
global OUTPUT;%                     �O���t�p�o�͕ϐ��̎�n���ɗ��p����ϐ�
global ANMPUT;%                     �A�j���p�o�͕ϐ��̎�n���ɗ��p����ϐ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ���i�K
%  �V�~�����[�V�����ɗp���鐔�l�p�����[�^�̐ݒ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �R�}�Ɗ��̃p�����[�^
%
g=980.;%                            �d�͉����x
Ma=32*pi;%                          ����
Joa=[128*pi 0      0;%
     0      256*pi 0;%
     0      0      128*pi];%        �d�S�܂��̊����s��
Rap=[0;%
    -3;%
     0];%�@ �@�@�@�@�@�@�@�@   �@�@ �@�x�_�̈ʒu
Cxz=0;%                             �w���A�y���܂��̊p���x�����ɑ΂���x�_�܂��̉�]�_���s���O�W���@
Cyy=0;%                             �x���܂��̊p���x�����ɑ΂���x�_�܂��̉�]�_���s���O�W��
%
%  �I�C���[�p�����[�^���艻�̂��߂̎��萔
%
TAU=0.1;%                           �I�C���[�p�����[�^���艻�̂��߂̎��萔
%
%  ��������
%
EoaINITIAL=[cos(pi/12);%
            0;%
            0;%
           -sin(pi/12)];%           ������]�p���i�I�C���[�p�����[�^�j
OmoaINITIAL=[  0;%
             113.369;%
               0];%                 �����p���x
%
%  �V�~�����[�V��������
%
t0=0;%                              �v�Z�J�n����
dt=0.01;%                           �v�Z���ʏo�̓L�U�~
tf=6;%                              �v�Z�I������
%
%  �A�j���p�̌`��@  
%      �x���܂��̉�]�݂̂̂�����
%      NumDevide,NumStep�̗v�f���͉�]�̂̐�
%      NumDevide�͉�]�����̕������i�R�ȏ�j
%      NumStep�͉�]�̌`��̒i���i�P�ȏ�j
%      YandRadius�ɂ͂��ׂẲ�]�̉�]�̂̂��ׂĂ̒i�ɑ΂���
%                     �x���W�l�Ƃ��̈ʒu�ł̔��a���^�����Ă���
%      ���a���[���̏ꍇ�͂x����Ɉ�̓_�������
%      ���a���[���ȊO�̏ꍇ�͉~����ɕ����������̓_�������
%      ��]�̒[���̔��a�������̏ꍇ�A���̒[�ʂɂ��p�b�`�𒣂�
%      ��]�̒[���̔��a�������̏ꍇ�A���a�Ƃ��Ă͂��̐�Βl��p����
%      ��]�̒[���̔��a�������̏ꍇ�A���̒[�ʂɂ̓p�b�`�𒣂�Ȃ�
%      ��]�̒[���ȊO�̔��a�̐����͖��Ӗ��ł���
%
AYrevnumDevide =[16];%             �x���܂����]�̕�����
AYrevnumStep   =[7];%              �x���ɉ����āA���a���w�肷��i��
AYrevYandRadii =[-3   0;%
                 -2.5 0.5;%
                 -1   0.5;%
                 -0.5 3.5;%
                  0.5 3.5;%
                  0.5 0.5;%
                  2   0.5];%        �x���ɉ������ʒu�ƁA���̈ʒu�ł̔��a
%
%  �O���t�p�o�͂ɑ΂��AANMSKIP�Ɉ��̊����ŃA�j���o�͂����B
%
ANMaxis=[-5 5 -1 5 -5 5];%         �A�j���\���̈�
ANMskip = 3;%                      �O���t�o�͂ɑ΂���A�j���o�͂̍팸����
AfaceColor = 'yellow';%            �`���\�����p�`�̖ʂ̐F
AedgeColor = 'red';%               �`���\�����p�`�̃G�b�W�̐F
ANMfps = 33;%                      ��b�ԓ�����̃A�j����ʕ\����
ANMview = [1 0 0];%                �A�j���\���p�J�����̕���
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ���i�K
%  ��������
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
YINITIAL=[ EoaINITIAL;%
          OmoaINITIAL];%           ��ԕϐ������l�i�����I�C���[�p�����[�^�Ə����p���x�j
Mag=Ma*g;%                         �d��
Foa=[0;%
    -Mag;%
     0];%                          ��p�́i�d�́j
TILDERap=TILDE(Rap);%              Rap�����������s��
pJoa=Joa+TILDERap'*Ma*TILDERap;%   �x�_�o�܂��̊����s��
CC=[Cxz 0   0;%
    0   Cyy 0;%
    0   0   Cxz];%                 �x�_�܂��̉�]�_���s���O�W���s��
%
%  �a�s��̏���
%
AM=zeros(9,9);%                    �X�~�X�s��
AM(1,1)=Ma;%                       ����̂R�~�R�����́A���ʂɂ��X�J���[�s��
AM(2,2)=Ma;%
AM(3,3)=Ma;%
AM(4:6,4:6)=Joa;%                  �����̂R�~�R�����́A�����s��
AM(7,1)=1;%                        �����́A�S�������̒萔�����i�P�ʍs��j
AM(8,2)=1;%
AM(9,3)=1;%
AM(1,7)=1;%                        �E��i�Ώ̈ʒu�j�́A���������̓]�u
AM(2,8)=1;%
AM(3,9)=1;%
SPAM=sparse(AM);%                  �萔�����ȊO�̓[���̂܂܂Ƃ��āA�a�s��ɕϊ�
BM=zeros(9,1);%                    �X�~�P�s��
BM(1:3)=Foa;%                      �㕔�̂R�~�P�ɂ́A�d�́i�萔�j
SPBM=sparse(BM);%                  �萔�����ȊO�̓[���̂܂܂Ƃ��āA�a�s��ɕϊ�
%                                  �搔�ȊO�̕����́A���v�Z�̒��Ŋe���Ԃ��Ƃɗ^������B
%                                  SPAM���W���s��Ƃ��ASPBM���E�ӂƂ���
%                                      �A���ꎟ�������������ƁA�����I�ɑa�s��p�̉�@�����s�����B
%
%  �A�j���p��������
%
[Avert,Aface]=ANMYrev(AYrevnumDevide,AYrevnumStep,AYrevYandRadii);%
%                                  �A�j���p�`����\�����鑽�p�`���_�̈ʒu�A�ʂ��\�����钸�_�̑g
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ��O�i�K
%  �����������̐ϕ��v�Z
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
[tt,YY]=ode45(@e_koma_1045,[t0:dt:tf],YINITIAL,[],SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);%
%                                �@�@�R�}�̉^���̌v�Z
%                                    �o��tt�́At0����tf�܂�dt�L�U�~�̎��Ԃ���������s��in�s1��j
%                                    �o��YY�́A�e���ԂɑΉ������R�}�̃I�C���[�p,
%                                                                      �p���x���܂ލs��in�s7��j
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ��l�i�K
%  �o�͏���
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
numOUTtable=size(tt,1);%                 �o�͎��Ԃ̐�
OUTtable=[];%                            �O���t�p�o�̓e�[�u���̏���
ANMtable=[];%                            �A�j���p�o�̓e�[�u���̏���
ANMcount=0;%                             �A�j���o�͐���̂��߂̃J�E���^�[
for i=1:numOUTtable;%                    �o�͎��Ԃ̏���
    t=tt(i);%                            ���Ԃ�
    Y=YY(i,:)';%                         ��ԕϐ�������
    DY=e_koma_1045(t,Y,...;%
        SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);% ������x�����������̉E�ӂ̌Ăяo��
    OUTtable=[OUTtable;OUTPUT];%         �o�̓e�[�u���ɏo�͂�ǉ�
    ANMcount=ANMcount+1;%                �A�j���o�͐���J�E���^�[���P��������
    if mod(ANMcount-1,ANMskip)==0;%      �A�j���o�͐���J�E���^�[����P���w���������l��ANMskip�̔{���̂Ƃ�
        ANMtable=[ANMtable;ANMPUT];%         �A�j���p�o�̓e�[�u���ɏo�͂�ǉ�
    end;%
end;%
%
%  �O���t�o��
%
figure(1);%                          �v�Z���ʏo�͐}�P������
plot(tt,OUTtable(:,1));%                       ���ԁ|�R�}�d�S�ʒu��X���W��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�d�S�ʒu��X���W�icm�j');%
figure(2);%                          �v�Z���ʏo�͐}�Q������
plot(tt,OUTtable(:,2));%                       ���ԁ|�R�}�d�S�ʒu��Y���W��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�d�S�ʒu��Y���W�icm�j');%
figure(3);%                          �v�Z���ʏo�͐}�R������
plot(tt,OUTtable(:,3));%                       ���ԁ|�R�}�d�S�ʒu��Z���W��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�d�S�ʒu��Z���W�icm�j');%
%
figure(4);%                          �v�Z���ʏo�͐}�S������
plot(OUTtable(:,3),OUTtable(:,1));%            �R�}�d�S�̂y���W�|�R�}�d�S�̂w���W��`��
xlabel('�R�}�d�S�̂y���W�icm�j');%
ylabel('�R�}�d�S�̂w���W�icm�j');%
%
figure(5);%                          �v�Z���ʏo�͐}�T������
plot(tt,YY(:,1));%                              ���ԁ|�I�C���[�p�����[�^�P��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�P');%
figure(6);%                          �v�Z���ʏo�͐}�U������
plot(tt,YY(:,2));%                              ���ԁ|�I�C���[�p�����[�^�Q��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�Q');%
figure(7);%                          �v�Z���ʏo�͐}�V������
plot(tt,YY(:,3));%                              ���ԁ|�I�C���[�p�����[�^�R��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�R');%
figure(8);%                          �v�Z���ʏo�͐}�W������
plot(tt,YY(:,4));%                              ���ԁ|�I�C���[�p�����[�^�S��`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�S');%
%
figure(9);%                          �v�Z���ʏo�͐}�X������
plot(tt,OUTtable(:,4));%                       ���ԁ|�R�}�^���ʂ�X���W������`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�^���ʂ�X���W�����ikg*cm/s�j');%
figure(10);%                          �v�Z���ʏo�͐}�P�O������
plot(tt,OUTtable(:,5));%                       ���ԁ|�R�}�^���ʂ�Y���W������`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�^���ʂ�Y���W�����ikg*cm/s�j');%
figure(11);%                          �v�Z���ʏo�͐}�P�P������
plot(tt,OUTtable(:,6));%                       ���ԁ|�R�}�^���ʂ�Z���W������`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�^���ʂ�Z���W�����ikg*cm/s�j');%
%
figure(12);%                          �v�Z���ʏo�͐}�P�Q������
plot(tt,OUTtable(:,7));%                       ���ԁ|�R�}�p�^���ʂ�X���W������`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�p�^���ʂ�X���W�����ikg*cm^2/s�j');%
figure(13);%                          �v�Z���ʏo�͐}�P�R������
plot(tt,OUTtable(:,8));%                       ���ԁ|�R�}�p�^���ʂ�Y���W������`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�p�^���ʂ�Y���W�����ikg*cm^2/s�j');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     �c���́A�����X�P�[���ŏ��l��0.9�{�`�����X�P�[���̍ő�l��1.1�{
figure(14);%                          �v�Z���ʏo�͐}�P�S������
plot(tt,OUTtable(:,9));%                       ���ԁ|�R�}�p�^���ʂ�Z���W������`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�p�^���ʂ�Z���W�����ikg*cm^2/s�j');%
%
figure(15);%                          �v�Z���ʏo�͐}�P�T������
plot(tt,OUTtable(:,10));%                       ���ԁ|�R�}�^���G�l���M�[��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�^���G�l���M�[�ikg*cm^2/s^2�j');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     �c���́A�����X�P�[���ŏ��l��0.9�{�`�����X�P�[���̍ő�l��1.1�{
figure(16);%                          �v�Z���ʏo�͐}�P�U������
plot(tt,OUTtable(:,11));%                       ���ԁ|�R�}�|�e���V�����G�l���M�[��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�|�e���V�����G�l���M�[�ikg*cm^2/s^2�j');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     �c���́A�����X�P�[���ŏ��l��0.9�{�`�����X�P�[���̍ő�l��1.1�{
figure(17);%                          �v�Z���ʏo�͐}�P�V������
plot(tt,OUTtable(:,12));%                       ���ԁ|�R�}�S�G�l���M�[��`��
xlabel('���ԁi�b�j');%
ylabel('�R�}�S�G�l���M�[�ikg*cm^2/s^2�j');%
tmpaxis=axis;axis([tmpaxis(1),tmpaxis(2),tmpaxis(3)*0.9,tmpaxis(4)*1.1]);%
%                     �c���́A�����X�P�[���ŏ��l��0.9�{�`�����X�P�[���̍ő�l��1.1�{
figure(18);%                          �v�Z���ʏo�͐}�P�W������
plot(tt,OUTtable(:,13));%                       ���ԁ|�I�C���[�p�����[�^�̍S����`��
xlabel('���ԁi�b�j');%
ylabel('�I�C���[�p�����[�^�̍S��');%
%
%  �A�j���o��
%
numANMtable = size(ANMtable,1);%     �A�j���p�o�̓e�[�u���̃X�e�b�v��
numAvert  = size(Avert,1);%          �R�}�̌`��̒��_�̐�
figure(19);%                         �o�͐}�P�V������
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
    M(1)=getframe;%�@�@�@�@�@�@�@�@�@ �@�@�p�b�`�𒣂��āA�t���[������荞��
%
for i=2:numANMtable;%                    ������Ƃ̌J��Ԃ�
    ANMPUT=ANMtable(i,:);%               �A�j���p�o�̓e�[�u������̎��o��
    Roa=ANMPUT(1:3)';%                   Roa�i�d�S�ʒu�j�̎��o��
    Coa=reshape(ANMPUT(4:12),3,3);%      Coa�i��]�s��j�̎��o��
    Avertex=[];%                         ���W�n�n�ŕ\�����R�}�̒��_�̈ʒu����肱�ލs��̏���
    for j=1:numAvert;%                   ���ׂĂ̌`��ߓ_�ɂ���
        Roj=Roa+Coa*Avert(j,:)';%�@�@�@�@�@�@ ���̈ʒu���v�Z���A
        Avertex=[Avertex;Roj'];%             ���������s��ɕ��ׂ�B
    end;%
    set(h,'Vertices',Avertex);%          ���_���̕ύX
    drawnow;%                            �`��
    M(i)=getframe;%                      ��ʂ̎�荞��
end;%
movie(M,0,ANMfps);%                      ��荞�񂾑S��ʂ̍Đ�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ����������
%  �E�ӂ��v�Z����֐�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function DY=e_koma_1045(t,Y,SPAM,SPBM,Ma,Mag,Joa,pJoa,Rap,TILDERap,CC,TAU);%
%
global OUTPUT;%
global ANMPUT;%
%
Eoa  = Y(1:4);%                                ��ԗʂ���I�C���[�p�����[�^�𒊏o
Omoa = Y(5:7);%                                ��ԗʂ���p���x�𒊏o
%
Coa = EtoC(Eoa);%                              ��]�s��
Soa = EtoS(Eoa);%                              �I�C���[�p�����[�^�̎��Ԕ����Ɗp���x�̊֌W�̂R�~�S�W���s��
%
Nop = -CC*Omoa;%                               �x�_�o�ɓ�����]�����g���N
Noa = Nop;%                                    �d�S�`�֓������Z���Ă������l
TILDEOmoa = TILDE(Omoa);%                      �p���xOmoa�����������s��
%
SPAM(7:9,4:6) = -Coa*TILDERap;%                �a�s��SPAM�̓��I�ɕϓ�����̕���
SPAM(4:6,7:9) = SPAM(7:9,4:6)';%               ���̑Ώ̕���
SPBM(4:6) = Noa-TILDEOmoa*(Joa*Omoa);%         �a�s��SPBM�̓��I�ɕϓ�����̕���
SPBM(7:9) = Coa*(TILDEOmoa*(TILDERap*Omoa));%  �a�s��SPBM�̓��I�ɕϓ�����̕���
SPXM = SPAM\SPBM;%
%
DEoa  = Soa'*Omoa*0.5-Eoa*(0.5*(1-1/(Eoa'*Eoa))/TAU);%  �I�C���[�p�����[�^�̎��Ԕ����i�{�S�����艻�j
DOmoa = full(SPXM(4:6));%                      �p���x�̎��Ԕ���
DY = [DEoa;DOmoa];%                            ��ԗʂ̎��Ԕ���
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �o�͂̂��߂̌v�Z
%
Roa = -Coa*Rap;%                               �d�S�ʒu
Voa = Coa*(TILDERap*Omoa);%                    �d�S���x
Poa = Ma*Voa;%                                 �^����
PIoa = Coa*(pJoa*Omoa);%                       �p�^���ʁi�������W�n�\���j
Ta = Omoa'*pJoa*Omoa*0.5;%                     �^���G�l���M�[
Ua = Roa(2)*Mag;%                              �ʒu�G�l���M�[
TUa = Ta+Ua;%                                  �S�G�l���M�[
%
OUTPUT = [Roa(1),Roa(2),Roa(3),...,%
          Poa(1),Poa(2),Poa(3),PIoa(1),PIoa(2),PIoa(3),Ta,Ua,TUa,...;%
          Eoa'*Eoa-1];%                        �O���t�p�o�͂�OUTPUT�ւ̊i�[
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
ANMPUT = [Roa',Coa(:)'];%                      �O���t�p�o�́i�d�S�ʒu�Ɖ�]�s��j��ANMPUT�ւ̊i�[
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  �x���܂���]�̂̒��_�Ɩʂ̏������֐�
%
function [Vert,Face] = ANMYrev(NumDevide,NumStep,YandRadii);%
%
%  ���̃v���O�����̏ڂ��������͏ȗ�����B
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
