% Column 1 (Equipment):1 is COM10,9600,0x50(cc4e56be-6cf7-401e-901a-d2c91c55186c)   
% Column 2:Chip Time   The interval (in seconds) between each piece of data and the start time, with the start time as the starting point
% Column 3:Temperature(℃)
% Column 4:Acceleration X(g)
% Column 5:Acceleration Y(g)
% Column 6:Acceleration Z(g)
% Column 7:X acceleration amplitude(g)
% Column 8:Y acceleration amplitude(g)
% Column 9:Z acceleration amplitude(g)
% Column 10:X velocity amplitude(mm/s)
% Column 11:Y velocity amplitude(mm/s)
% Column 12:Z velocity amplitude(mm/s)
% Column 13:X displacement amplitude(um)
% Column 14:Y displacement amplitude(um)
% Column 15:Z displacement amplitude(um)
% Column 16:X frequency vibration frequency(Hz)
% Column 17:Y frequency vibration frequency(Hz)
% Column 18:Z frequency vibration frequency(Hz)
% Column 19:(g)
% Column 20:(g)
% Column 21:(g)
% Column 22:(g)
% Column 23:(g)
% Column 24:
% Column 25:(g)
% Column 26:
% Column 27:(g)
% Column 28:(mm/s)
% Column 29:(g)
% Column 30:(mm)
% Column 31:(g)
% Column 32:(g)
% Column 33:(g)
% Column 34:(g)
% Column 35:(g)
% Column 36:
% Column 37:(g)
% Column 38:
% Column 39:(g)
% Column 40:(mm/s)
% Column 41:(g)
% Column 42:(mm)
% Column 43:(g)
% Column 44:(g)
% Column 45:(g)
% Column 46:(g)
% Column 47:(g)
% Column 48:
% Column 49:(g)
% Column 50:
% Column 51:(g)
% Column 52:(mm/s)
% Column 53:(g)
% Column 54:(mm)
% Column 55:
% Column 56:
% Column 57:
% Column 58:
% Column 59:
% Column 60:
% Column 61:
% Column 62:
% Column 63:
% Column 64:
% Column 65:
% Column 66:
% Column 67:
% Column 68:
% Column 69:
% Column 70:
% Column 71:
% Column 72:
% Column 73:
% Column 74:
% Column 75:
% Column 76:
% Column 77:
% Column 78:
% Column 79:
% Column 80:
% Column 81:
% Column 82:
% Column 83:
% Column 84:
% Column 85:
% Column 86:
% Column 87:
% Column 88:
% Column 89:
% Column 90:
% Column 91:
% Column 92:
% Column 93:
% Column 94:
% Column 95:
% Column 96:
% Column 97:
% Column 98:
% Column 99:
% Column 100:
% Column 101:
% Column 102:
% 函数调用：a=readMatData;
function d = readMatData(file)

    if nargin<1
        disp('默认数据')
        file='data.mat';
    else
        disp(file);
    end

    disp('加载mat文件')
    load('data.mat')
    S=whos;
    len = length(S)-1;
    dend = eval(S(len).name);
    d1 = eval(S(1).name);
    len_m = length(d1);
    len_n = length(d1(1,:));

    d=zeros(len_m*(len-1)+length(dend),len_n);
    %h=waitbar(0,'数据合并中……');
    for i=1:len-1
        dTemp = eval(S(i).name);
        d(len_m*(i-1)+1:len_m*i,:)=[dTemp];
        m=len-1;
        %p=fix(i/(m)*len_m)/100; %这样做是可以让进度条的%位数为2位
        %str=['正在合并，目前进度为 ',num2str(p),' %，完成 ',num2str(i),'/',num2str(m)];%进度条上显示的内容
        %waitbar(i/m,h,str);
    end
    d(len_m*(len-1)+1:len_m*(len-1)+length(dend),:)=dend;

end