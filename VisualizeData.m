function  VisualizeData(xmin,xmax, Xf_m,U_m, Delta_C,C)

S_Delta_C=[];
for i=1:length(Delta_C)
S_Delta_C=[S_Delta_C, sum(Delta_C(:,1:i),2)];
end
[N D]=size(Xf_m);
for i=1:D;
    subplot(D,1,i)
    plot(S_Delta_C,Xf_m(:,i))
    hold on
    xmax_i=xmax(i)*ones(1,N);
    xmin_i=xmin(i)*ones(1,N);
    plot(S_Delta_C,xmax_i)
    hold on
    plot(S_Delta_C,xmax_i)
    hold on
end
maxC=max(C);
minC=min(C);
TmaxC=maxC==C;
TminC=minC==C;
TothC=minC~=C&maxC~=C;

[N M]=size(U_m);
figure
for j=1:M;
    subplot(M,1,j)
    UmaxC=TmaxC.*U_m(:,j)';
    UminC=TminC.*U_m(:,j)';
    UothC=TothC.*U_m(:,j)';
    bar(UmaxC,'r')
    hold on
    bar(UminC,'g')
    hold on
    bar(UothC,'b')
end