function  VisualizeData(xmin,xmax, Xf_m,U_m, Delta_C,C)


[N D]=size(Xf_m);
figure
for i=1:D;
    subplot(D,1,i)
    bar(Xf_m(:,i))
    hold on
    xmax_i=xmax(i)*ones(1,N);
    xmin_i=xmin(i)*ones(1,N);
    plot([1:N],xmin_i,'r')
    hold on
    plot([1:N],xmax_i,'r')
    hold off
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
    hold off
end