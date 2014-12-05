function  VisualizeData(xmin,xmax, Xf_m,U_m, Delta_C,C,Energy)


[N D]=size(Xf_m);
figure('name','Deposits final levels');
for i=1:D;
    subplot(D,1,i)
    bar(Xf_m(:,i))
    str = sprintf('Deposit %d',i);
    title(str);
    ylabel('Volume[V]');
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
figure('name','Pumps running time');
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
    str = sprintf('Pump %d',j);
    title(str);
    ylabel('Time[T]');
end

% [N]=size(Energy);
% figure('name','Total of energy consumed per period');
%     subplot(M,1,1)
%     Energy_maxC=TmaxC.*Energy';
%     Energy_minC=TminC.*Energy';
%     Energy_othC=TothC.*Energy';
%     bar(Energy_maxC,'r')
%     hold on
%     bar(Energy_minC,'g')
%     hold on
%     bar(Energy_othC,'b')
%     hold off
%     ylabel('Energy[P]');
    
    