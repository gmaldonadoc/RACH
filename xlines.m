function xlines (bins)
% label = 'bins';
    for i = 1:64
        if i == 1
            xline(bins(i),'-k','BINS');
        else
            xline(bins(i),'--k');
        end
    end
end
 