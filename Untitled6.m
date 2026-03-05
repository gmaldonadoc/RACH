function  Untitled6 (pdp, picos, bestCorr, threshold, cyclicShift)
    figure;
    stem(pdp, 'b'); % Plot the entire PDP signal
    hold on;
%     stem(picos, bestCorr, 'ro', 'MarkerSize', 8); % Plot peaks with red circles
     yline(threshold,'--r','Threshold','LineWidth',1)
    xlines(cyclicShift)
    xlabel('Sample Index');
    ylabel('Correlation Value');
    title('Peaks and Correlation Values');
    legend('PDP Signal',  'Threshold', 'Bins');
    hold off;
end