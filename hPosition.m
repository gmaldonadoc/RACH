%hPositioningPlotPositions Plot eNodeB positions
%   hPositioningPlotPositions(ENB) plots the positions of eNodeBs in the
%   cell array ENB.

%   Copyright 2011-2020 The MathWorks, Inc.

function hPosition(prach, radio)
    figure;
    hold on;
    
    % Prepare legend strings
    legendstr = cell(1, numel(prach) + 2);  % +2 for 'Radio de celda' and 'eNB'
    
    % Plot the positions of UEs
    for u = 1:numel(prach)
        plot(prach{u}.Position(1), prach{u}.Position(2), '*', 'MarkerSize', 7, 'LineWidth', 3);
        legendstr{u} = sprintf('UE %d (PreambleIdx = %d)', u, prach{u}.PreambleIdx);
        str = {sprintf('UE %d', u), sprintf('PreambleIdx = %d', prach{u}.PreambleIdx)};
        text(prach{u}.Position(1), prach{u}.Position(2), str);
    end
    
    % Plot the cell radius
    t = linspace(0, 2 * pi);
    x1 = radio * cos(t) + 4;
    y1 = radio * sin(t) + 3;
    plot(x1, y1, 'LineWidth', 2, 'MarkerSize', 10);
    
    % Plot the eNodeB
    plot(0, 0, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', '#D95319', 'LineWidth', 2);
    legendstr{numel(prach) + 1} = 'Radio de celda';
    legendstr{numel(prach) + 2} = 'eNB';
    
    % Display the legend and labels
    legend(legendstr);
    xlabel('Position X');
    ylabel('Position Y');
    title(['\fontsize{16} Distribution of '  ,num2str(numel(prach)),  ' UEs and 1 eNB']);
%     grid on;
    
    hold off;
end

