function [x_peak_positions, y_peak_positions] = find_peaks_above_threshold(pdp, threshold)
    % Preasignar para lograr eficiencia
    x_peak_positions = zeros(64, 1);
    y_peak_positions = zeros(64, 1);
    peak_count = 0;
    
    window_length = 23.90; 
    total_windows = 64;
    min_peak_distance = 7; % Distancia mínima entre picos detectados
    
    window_start = 1; % Inicializar la posición de inicio de la primera ventana
    pdp_length = length(pdp); % Precalcular la longitud del pdp para lograr eficiencia

    for i = 1:total_windows
        window_end = window_start + window_length - 1;
        
        if window_end > pdp_length
            window_end = pdp_length;
        end
        
        window_pdp = pdp(window_start:window_end);
        [peak_value, peak_index_within_window] = find_highest_peak_within_window(window_pdp, threshold);

        if ~isempty(peak_index_within_window)
            peak_position = window_start - 1 + peak_index_within_window;
            [x_peak_positions, y_peak_positions, peak_count] = merge_peak(x_peak_positions, y_peak_positions, peak_position, peak_value, min_peak_distance, peak_count);
        end
        
        window_start = window_end + 1; % Pasar a la siguiente ventana sin superposición
    end
    
    % Eliminar espacio preasignado no utilizado
    x_peak_positions = x_peak_positions(1:peak_count);
    y_peak_positions = y_peak_positions(1:peak_count);
end

function [peak_value, peak_index] = find_highest_peak_within_window(window_pdp, threshold)
    [peak_value, peak_index] = max(window_pdp);

    if peak_value <= threshold
        peak_index = [];
    end
end

function [x_combined, y_combined, peak_count] = merge_peak(x_existing, y_existing, x_new, y_new, min_distance, peak_count)
    if peak_count == 0
        peak_count = 1;
        x_combined = x_new;
        y_combined = y_new;
        return;
    end

    distances = abs(x_existing(1:peak_count) - x_new);
    [min_distance_val, min_distance_idx] = min(distances);
    
    if min_distance_val <= min_distance
        if y_new > y_existing(min_distance_idx)
            x_existing(min_distance_idx) = x_new;
            y_existing(min_distance_idx) = y_new;
        end
    else
        peak_count = peak_count + 1;
        x_existing(peak_count) = x_new;
        y_existing(peak_count) = y_new;
    end

    x_combined = x_existing;
    y_combined = y_existing;
end

% function [x_peak_positions, y_peak_positions] = find_peaks_above_threshold(pdp, threshold)
%     x_peak_positions = [];
%     y_peak_positions = [];
%     
%     window_length = 23.90;%23.7997616209774
%     total_windows = 64;
%     min_peak_distance = 7; % Minimum distance between detected peaks
%     
%     window_start = 1; % Initialize the first window start position
% 
%     for i = 1:total_windows
%         window_end = window_start + window_length - 1;
%         
%         if window_end > length(pdp)
%             window_end = length(pdp);
%         end
%         
%         window_pdp = pdp(window_start:window_end);
% 
%         [peak_value, peak_index_within_window] = find_highest_peak_within_window(window_pdp, threshold);
% 
%         if ~isempty(peak_index_within_window)
%             peak_position = window_start - 1 + peak_index_within_window;
%             [x_peak_positions, y_peak_positions] = merge_peak(x_peak_positions, y_peak_positions, peak_position, peak_value, min_peak_distance);
%         end
%         
%         window_start = window_end + 1; % Move to the next window without overlap
%     end
% end
% 
% function [peak_value, peak_index] = find_highest_peak_within_window(window_pdp, threshold)
%     [peak_value, peak_index] = max(window_pdp);
% 
%     if peak_value <= threshold
%         peak_index = [];
%     end
% end
% 
% function [x_combined, y_combined] = merge_peak(x_existing, y_existing, x_new, y_new, min_distance)
%     x_combined = x_existing;
%     y_combined = y_existing;
% 
%     is_duplicate = false;
% 
%     for j = 1:length(x_existing)
%         if abs(x_new - x_existing(j)) <= min_distance
%             is_duplicate = true;
% 
%             if y_new > y_existing(j)
%                 x_combined(j) = x_new;
%                 y_combined(j) = y_new;
%             end
% 
%             break;
%         end
%     end
% 
%     if ~is_duplicate
%         x_combined = [x_combined; x_new];
%         y_combined = [y_combined; y_new];
%     end
% end
