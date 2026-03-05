function [x_peak_positions, y_peak_positions] = find_multiple_peaks(pdp, threshold, min_peak_distance)
    % Preasignar para lograr eficiencia
    x_peak_positions = [];
    y_peak_positions = [];

    % Detectar todos los picos que superen el umbral
    for i = 1:length(pdp) - 1
        if pdp(i) > threshold && pdp(i) > pdp(i-1) && pdp(i) > pdp(i+1)
            % Detectar un pico
            x_peak_positions(end+1) = i;
            y_peak_positions(end+1) = pdp(i);
        end
    end

    % Aplicar la restricción de distancia mínima entre picos (delay spread)
    [x_peak_positions, y_peak_positions] = enforce_min_peak_distance(x_peak_positions, y_peak_positions, min_peak_distance);
end

function [x_filtered, y_filtered] = enforce_min_peak_distance(x_peaks, y_peaks, min_distance)
    x_filtered = [];
    y_filtered = [];

    if isempty(x_peaks)
        return;
    end

    % Inicializar con el primer pico
    x_filtered(end+1) = x_peaks(1);
    y_filtered(end+1) = y_peaks(1);
    last_peak = x_peaks(1);

    % Recorrer los picos detectados y aplicar la restricción de distancia mínima
    for i = 2:length(x_peaks)
        if abs(x_peaks(i) - last_peak) > min_distance
            x_filtered(end+1) = x_peaks(i);
            y_filtered(end+1) = y_peaks(i);
            last_peak = x_peaks(i);
        end
    end
end
