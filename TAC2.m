function datos_cells = TAC2(detected, offsets)
    speedOfLight = 299792458.0; % Speed of light in meters per second    
    datos_cells = cell(1, length(offsets)); % Preallocate the cell array

    for ck = 1:length(offsets)
        offsets_min = floor(offsets(ck));
        NTA = ((offsets_min) / 1920000); % Convert samples to time
        Distance = (speedOfLight* NTA) / 2;
        Distance = floor(Distance);

        datos = struct('Preambulo', detected(ck), 'TA', Distance); % Create the structure
        datos_cells{ck} = datos; % Assign the structure to the cell array
    end
end


