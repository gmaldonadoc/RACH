function [datos] = alocated(prbs, datos)
    pp = zeros(1, numel(datos));
    for ck = 1:numel(datos)
        pp(ck) = datos{ck}.Preambulo;
    end

    randomNumbers = randi(prbs, [1, numel(pp)]);

    for ca = 1:numel(datos)
        datos{ca}.Prb = randomNumbers(ca); % Assign random value from randomNumbers
    end
end
