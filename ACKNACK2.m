function [alocados] = ACKNACK2(alocados, p_tra)
    Preambulo = zeros(1, length(alocados));

    for ii = 1 : length(alocados)
        Preambulo(ii) = alocados{ii}.Preambulo;
    end

    for k = 1 : length(Preambulo)
        num_repetitions = sum(p_tra == Preambulo(k));

        if num_repetitions == 2
            alocados{k}.ACKNACK = 2; % 'Class 2'
        elseif num_repetitions == 3
            alocados{k}.ACKNACK = 3; % 'Class 3'
        elseif num_repetitions == 4
            alocados{k}.ACKNACK = 4; % 'Class 4'
        elseif num_repetitions == 5
            alocados{k}.ACKNACK = 5; % 'Class 5'
        %elseif num_repetitions >= 14
         %   alocados{k}.ACKNACK = 0; % 'Class 0'
        else
            x = double(randi(10, 1, 1) > 1); % 10% probability of misclassification
            if x == 1
                alocados{k}.ACKNACK = 1; % 'Class 1 (unique)'
            else
                alocados{k}.ACKNACK = 2; % 'Class 2 (misclassified)'
            end
        end
    end
end
