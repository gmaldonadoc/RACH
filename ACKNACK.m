function [alocados]= ACKNACK (alocados,p_tra)
Preambulo = [];
for ii = 1 : length(alocados)
    Preambulo(ii) = alocados{ii}.Preambulo;

end

    for k = 1 : length (Preambulo)
        if sum(p_tra == Preambulo(k)) >=2
            % Repeated
            alocados{k}.ACKNACK= 1; %'NACKcolision
        else
            x = double(randi(10,1,1) > 1); % ACK no colision
            if x == 1
                x=0;
            else
                x=1;
            end
            alocados{k}.ACKNACK= x; 
        end
    end
end   