function P_Enviado = sucess(P_Enviado)
    [unique_values, ~, unique_indices] = unique(P_Enviado, 'stable');
    P_Enviado = unique_values;
end





