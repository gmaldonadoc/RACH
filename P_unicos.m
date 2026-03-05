function [xi]=P_unicos(P_Enviado)
    [b,m1,~] = unique(P_Enviado,'first');
    [~,d1] =sort(m1);
    b = b(d1);
    xi= length(b);   
end
