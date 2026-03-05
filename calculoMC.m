function Mc = calculoMC(nc, ic, Nack, NPTmax, Mi, R, percent)
    euler = 2.718281;
    
    if nc < 1 || ic < 1
        Mc = 0;
    else
        Msumrow = 0;
        for n = 1:NPTmax
            Msumrow = Msumrow + Mi(ic, n);
        end
        
        pn = calculopn(nc, percent);
        
        Xi = 0;
        for x = 1:NPTmax
            Xi = Xi + (Mi(ic, x) * pn * euler^(-Msumrow / R));
        end
        
        fone = Mi(ic, nc) * pn * euler^(-Msumrow / R);
        
        if Xi <= Nack
            Ms = fone;
        else
            Ms = (Nack * fone) / Xi;
        end
        
        Mc = Mi(ic, nc) - Ms;
    end
end
