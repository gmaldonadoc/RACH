function pn = calculopn(nc, percent)
    euler = 2.718281; % Approximate value of Euler's number
    if percent == 0.99
        pn = 0.99;
    else
        pn = 1 - euler^(-nc);
    end
end
