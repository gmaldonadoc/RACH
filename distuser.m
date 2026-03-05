function [p] = distuser(user_no_perbs, bs, cell_rad)
    user = zeros(user_no_perbs, 2);  % Initialize user matrix
    for k = 1:user_no_perbs
        [coord_x, coord_y] = obtcoordenadas(bs(1, 1), bs(1, 2), cell_rad);
        user(k, 1) = coord_x;
        user(k, 2) = coord_y;
        [X, Y] = pol2cart(coord_x, coord_y);
        p = [X, Y];
    end
end
