function [coord_x, coord_y] = obtcoordenadas(bs_x, bs_y, radius)
    outside = 1;
    R = radius;
    while outside
         x_temp = -R + 2 * R * rand;  % Generate x within [-R, R]
         y_temp = -R + 2 * R * rand;  % Generate y within [-R, R]
         distance = sqrt(x_temp^2 + y_temp^2);  % Calculate distance from the origin
         
         if distance <= R  % Check if the point is within the circle
             outside = 0;
         end
    end
    coord_x = x_temp + bs_x;
    coord_y = y_temp + bs_y;
end



