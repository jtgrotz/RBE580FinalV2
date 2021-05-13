function A = dhparamtomatrix(t,d,a,alpha)

A = [cosd(t) -sind(t)*cosd(alpha) sind(t)*sind(alpha) a*cosd(t);...
    sind(t) cosd(t)*cosd(alpha) -cosd(t)*sind(alpha) a*sind(t);... 
    0 sind(alpha) cosd(alpha) d; 0 0 0 1];

end

    

