% Rotation matrix around y-axis

function Ry=Ry(alpha)
Ry=[cos(alpha), 0, sin(alpha);
    0,1,0;
    -sin(alpha), 0,cos(alpha)];
end
