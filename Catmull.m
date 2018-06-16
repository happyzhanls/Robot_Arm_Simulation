function Func_Catumull = Catmull(s,P_1,P_2,P_3,P_4)

    Func_Catumull = zeros(1,3,length(s));
    
    for i = 1:length(s)
        Matrix = [-0.5  1.5  -1.5  0.5;...
                   1   -2.5   2   -0.5;...
                  -0.5   0    0.5   0;...
                   0     1    0     0;];
        Func_Catumull(:,:,i) = [s(i,1)^3,s(i,1)^2,s(i,1),1]*Matrix*[P_1;P_2;P_3;P_4];
    end
end