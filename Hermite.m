function Func_Hermite = Hermite(s,Pi,Pii,Di,Dii)

    Func_Hermite = zeros(1,length(s));
    for i = 1:length(s)
        Matrix = [ 2 -2  1  1;...
                  -3  3 -2 -1;...
                   0  0  1  0;...
                   1  0  0  0;];
        Func_Hermite(1,i) = [s(i,1)^3,s(i,1)^2,s(i,1),1]*Matrix*[Pi;Pii;Di;Dii];
    end
end
