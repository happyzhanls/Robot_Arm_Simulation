function Part2()
format long
object_main = importdata('object.key');
% [length,width] = size(object_main);
main_frame = object_main(1,1);
total_frame = object_main(1,2);
rotation_main = zeros(3,3,main_frame);
position_main = zeros(1,3,main_frame+2);

%%%%% record n
n = 2;  
for j = 1:main_frame   
    for i = 1:3
        rotation_main(i,1,j) = object_main(n,1);
        n = n+2;
    end
end
%%%% record o
n = 2;
for j = 1:main_frame
    for i = 1:3
        rotation_main(i,2,j) = object_main(n,2);
        n = n+2;
    end
end
%%%%% record a
n = 3;
for j = 1:main_frame
    for i = 1:3
        rotation_main(i,3,j) = object_main(n,1);
        n = n+2;
    end
end

n = 3;
for j = 2:main_frame+1
    for i = 1:3
        position_main(1,i,j) = object_main(n,2);
        n = n+2;
    end
end

%%%%% NOTE:P0=P1 P(n+1)=P(n)
position_main(:,:,1) = position_main(:,:,2);
position_main(:,:,main_frame+2) = position_main(:,:,main_frame+1);

%%%%% Position Function
step_size = (main_frame-1)/(total_frame-1);

    j = 1;
    add_size = 0;
    total_step = zeros(total_frame,1);
    point_part = zeros(main_frame-1,1);
    Position_Catmull = zeros(1,3,total_frame);
    for i = 2:total_frame
        add_size = add_size + step_size;
        total_step(i,1)= add_size;
        if abs(add_size - j) < 0.0000001
            if j == 1 
                point_part(j,1) = i;
                j = j+1;
            else 
                point_part(j,1) = i-sum(point_part(1:(j-1),1));
                j = j+1;    
            end
        elseif add_size > j
            if j == 1 
                point_part(j,1) = i-1;
                j = j+1;
            else 
                point_part(j,1) = i-1-sum(point_part(1:j-1,1));
                j = j+1;    
            end
        end
    end
    j=2;
    for i = 1:main_frame-1
        if i == 1
            Position_Catmull(:,:,1:point_part(i,1))=Catmull(total_step(1:point_part(i,1)),position_main(:,:,j-1),...
                position_main(:,:,j),position_main(:,:,j+1),position_main(:,:,j+2));
        else
            start_point = sum(point_part(1:i-1,1))+1;
            end_point = sum(point_part(1:i,1));
            Position_Catmull(:,:,start_point:end_point)=Catmull((total_step(start_point:end_point)-(i-1)),...
                position_main(:,:,j-1),position_main(:,:,j),position_main(:,:,j+1),position_main(:,:,j+2));
        end
     j = j+1;
    end

Quaternion_main = zeros(main_frame+2,4);
for j = 1:main_frame
    Quaternion_main(j+1,:) = R_to_Q(rotation_main(:,:,j)) ;
end
    Quaternion_main(1,:) = Quaternion_main(2,:);
    Quaternion_main(main_frame+2,:)=Quaternion_main(main_frame+1,:);
    Quaternion_total = Q_interpolation(Quaternion_main,main_frame,total_frame);

Rotation_total = zeros(3,3,total_frame);
for i = 1:total_frame
    Rotation_total(:,:,i) = Q_to_R(Quaternion_total(i,:));
end

%     for i =1:total_frame
%         Rotation_total(:,:,i) = [1 0 0; 0 1 0; 0 0 1;];
%     end
    %%%%%%%%%%%%%%%% write robot.ang
file = fopen('object/object.traj','wt');
fprintf(file,'%g\n\n',total_frame);
for i = 1:total_frame
    for j = 1:3
        fprintf(file,'%g\t',Rotation_total(j,1,i));
        fprintf(file,'%g\t',Rotation_total(j,2,i));
        fprintf(file,'%g\t',Rotation_total(j,3,i));
        fprintf(file,'%g\n',Position_Catmull(1,j,i));
    end
    fprintf(file,'\n');
end
fclose(file);

end
    