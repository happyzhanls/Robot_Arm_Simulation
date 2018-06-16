function Part1()
format long
joint = 6;
robot_main = importdata('robot.key');
%  robot_ang = importdata('robot/robot.ang');
% [length,width] = size(robot_main);
main_frame = robot_main(1,1);
total_frame = robot_main(1,2);
robot_velocity = zeros(main_frame,joint);
robot_position = zeros(main_frame,joint);

n = 2;
for i = 1:main_frame
    robot_position(i,1) = robot_main(n,1);
    robot_position(i,2) = robot_main(n,2);
    robot_position(i,3) = robot_main(n+1,1);
    robot_position(i,4) = robot_main(n+1,2);
    robot_position(i,5) = robot_main(n+2,1);
    robot_position(i,6) = robot_main(n+2,2);
    n = n+6;
end

n = 5;
for i = 1:main_frame
    robot_velocity(i,1) = robot_main(n,1);
    robot_velocity(i,2) = robot_main(n,2);
    robot_velocity(i,3) = robot_main(n+1,1);
    robot_velocity(i,4) = robot_main(n+1,2);
    robot_velocity(i,5) = robot_main(n+2,1);
    robot_velocity(i,6) = robot_main(n+2,2);
    n = n+6;
end

step_size = (main_frame-1)/(total_frame-1);
joint = 6;
joint_position = zeros(total_frame,1,joint);

for h = 1: joint
    j = 1;
    add_size = 0;
    total_step = zeros(total_frame,1);
    point_part = zeros(main_frame-1,1);
    Position_Hermite = zeros(total_frame,1);
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
    for i = 1:main_frame-1
        if i == 1
            Position_Hermite(1:point_part(i,1),1)=Hermite(total_step(1:point_part(i,1)),robot_position(i,h),...
                robot_position(i+1,h),robot_velocity(i,h),robot_velocity(i+1,h));
        else
            start_point = sum(point_part(1:i-1,1))+1;
            end_point = sum(point_part(1:i,1));
            Position_Hermite(start_point:end_point,1)=Hermite((total_step(start_point:end_point)-(i-1)),...
                robot_position(i,h),robot_position(i+1,h),robot_velocity(i,h),robot_velocity(i+1,h));
        end
    end

    joint_position(:,1,h) = Position_Hermite;
end

%%%%%%%%%%%%%%%% write robot.ang
file = fopen('robot/robot.ang','wt');
fprintf(file,'%g\n',total_frame);
for i = 1:total_frame
    for h = 1:joint
        if h == joint
            fprintf(file,'%g\n',joint_position(i,1,h));
        else 
            fprintf(file,'%g\t',joint_position(i,1,h));
        end
    end   
end
fclose(file);
end


