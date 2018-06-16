% Perform Bezier interpolation between quaternions

function total_q=Q_interpolation(key_q,key_frame,total_frame)
    step_size=(key_frame-1)/(total_frame-1);
    ai=zeros(key_frame,4);
    bii=zeros(key_frame-1,4);
    for i=2:key_frame+1
        ai(i-1,:)=Bisect(Double(key_q(i-1,:),key_q(i,:)),key_q(i+1,:));
% 		ai(i,:)=slerp(key_q(i,:),ai(i,:),1.0/3.0);      
    end
    for i = 2:key_frame
        bii(i-1,:)=Double(ai(i,:),key_q(i+1,:)); 
    end

    total_q=zeros(total_frame,4);
    segment = 1;
    u = 0:step_size:key_frame-1;
    u = u';
        for i = 1:total_frame
            if u(i) < segment
                C = slerp(key_q(segment+1,:),ai(segment,:),u(i)-(segment-1));
                D = slerp(ai(segment,:),bii(segment,:),u(i)-(segment-1));
                E = slerp(bii(segment,:),key_q(segment+2,:),u(i)-(segment-1));
                F = slerp(C,D,u(i)-(segment-1));
                G = slerp(D,E,u(i)-(segment-1));
                total_q(i,:) = slerp(F,G,u(i)-(segment-1));
            else 
                segment = segment+1;
                if segment == key_frame
                    segment = segment -1;
                    C = slerp(key_q(segment+1,:),ai(segment,:),u(i)-(segment-1));
                    D = slerp(ai(segment,:),bii(segment,:),u(i));
                    E = slerp(bii(segment,:),key_q(segment+2,:),u(i)-(segment-1));
                    F = slerp(C,D,u(i)-(segment-1));
                    G = slerp(D,E,u(i)-(segment-1));
                    total_q(i,:) = slerp(F,G,u(i)-(segment-1));
                else 
                    C = slerp(key_q(segment+1,:),ai(segment,:),u(i)-(segment-1));
                    D = slerp(ai(segment,:),bii(segment,:),u(i));
                    E = slerp(bii(segment,:),key_q(segment+2,:),u(i)-(segment-1));
                    F = slerp(C,D,u(i)-(segment-1));
                    G = slerp(D,E,u(i)-(segment-1));
                    total_q(i,:) = slerp(F,G,u(i)-(segment-1));
                end
            end
        end
end