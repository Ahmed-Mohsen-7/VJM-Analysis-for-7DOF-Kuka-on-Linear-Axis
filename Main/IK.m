
function [q_final]=IK(goal,q_start,link_lengths)
%{
#goal is a 6 by 1 vector of desired goal
#q_start contains 7 joint angles of robot
#example: IK([-1.3038; 1.1741 ;0.5832;0;0;0],[0,0,0,0,0,0,0])
# it returns 7 joint angles needed to reach the position.
%}
q_0 = q_start;

 

%% Visualizing the home position
[T, ~, ~, ~, ~, ~, ~] = FK(q_0, link_lengths);


phi_x = atan2(T(3,1),T(3,2));
phi_z = atan2(T(1,3),-T(2,3));
phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));

cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

count = 1;


%% Getting the pseudo inverse
while norm(goal(1:3) - cur_pos(1:3)) > 1e-01
        % [q, q_dot] = PseudoInverse(q_0, link_lengths, p_global, 0);
        [q, ~] = Damped_LS(q_0, link_lengths, goal);
        %[q, q_dot] = Null_Space(q_0, link_lengths, p_global, 0);
        %[q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global);
        norm(goal(1:3) - cur_pos(1:3));
        [T, ~, ~, ~, ~, ~, ~] = FK(q, link_lengths);
%         if norm(old(1:3) - cur_pos(1:3)) < 1.1e-01
%           Visualize_Robot(T, T1, T2, T3, T4, T5, T6, color_list{3}, 1)
%           pause(0.1);
%         end
%         if norm(old(1:3) - cur_pos(1:3)) > 70.1e-03
%             if norm(old - p_global) ~= 0
%               Visualize_Robot(T, T1, T2, T3, T4, T5, T6, color_list{3}, 0)
%               pause(0.1);
%             end
%             old = cur_pos;
%         end
        phi_x = atan2(T(3,1),T(3,2));
        phi_z = atan2(T(1,3),-T(2,3));
        phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
        cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

        q_0 = q;
        count = count + 1;
end
% end
q_final=q_0;

end
