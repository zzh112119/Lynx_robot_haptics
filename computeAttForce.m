%q is the six joint angles, x is the position of goal, Spotential is the
%attractive field strength

function [Fa]=computeAttForce(q,Pg,Spotential)
    %q=[q1,q2,q3,q4,q5,q6];
    %Spotential=1; %assumes attractive field strength = 1
%     [q_goal,~]=IK_lynx([1,0,0,x(1);0,1,0,x(2);0,0,1,x(3);0,0,0,1]);
    [P0,~]=updateQ([q(1),q(2),q(3),q(4),q(5),q(6)]);
%     [Pg,~]=updateQ(q_goal);
    Fa=-Spotential*(P0-Pg);
    
%     Fa
end