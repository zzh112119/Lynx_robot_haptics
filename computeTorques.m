% Convert end effector forces into joint torques
% For 10V, and i = [1,2,1].*tau, we have tauLim = [0.7,0.5,0.9];

% Fill in the necessary inputs
function Tau = computeTorques(Jv,F)

Tau = Jv'*F;

if Tau(1)>0.7
    Tau(1)=0.7;
    disp("Tau 1 max");
end

if Tau(1)<-0.7
    Tau(1)=-0.7;
    disp("Tau 1 min");
    
end

if Tau(2)>0.5
    Tau(2)=0.5;
    disp("Tau 2 max");
end

if Tau(2)<-0.5
    Tau(2)=-0.5;
    disp("Tau 2 min");    
end

if Tau(3)>0.9
    Tau(3)=0.9;
    disp("Tau 3 max");    
end

if Tau(3)<-0.9
    Tau(3)=-0.9;
    disp("Tau 3 min");    
end

end