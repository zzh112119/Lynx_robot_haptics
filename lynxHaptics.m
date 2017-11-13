% Fill this script in with your position tracking, force computation,
% and graphics for the virtual environment

close all

%% Run on hardware or simulation
hardwareFlag = false;

%% Plot end effector in environment
global qs % configuration (NOTE: This is only 3 angles now)
global posEE % position of end effectr
global btnPressed %boolean array indicating stastus of each button
global velocity %velocity of end effector

figClosed = 0;
qs = [0,0,0]; % initialize robot to zero pose
posEE = [0,0,0];  % initialize position of end effector
posEE_old = computeEEposition();

hold on; scatter3(0, 0, 0, 'kx', 'Linewidth', 2); % plot origin
h1 = scatter3(0, 0, 0, 'r.', 'Linewidth', 2); % plot end effector position
h2 = quiver3(0, 0, 0, 0, 0, 0, 'b'); % plot output force
if ~hardwareFlag
    h_fig = figure(1);
    set(h_fig, 'Name','Haptic environment: Close figure to quit.' ,'KeyPressFcn', @(h_obj, evt) keyPressFcn(h_obj, evt));
end

%% Create Environment here:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create static objects and interactive objects in their initial state

syms Fn v 

Env_1=[200 200 200 200; -300 -300 300 300; -300 300 300 -300];
Env={Env_1};    %Environment Cell
Text_1.area = [200 200 200 200; -50 -50 50 50; -50 50 50 -50];



Obs={};         %Obstacle Cell

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set camera properties
axis([-1000 1000 -1000 1000 -1000 1000]);
view([75,30]);

i = 0; frameSkip = 3; % plotting variable - set how often plot updates
while(1)
    %% Read potentiometer values, convert to angles and end effector location
    if hardwareFlag
        qs = lynxGetAngles();
    end
    
    %% Calculate current end effector position
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    posEE = computeEEposition();
    velocity = posEE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Calculate desired force based on current end effector position
    % Check for collisions with objects in the environment and compute the total force on the end effector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F = computeForces();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Plot Environment
    if i == 0
        figClosed = drawLynx(h1, h2, F);
        
        for j=1:1:length(Env)
            
            fill3(Env{1,j}(1,:),Env{1,j}(2,:), Env{1,j}(3,:),[0.7 0 0], 'facealpha', 0.3);

        end
       
        drawnow
        hold on;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute torques from forces and convert to currents for servos
    Tau = computeTorques();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if hardwareFlag
        if figClosed % quit by closing the figure
            lynxDCTorquePhysical(0,0,0,0,0,0);
            return;
        else
            currents = torquesToCurrents(Tau);
            lynxDCTorquePhysical(currents(1),currents(2),currents(3),0,0,0);
        end
    end
    
    if (figClosed) % quit by closing the figure
        return;
    end
    
    %% Debugging
    %[posEE, qs, F', Tau']
    i = mod(i+1, frameSkip);
end