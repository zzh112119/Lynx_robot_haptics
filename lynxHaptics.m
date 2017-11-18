% Fill this script in with your position tracking, force computation,
% and graphics for the virtual environment

close all

% Run on hardware or simulation
hardwareFlag = true;

% Plot end effector in environment
global qs % configuration (NOTE: This is only 3 angles now)
global posEE % position of end effectr
global BtnFlag %boolean array indicating stastus of each button
global velocity %velocity of end effector
global deltaT
global posobst
%global posEE_obs % position of end effector is there's force applied

BtnFlag=0;
figClosed = 0;
qs = [0,0,0]; % initialize robot to zero pose
posEE = [0,0,0];  % initialize position of end effector
posEE_old = computeEEposition();

hold on; scatter3(0, 0, 0, 'kx', 'Linewidth', 2); % plot origin
h1 = scatter3(0, 0, 0, 'r.', 'Linewidth', 2); % plot end effector position
h2 = quiver3(0, 0, 0, 0, 0, 0, 'b'); % plot output force
%h3 = scatter3(0, 0, 0, 'b','Linewidth',2);
if ~hardwareFlag
    h_fig = figure(1);
    set(h_fig, 'Name','Haptic environment: Close figure to quit.' ,'KeyPressFcn', @(h_obj, evt) keyPressFcn(h_obj, evt));
end
L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;  

% Create Environment here:
% Create static objects and interactive objects in their initial state

syms Fn v pos;

% Define environment
xmin = 150;
xmax = 350;
ymin = -200;
ymax = 200;
zmin = 100;
zmax = 300;
Env_1 = [xmin xmin xmin xmin; ymin ymin ymax ymax; zmin zmax zmax zmin];
Env_3 = [xmax xmax xmax xmax; ymin ymin ymax ymax; zmin zmax zmax zmin];
Env_4 = [xmin xmin xmax xmax; ymin ymax ymax ymin; zmax zmax zmax zmax];
% Env_1=[150 150 150 150; -1300 -1300 1300 1300; 200 1300 1300 200];
% Env_2 = [150 150 150 150; -1300 -1300 0 0; -1300 200 200 -1300];
%Env_3 = [150 150 150 150; 0 0 1300 1300; -1300 0 0 -1300];
Env={};   

% Define texture areas 
Text_1.area = [xmin xmin xmin xmin; ymin ymin ymax ymax; zmin zmax zmax zmin];
Text_2.character = -10 * Fn .* v ./1000 + 0 * pos;
Text_2.area = [xmin xmin xmax xmax; ymax ymax ymax ymax; zmin zmax zmax zmin];
Text_1.character = (-0.5 * Fn + sum(-10 .* sin(pos))) .* v ./100;
texts = {};

% Define att/rep points
pts_1 = struct('pos', [250;0;200], 'isattract', 1, 'strength', 0.1);
%scatter3(250,200,200,'b.')
%hold on;
pts = {pts_1};

% Define buttons
%btn_1.area = [xmin xmin xmax xmax; ymin ymin ymin ymin; zmin zmax zmax zmin];
btn_1.area = [xmin xmin xmin xmin; ymin ymin ymax ymax; zmin zmax zmax zmin];
btn_1.c = 0.05;
btns = {btn_1};

% Define Obstacles
obsts_1.pos = [250; 0; 200];
obsts_1.mass = 0.01;
obsts_1.r = 1;
obsts_1.v = [-0;-0;0];
obsts = [obsts_1];         

h3 = scatter3(0, 0, 0, 10 * obsts(1).r, 'ro', 'filled');

% set camera properties
axis([50 400 -300 300 0 400]);
view([75,30]);

i = 0; frameSkip = 3; % plotting variable - set how often plot updates
time_old = cputime;

while(1)
    % Read potentiometer values, convert to angles and end effector location
    if hardwareFlag
        qs = lynxGetAngles();
    end
    
    Jv = computeJacobian(qs(1), qs(2), qs(3), L1, L2, L3);

    posEE_obs=posEE;

    % Calculate current end effector position
    
    posEE = computeEEposition();
    time_cur = cputime;
    if time_cur - time_old > 0.0001
        deltaT = time_cur - time_old;
        velocity = (posEE' - posEE_old') ./ deltaT;
        posEE_old = posEE;
        time_old = time_cur;
    end
        
    % Calculate desired force based on current end effector position
    % Check for collisions with objects in the environment and compute the total force on the end effector

    [F, obsts] = computeForces(Env, texts, obsts, btns, pts);
    
    
    % Compute torques from forces and convert to currents for servos
    [Tau, Tauflag] = computeTorques(Jv,F);
    %Tau
%     if Tauflag
%         scatter3(posEE_obs(1),posEE_obs(2),posEE_obs(3),'b.');
%     end
        
    % Plot Environment
    if i == 0
        if length(obsts) > 0
            posobst = obsts(1).pos;
        end
         figClosed = drawLynx(h1, h2, h3, F);
 %       figClosed = drawLynx(h1, h2, F);
        
        for j=1:1:length(Env)          
            fill3(Env{1,j}(1,:),Env{1,j}(2,:), Env{1,j}(3,:),[0.7 0 0], 'facealpha', 0.3);
        end
       
        for j = 1 : length(texts)
            fill3(texts{j}.area(1, :), texts{j}.area(2, :), texts{j}.area(3, :) ,[0 1-0.25*j 0.25*j], 'facealpha', 0.3)
        end
        
        for j = length(texts) + 1 : length(texts) + length(btns)
            fill3(btns{j-length(texts)}.area(1, :), btns{j-length(texts)}.area(2, :), btns{j-length(texts)}.area(3, :) ,[0 1-0.25*j 0.25*j], 'facealpha', 0.3)
            fill3(btns{j-length(texts)}.area(1, :)-[40 40 40 40], btns{j-length(texts)}.area(2, :), btns{j-length(texts)}.area(3, :) ,[1 1-0.25*j 0.25*j], 'facealpha', 0.3)
        end
        drawnow
%         hold on;
    end
    
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
    
    % Debugging
    %[posEE, qs, F', Tau']
    i = mod(i+1, frameSkip);
end