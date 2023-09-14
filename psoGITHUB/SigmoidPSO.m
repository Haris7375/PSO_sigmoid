clear all
close all
clc
figure
%% Initial values
x = linspace(-10,10,30);
y = linspace(-10,10,30);

%for particle movement 1
% set(gca,'XLimMode','manual');
% set(gca,'YLimMode','manual');
% axis([-15,15,-15,15])
% hold on
% view(2)
% shading interp

% No of Particles
Particles =35;
nVar = 2;
% Objective function details
Objectivefunction = @schwefel;
lb = -100  ; % Lower bound
ub = 100 ;   % upper bound


%% PSO paramters
Max_it = 1000;
V_max=6;
% Acceleration Coefficents limits
a=0.000035;            % Inertia Weight
b = 0.005;
c= 0;
d = 1.2;
% Inertia weight
W_max=0.8;
W_min=0.2;


%% Particle movement
sum=zeros(1,1000);
iter=1000;
for ii=1:length(iter)


cg_curve = zeros(1,Max_it);
x = -10:4:10;
y= x;
idx = 1;
show_vel = 1;
for t1 = 1: length (x)
    for t2 =  1: length (y)
        Swarm.Particles(idx).X = [x(t1) y(t2)];
        idx = idx+1;
    end
end


%% Initializations
for k = 1: Particles
    Swarm.Particles(k).V = rand(1,nVar);
    PBEST = rand(1,nVar)*V_max;
    Swarm.Particles(k).PBEST.O= inf; % for minimization problems
    h(k) = plot(Swarm.Particles(k).X(1),Swarm.Particles(k).X(2),'ok', 'markerFaceColor','k')
    set(h(k),'EraseMode','normal')
    if show_vel == 1
        p1 = [Swarm.Particles(k).X(1) Swarm.Particles(k).X(2)];                         % First Point
        p2 = [Swarm.Particles(k).V(1) Swarm.Particles(k).V(2)];                         % Second Point
        dp = p2-p1/100;                         % Difference
        
        h2(k) = plot([p1(1) dp(1)],[p1(2) dp(2)],'-k' , 'LineWidth',2.5)
        set(h2(k),'EraseMode','normal')
    end
end
GBEST=zeros(1,nVar);
Swarm.GBEST.O= inf;
for t=1:Max_it % main loop
    for k=1:Particles
        %Calculate objective function for each particle
        Swarm.Particles(k).O=Objectivefunction( Swarm.Particles(k).X );
        
        if(Swarm.Particles(k).O < Swarm.Particles(k).PBEST.O)
            Swarm.Particles(k).PBEST.O = Swarm.Particles(k).O;
            PBEST = Swarm.Particles(k).X;
        end
        if(Swarm.Particles(k).O < Swarm.GBEST.O)
            Swarm.GBEST.O = Swarm.Particles(k).O;
            GBEST = Swarm.Particles(k).X;
        end
    end
 R1 = rand(1,0);
 
 R2 = rand(1, 0);   

 %Update the inertia weight
    w=W_max -((W_max-W_min)*k/Max_it);   
% Sigmoid  section     
    % G =    PBEST - Swarm.Particles(k).X;
     %H =    GBEST -Swarm.Particles(k).X;
     %D = H;
     c1 = b./ (1 + abs(a))+1.35;
     %F_G = (b)./(1+exp(-a*(G-c))) +d;   
    c2 = a./ (1 + abs(b))+1.35;
     %F_H = (b)./(1+exp(-a*(H-c))) +d;   
    
     %c1 = (F_G);
    % c2 = (F_H);
    
    
%Update the Velocity and Position of particles
   
for k=1:Particles
        first_vel(k,:) = Swarm.Particles(k).V;
        Swarm.Particles(k).V  = w .* Swarm.Particles(k).V +   ...  % inertia
            c1 .* rand(1,nVar) .* (PBEST - Swarm.Particles(k).X ) +  ...   % congnitive
            c2 .* rand(1,nVar).* (GBEST - Swarm.Particles(k).X) ;  % social
        
        second_vel(k,:) = Swarm.Particles(k).V;
             
        first_loc(k,:) = Swarm.Particles(k).X ;
        Swarm.Particles(k).X = Swarm.Particles(k).X + Swarm.Particles(k).V;
        second_loc(k,:) = Swarm.Particles(k).X ;
        

        % particle Moivement 2
%         nn = 20;
%         moving_x(k,:) =  linspace(first_loc(k,1),second_loc(k,1),nn);
%         moving_y(k,:) = linspace(first_loc(k,2),second_loc(k,2),nn);
%         
%         moving_vx(k,:) =  linspace(first_vel(k,1),second_vel(k,1),nn);
%         moving_vy(k,:) = linspace(first_vel(k,2),second_vel(k,2),nn);
        
    end
    
    % Particle Movement 3
%          for rr = 1: nn
%          for JJ = 1:Particles;
%              set(h(JJ),'XData', moving_x(JJ,rr),'YData', moving_y(JJ,rr));
%              
%              if show_vel == 1
%                  p1 = [moving_x(JJ,rr) moving_y(JJ,rr)];                         % First Point
%                  p2 = [moving_x(JJ,end) moving_y(JJ,end)];
%                  dp = (p2+p1)/2;                         % Difference
%                  dp = (p1+dp)/2;
%                  set(h2(JJ),'XData', [p1(1), dp(1)],'YData',  [p1(2), dp(2)]);
%              end
%          end
%          
%          drawnow
%      end
    
  
    cg_curve(t) = Swarm.GBEST.O;
    Swarm.GBEST.O
end
sum=sum+cg_curve;
end
result=sum./length(iter);
figure

%plot(cg_curve, 'LineWidth',2)
semilogy(cg_curve,'LineWidth',2);
hold on
semilogy(result,'LineWidth',2);
xlabel('Iteration')
ylabel('Best cost')
grid on;