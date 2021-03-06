function [posEst,oriEst,driftEst, posVar,oriVar,driftVar,estState] = Estimator(estState,actuate,sense,tm,estConst)
% [posEst,oriEst,driftEst, posVar,oriVar,driftVar,estState] = 
%   Estimator(estState,actuate,sense,tm,estConst)
%
% The estimator.
%
% The function will be called in two different modes:
% If tm==0, the estimator is initialized; otherwise the estimator does an
% iteration step (compute estimates for the time step k).
%
% Inputs:
%   estState        previous estimator state (time step k-1)
%                   May be defined by the user (for example as a struct).
%   actuate         control input u(k), [1x2]-vector
%                   actuate(1): u_v, drive wheel angular velocity
%                   actuate(2): u_r, drive wheel angle
%   sense           sensor measurements z(k), [1x3]-vector, INF if no
%                   measurement
%                   sense(1): z_d, distance measurement
%                   sense(2): z_c, compass measurement
%                   sense(3): z_g, gyro measurement
%   tm              time, scalar
%                   If tm==0 initialization, otherwise estimator
%                   iteration step.
%   estConst        estimator constants (as in EstimatorConstants.m)
%
% Outputs:
%   posEst          position estimate (time step k), [1x2]-vector
%                   posEst(1): x position estimate
%                   posEst(2): y position estimate
%   oriEst          orientation estimate (time step k), scalar
%   driftEst        estimate of the gyro drift b (time step k), scalar
%   posVar          variance of position estimate (time step k), [1x2]-vector
%                   posVar(1): x position variance
%                   posVar(2): y position variance
%   oriVar          variance of orientation estimate (time step k), scalar
%   driftVar        variance of gyro drift estimate (time step k), scalar
%   estState        current estimator state (time step k)
%                   Will be input to this function at the next call.
%
%
% Class:
% Recursive Estimation
% Spring 2017
% Programming Exercise 1
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Raffaello D'Andrea, Michael Muehlebach, Lukas Hewing
% michaemu@ethz.ch
% lhewing@ethz.ch
%
% --
% Revision history
% [19.04.11, ST]    first version by Sebastian Trimpe
% [30.04.12, PR]    adapted version for spring 2012, added unknown wheel
%                   radius
% [06.05.13, MH]    2013 version
% [23.04.15, MM]    2015 version
% [14.04.16, MM]    2016 version
% [05.05.17, LH]    2017 version


%% Mode 1: Initialization
if (tm == 0)
    % Do the initialization of your estimator here!
    
    % Replace the following:
    posEst = [0 0];    
    oriEst = 0;    
    driftEst = 0;
    posVar = [(estConst.TranslationStartBound)^2 (estConst.TranslationStartBound)^2]*1/3;
    oriVar = ((estConst.RotationStartBound)^2)*1/3;
    driftVar = 0;
    estState = [posEst oriEst driftEst posVar oriVar driftVar tm];

    return;
end


%% Mode 2: Estimator iteration.
% If we get this far tm is not equal to zero, and we are no longer
% initializing.  Run the estimator.

%Read previous iteration values
prevX = estState(1);
prevY = estState(2);
prevOri = estState(3);
prevDrift = estState(4);

prevXVar = estState(5);
prevYVar = estState(6);
prevOriVar = estState(7);
prevDriftVar = estState(8);

time_step = tm - estState(9);

Uv = actuate(1);
Ur = actuate(2);
B = estConst.WheelBase;
W = estConst.WheelRadius;
cNoise = estConst.CompassNoise;
gNoise = estConst.GyroNoise;
dNoise = estConst.DistNoise;
Qb = estConst.GyroDriftPSD;
Qv = estConst.VelocityInputPSD;

%Prior update/Prediction step
%Mean:
% tspan = [tm-time_step tm];
% x0 = [prevX prevY prevOri];
% [~,predMean] = ode45(@(t,x) odefcn(t,x, estConst, estState, actuate), tspan, x0);
% 
% predX = predMean(end,1);
% predY = predMean(end,2);
% predOri = predMean(end,3);

Sv = W*Uv;
St = Sv*cos(Ur);
Sr = -Sv*sin(Ur)/B;

predX = St/Sr*sin(Sr*tm-Sr*(tm-time_step)+prevOri) - St/Sr*sin(prevOri) + prevX;
predY = -St/Sr*cos(Sr*tm-Sr*(tm-time_step)+prevOri) + St/Sr*cos(prevOri) + prevY;
predOri = Sr*(time_step)+prevOri;
predDrift = prevDrift;

xp = [predX
    predY
    predOri
    predDrift];

%Variace:
% tspan = [tm-time_step tm];
% x0 = [prevXVar
%       prevYVar
%       prevOriVar];
% [~,predVar] = ode45(@(t,x) odefcn1(t,x, estConst, estState, actuate, tm), tspan, x0);

% predXVar = predVar(end,1);
% predYVar = predVar(end,2);
% predOriVar = predVar(end,3)

r = Sr*(time_step) + prevOri;
predXVar = Qv*St^2/2*(  time_step + sin(2*r)/(2*Sr) - sin(2*prevOri)/(2*Sr)  ) + prevXVar;
predYVar = Qv*St^2/2*(  time_step - sin(2*r)/(2*Sr) + sin(2*prevOri)/(2*Sr)  ) + prevYVar;
predOriVar = Qv*( Sr^2*time_step ) + prevOriVar;
predDriftVar = Qb*(time_step) + prevDriftVar;
Pp = [predXVar 0 0 0
    0 predYVar 0 0
    0 0 predOriVar 0
    0 0 0 predDriftVar];

%Estimation Update:
H = [0 0 1 0
     0 0 1 1
     predX*(predX^2 + predY^2)^(-1/2) predY*(predX^2 + predY^2)^(-1/2) 0 0];

M = eye(3);

R = diag([cNoise gNoise dNoise]);

K = Pp * H' / (H*Pp*H' + M*R*M');

hk = [predOri;
    predOri + predDrift;
    (predX^2 + predY^2)^(1/2)];

%Mean:
conversion_sense = sense;
sense(1) = conversion_sense(2);
sense(2) = conversion_sense(3);
sense(3) = conversion_sense(1);

if sense(1) == Inf
    sense(1) = predOri;
end
if sense(2) == Inf
    sense(2) = predOri + predDrift;
end
if sense(3) == Inf    
    sense(3) = (predX^2 + predY^2)^(1/2);
end
xm = xp + K * (sense'-hk);

%Variance:
Pm = (eye(4) - K*H)*Pp;

% Replace the following:
posEst = [xm(1) xm(2)];
oriEst = xm(3);
driftEst = xm(4);
posVar = [Pm(1,1) Pm(2,2)];
oriVar = Pm(3,3);
driftVar = Pm(4,4);
estState = [posEst oriEst driftEst posVar oriVar driftVar tm];

end

function dydt = odefcn(t, x, estConst, estState, actuate)
prevX = estState(1);
prevY = estState(2);
prevOri = estState(3);
prevDrift = estState(4);

prevXVar = estState(5);
prevYVar = estState(6);
prevOriVar = estState(7);
prevDriftVar = estState(8);

Uv = actuate(1);
Ur = actuate(2);
B = estConst.WheelBase;
W = estConst.WheelRadius;

Sv = W*Uv;
St = Sv*cos(Ur);
Sr = -Sv*sin(Ur)/B;

dydt = zeros(3,1);
dydt(1) = St*cos(x(3));
dydt(2) = St*sin(x(3));
dydt(3) = Sr;
end

function dydt = odefcn1(t, x, estConst, estState, actuate, tm)
prevX = estState(1);
prevY = estState(2);
prevOri = estState(3);
prevDrift = estState(4);

prevXVar = estState(5);
prevYVar = estState(6);
prevOriVar = estState(7);
prevDriftVar = estState(8);

time_step = tm - estState(9);

Uv = actuate(1);
Ur = actuate(2);
B = estConst.WheelBase;
W = estConst.WheelRadius;
Qv = estConst.VelocityInputPSD;

Sv = W*Uv;
St = Sv*cos(Ur);
Sr = -Sv*sin(Ur)/B;
r = Sr*(t-(tm-time_step)) + prevOri;

% dydt = zeros(3);

% A = [0 0 -St*sin(r)
%      0 0 St*cos(r)
%      0 0 0];
 
L = [St*cos(r)
     St*sin(r)
     Sr]*Qv^(1/2);
 
% dydt = A*x + x*A' + L*Qv*L';
dydt = zeros(3,1);
dydt(1) = L(1)^2;
dydt(2) = L(2)^2;
dydt(3) = L(3)^2;
end