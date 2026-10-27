%% Feedback control
%%% Definition %%%
% Position = distp --> the distance between the prismatic joint and the
% center


% INPUT
% Iteration, Position, dt, AdjDefpos

%%%% Strategy %%%%
% Calculate the offset between the ideal position and the real position,
% and alter the velocity so that the change in the velocity is directly
% proportional to the offset. Set 2 parameters: one for the lower bound and
% another for the upper bound of the adjustment.
% * The true upper bound of the adjustment is subject to change since the altered velocity
% should not go beyond the Maxspeed.

%%%% Input %%%%
% Position: N*6 matrix of position relative to the intial position
% Currpos: 1*6 matrix of the current position
% Iteration: the current iteration count

% The measurement of the current position (Currpos) at k-th iteration is assumed to be taken
% at the beginning of the lopp, corresponding to the k-th row of Position
% matrix in the ideal case.
% Keep in mind that array indexing in matlab and labview starts with 1
% and 0, respectively.


% bound=10;		% mm
% Iteration count = 0-N (N=size(Position,1)-1)
% --> the total number of iterations = N

N=size(Position,1)-1;
Iteration=Iteration+1;			% labview indexing starts from 0
% This iteration range: 1 - N+1
if Iteration>=N+1 % 2023/08/17 previous --> (if Iteration==N+1)
	Iteration=N;
end
Position=Position-Defpos;	% Redefining Position to be the relative position
Posoffset=Currpos-Position(Iteration,:);
Rawoffset=Posoffset;
Posoffset(Posoffset<-bound)=-bound;
Posoffset(Posoffset>bound)=bound;
dx=Position(mod((Iteration+2)-1,N)+1,:)-Position(mod((Iteration+1)-1,N)+1,:)-Posoffset;
pulseinterval=[];								% (usec) the precise interval at which arduino has to send pulses
direction=[];
volt=[];
abs_dx=abs(dx);
pulseinterval=(10^3)*(distrev/pulserev)*dt./abs_dx;
pulseinterval(pulseinterval>=TT)=TT;			% Making sure that pulseinterval does not exceed TT
pulseinterval(pulseinterval<tt)=tt;
volt=(5/(1024*ArdNI_ratio))*((B./pulseinterval)-A);
direction=sign(dx);
direction=(1/2)*(direction+1);
For_Iteration=mod(Iteration+1-1,N)+1-1;



% Subsequently, send the newly acquired volt and direction to the output
% subvi

% Assumptions: Currpos is sampled at the beginning of the k-th loop of
% the output VI (corresponding to the k-th row of the Position matrix).
% Even under the influence of noise, the motor is assumed to travel roughly the
% distance specified by the k-th row of volt (the problem is the
% accumulation of noise over multiple iterations; noise at each iteration
% is quite small). Since Currpos sampled in the k-th loop cannot be used to feedback
% control the same loop (corresponding to the transition from k --> k+1 row
% of Position matrix), it is used to feedback control the k+1 loop
% (corresponding to the transition from k+1 --> k+2 row of Position matrix)
%%
%%%%%%%%%%%%%%%%%%%%%%%%%
bound=10;
Position=distp;

% Iteration=size(Position,1)-2;
Iteration = 1;
Currpos=Position(Iteration+1,:)-AdjDefpos;
%%%%%%%%%%%%%%%%%%%%%%%%%


N=size(Position,1)-1;
Iteration=Iteration+1;			% labview indexing starts from 0
% This iteration range: 1 - N+1
if Iteration==N+1
	Iteration=N;
end
Position=Position-AdjDefpos;	% Redefining Position to be the relative position
Posoffset=Currpos-Position(Iteration,:);
Posoffset(Posoffset<-bound)=-bound;
Posoffset(Posoffset>bound)=bound;
adj_dx=Position(mod((Iteration+2)-1,N)+1,:)-Position(mod((Iteration+1)-1,N)+1,:)-Posoffset;
adj_pulseinterval=[];								% (usec) the precise interval at which arduino has to send pulses
adj_direction=[];
adj_volt=[];
abs_adj_dx=abs(adj_dx);
adj_pulseinterval=(10^3)*(distrev/pulserev)*dt./abs_adj_dx;
adj_pulseinterval(adj_pulseinterval>=TT)=TT;			% Making sure that pulseinterval does not exceed TT
adj_pulseinterval(adj_pulseinterval<tt)=tt;
adj_volt=(5/(1024*ArdNI_ratio))*((B./adj_pulseinterval)-A);
adj_direction=sign(adj_dx);
adj_direction=(1/2)*(adj_direction+1);
For_Iteration=mod(Iteration+1-1,N)+1-1;

%% Feedback simulation
bound=10;
N=size(Position,1)-1;
Position=distp-Defpos;	% Redefining Position to be the relative position
Currpos=zeros(N+1,6);

Noise=1;
RandMat=Noise*(rand(N+1,6)-0.5);
% RandMat=Noise*(normrnd(0,1,N+1,6));


for Iteration = 1:N+1
% This iteration range: 1 - N+1
% if Iteration==N+1
% 	Iteration=N;
% end
if Iteration ==1
	Currpos(Iteration,:)=Position(Iteration,:)+RandMat(Iteration,:);
else
	Currpos(Iteration,:)=next_dx+Currpos(Iteration-1,:)+RandMat(Iteration,:);
end
Posoffset=Currpos(Iteration,:)-Position(Iteration,:);
Posoffset(Posoffset<-bound)=-bound;
Posoffset(Posoffset>bound)=bound;
adj_dx=Position(mod((Iteration+2)-1,N)+1,:)-Position(mod((Iteration+1)-1,N)+1,:)-Posoffset;
adj_pulseinterval=zeros(1,6);								% (usec) the precise interval at which arduino has to send pulses
adj_direction=zeros(1,6);
adj_volt=zeros(1,6);
abs_adj_dx=abs(adj_dx);
adj_pulseinterval=(10^3)*(distrev/pulserev)*dt./abs_adj_dx;
adj_pulseinterval(adj_pulseinterval>=TT)=TT;			% Making sure that pulseinterval does not exceed TT
adj_pulseinterval(adj_pulseinterval<tt)=tt;
adj_volt=(5/(1024*ArdNI_ratio))*((B./adj_pulseinterval)-A);
adj_direction=sign(adj_dx);
adj_direction=(1/2)*(adj_direction+1);
next_dx=(10^3)*(distrev/pulserev)*dt./adj_pulseinterval.*sign(adj_dx);
Iteration
end

%%
bound=10;
N=size(Position,1)-1;
Position=distp-distp(1,:);
% Currpos(Iteration,:)=[-49.32,-7.128,42.864,54.72,-10.104,-43.296];
% Iteration = 29;
% Currpos(Iteration,:)=[0.816,0.792,2.088,-2.16,1.68,-2.88];
Iteration=60;
Currpos(Iteration,:)=[-28.944,-13.464,0.024,30.648,-2.736,-0.048];

Posoffset=Currpos(Iteration,:)-Position(Iteration,:);
Posoffset(Posoffset<-bound)=-bound;
Posoffset(Posoffset>bound)=bound;
adj_dx=zeros(1,6);	
adj_dx=Position(mod((Iteration+2)-1,N)+1,:)-Position(mod((Iteration+1)-1,N)+1,:)-Posoffset;
adj_pulseinterval=zeros(1,6);								% (usec) the precise interval at which arduino has to send pulses
adj_direction=zeros(1,6);
adj_volt=zeros(1,6);
abs_adj_dx=abs(adj_dx);
adj_pulseinterval=(10^3)*(distrev/pulserev)*dt./abs_adj_dx;
adj_pulseinterval(adj_pulseinterval>=TT)=TT;			% Making sure that pulseinterval does not exceed TT
adj_pulseinterval(adj_pulseinterval<tt)=tt;
adj_volt=(5/(1024*ArdNI_ratio))*((B./adj_pulseinterval)-A);
adj_direction=sign(adj_dx);
adj_direction=(1/2)*(adj_direction+1);
next_dx=(10^3)*(distrev/pulserev)*dt./adj_pulseinterval.*sign(adj_dx);
%% Parameters of the platform, NI, and Arduino

%%%% Execution step %%%% CHECK ARDUINO CODE
TT=100000;							% Max pulseinterval
tt=100;								% min pulseinterval

%%%% Execution step %%%%
% T needs to be divisible by dt!!!!!
% dt=20;								% (ms) the time it takes for labview to complete one loop %%%%% IF YOU CHANGE THIS, MAKE SURE TO CHANGE ARDUINO CODE AS WELL
dt=20;

%%%% Stepper motor constants %%%%
pulserev=1600;								% pulse/revolution
distrev=12;									% (mm) distance/revolution

%%%% Arduino constants %%%%
readinterval=20;					% (ms) interval between analogRead--> this should be equal to dt!!!
ArdNI_ratio=0.9918;					% NI voltage / Arduino voltage
% R=1023;
upbound=990;
Maxvolt=upbound;
minvolt=0;
A=(Maxvolt*tt-minvolt*TT)/(TT-tt);
B=TT*tt*(Maxvolt-minvolt)/(TT-tt);
Maxspeed=((distrev/pulserev)/tt)*1000*1000;	% (mm/second)

%%%% Stewart platform dimensions %%%%

cent=64.868+25-36+zeros(1,6);		% prismatic joints are counted from 1 to 6 in a counterclockwise order (looking from above)
qside=[30,30,30,30,30,30];
qz=47.723+zeros(1,6);

Hrdmrgn=5;							% (mm) (range of movement of the prismatic joints in practice) = (full range of movment) - (Hrdmrgn) 
qmin=6+9.206+30;							% (mm) true qmin   % 2023/08/18 added 30 due to rotary encoder
qmax=6+30+175+30-23.294 ;						% (mm) true qmax
qmin=qmin+Hrdmrgn;
qmax=qmax-Hrdmrgn;
q0min=qmin+zeros(1,6);				% available prismatic joint length
lgth=qmax-cent(1,1);

newqside=zeros(1,2);
for c=1:3
	newqside(1,2*c-1)=-qside(2*c-1);
	newqside(1,2*c)=qside(2*c);
end

% defining qi0 (foot of sliders)%%%%%%%%% IN THIS VERSION, THIS DEFINITION
% IS FIXED (PREVIOUS VERSIONS WERE INACCURATE; THEY ADDED q0min, WHICH SHIFTS THE ORGIN OF MEASUREMENT)
preq0=[cent;newqside;qz];
R1=[cos(2*pi/3),-sin(2*pi/3),0;sin(2*pi/3),cos(2*pi/3),0;0,0,1];
R2=[cos(-2*pi/3),-sin(-2*pi/3),0;sin(-2*pi/3),cos(-2*pi/3),0;0,0,1];
q0=zeros(3,6,1);
q0(:,:,1)=[preq0(:,[1,2]),R1*preq0(:,[3,4]),R2*preq0(:,[5,6])];

% defining the ball joint between the disk and the arm
r=85.869; 
degrees1=40.897;
degrees2=79.103;
radians1=deg2rad(degrees1);
radians2=deg2rad(degrees2);
ballangle=zeros(1,6,1);
ballangle0=[-radians1/2,radians1/2,radians1/2+radians2,radians1/2+radians2+radians1,2*pi-(radians1/2+radians2+radians1),2*pi-(radians1/2+radians2)];
ballangle(1,:,1)=ballangle0;

% distance between the disk surface and the center of balls
diskofst=10+15.458;

% prismatic joint vector
l=[];			% (3,6)
l(:,[1,3,5])=[1,0,0;cos(2*pi/3),sin(2*pi/3),0;cos(4*pi/3),sin(4*pi/3),0]';
l(:,[2,4,6])=[1,0,0;cos(2*pi/3),sin(2*pi/3),0;cos(4*pi/3),sin(4*pi/3),0]';


% arm length
L=196; % the radius of the magnetic ball is 0.5 cm

Achievable=1;
%% One way motion: from one position to another with BUFFER (OUTDATED)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=2000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
theta0=0;
phi0=0;
tau0=0;
alpha0=0;
beta0=0;
gamma0=230;

theta1=0;
phi1=10;
tau1=0;
alpha1=0;
beta1=0;
gamma1=230;
% theta and phi are in degrees

k=0.2;					% buffer size
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Define previousSTATE and nextSTATE --> contains variables for the position and angle
% theta0=previousSTATE(1);
% phi0=previousSTATE(2);
% tau0=previousSTATE(3);
% alpha0=previousSTATE(4);
% beta0=previousSTATE(5);
% gamma0=previousSTATE(6);
% 
% theta1=nextSTATE(1);
% phi1=nextSTATE(2);
% tau1=nextSTATE(3);
% alpha1=nextSTATE(4);
% beta1=nextSTATE(5);
% gamma1=nextSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;
theta1=(pi/180)*theta1;
phi1=(pi/180)*phi1;
tau1=(pi/180)*tau1;

if k>0.5
	k=0.5;
end

x0=[0:dt/T:1];
prey1=(1/2).*(x0.^2)./(k*(1-k));
prey1(x0>=k)=0;
prey2=k.*(x0-k/2)./(k*(1-k));
prey2(x0<k | x0>1-k)=0;
prey3=(k*(1-k)-(1/2).*((1-x0).^2))./(k*(1-k));
prey3(x0<=1-k)=0;
prey=prey1+prey2+prey3;

theta=zeros(1,1,T/dt+1);
phi=zeros(1,1,T/dt+1);
tau=zeros(1,1,T/dt+1);
p=zeros(3,6,T/dt+1);
if k<=dt/T
	if theta0==theta1
		theta=zeros(1,1,T/dt+1)+theta0;
	else
		theta(1,1,:)=[theta0:(theta1-theta0)*dt/T:theta1];
	end
	if phi0==phi1
		phi=zeros(1,1,T/dt+1)+phi0;
	else
		phi(1,1,:)=[phi0:(phi1-phi0)*dt/T:phi1];
	end
	if tau0==tau1
		tau=zeros(1,1,T/dt+1)+tau0;
	else
		tau(1,1,:)=[tau0:(tau1-tau0)*dt/T:tau1];
	end
	
	p0=alpha0;
	p1=alpha1;
	dimension=1;
	if p0==p1
		prep=zeros(1,6,T/dt+1)+p0;
		p(dimension,:,:)=prep;
	else
		prep=zeros(1,1,T/dt+1);
		prep(1,1,:)=[p0:(p1-p0)*dt/T:p1];
		prep=zeros(1,6,T/dt+1)+prep;
		p(dimension,:,:)=prep;
	end
	
	p0=beta0;
	p1=beta1;
	dimension=2;
	if p0==p1
		prep=zeros(1,6,T/dt+1)+p0;
		p(dimension,:,:)=prep;
	else
		prep=zeros(1,1,T/dt+1);
		prep(1,1,:)=[p0:(p1-p0)*dt/T:p1];
		prep=zeros(1,6,T/dt+1)+prep;
		p(dimension,:,:)=prep;
	end
	
	p0=gamma0;
	p1=gamma1;
	dimension=3;
	if p0==p1
		prep=zeros(1,6,T/dt+1)+p0;
		p(dimension,:,:)=prep;
	else
		prep=zeros(1,1,T/dt+1);
		prep(1,1,:)=[p0:(p1-p0)*dt/T:p1];
		prep=zeros(1,6,T/dt+1)+prep;
		p(dimension,:,:)=prep;
	end
else
	L0=theta0;
	L1=theta1;
	y=(L1-L0).*prey+L0;
	theta(1,1,:)=y;
	
	L0=phi0;
	L1=phi1;
	y=(L1-L0).*prey+L0;
	phi(1,1,:)=y;
	
	L0=tau0;
	L1=tau1;
	y=(L1-L0).*prey+L0;
	tau(1,1,:)=y;
	
	L0=alpha0;
	L1=alpha1;
	prep=zeros(1,1,T/dt+1);
	y=(L1-L0).*prey+L0;
	prep(1,1,:)=y;
	prep=zeros(1,6,T/dt+1)+prep;
	p(1,:,:)=prep;
	
	L0=beta0;
	L1=beta1;
	prep=zeros(1,1,T/dt+1);
	y=(L1-L0).*prey+L0;
	prep(1,1,:)=y;
	prep=zeros(1,6,T/dt+1)+prep;
	p(2,:,:)=prep;
	
	L0=gamma0;
	L1=gamma1;
	prep=zeros(1,1,T/dt+1);
	y=(L1-L0).*prey+L0;
	prep(1,1,:)=y;
	prep=zeros(1,6,T/dt+1)+prep;
	p(3,:,:)=prep;
end
%% One way motion: from one position to another (linear + sine wave)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=2000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=0;
% phi0=0;
% tau0=0;
% alpha0=0;
% beta0=0;
% gamma0=230;
% 
% theta1=0;
% phi1=10;
% tau1=0;
% alpha1=0;
% beta1=0;
% gamma1=230;
% % theta and phi are in degrees
% 
% k=0.5;					% buffer size only for the linear one 
% lin_or_sin=1;
% n=3;					% buffer power only for the linear one
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);

theta1=nextSTATE(1);
phi1=nextSTATE(2);
tau1=nextSTATE(3);
alpha1=nextSTATE(4);
beta1=nextSTATE(5);
gamma1=nextSTATE(6);
lin_or_sin;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;
theta1=(pi/180)*theta1;
phi1=(pi/180)*phi1;
tau1=(pi/180)*tau1;

if k>0.5
	k=0.5;
end

theta=zeros(1,1,T/dt+1);
phi=zeros(1,1,T/dt+1);
tau=zeros(1,1,T/dt+1);
p=zeros(3,6,T/dt+1);
if k<=dt/T
	if theta0==theta1
		theta=zeros(1,1,T/dt+1)+theta0;
	else
		theta(1,1,:)=[theta0:(theta1-theta0)*dt/T:theta1];
	end
	if phi0==phi1
		phi=zeros(1,1,T/dt+1)+phi0;
	else
		phi(1,1,:)=[phi0:(phi1-phi0)*dt/T:phi1];
	end
	if tau0==tau1
		tau=zeros(1,1,T/dt+1)+tau0;
	else
		tau(1,1,:)=[tau0:(tau1-tau0)*dt/T:tau1];
	end
	
	p0=alpha0;
	p1=alpha1;
	dimension=1;
	if p0==p1
		prep=zeros(1,6,T/dt+1)+p0;
		p(dimension,:,:)=prep;
	else
		prep=zeros(1,1,T/dt+1);
		prep(1,1,:)=[p0:(p1-p0)*dt/T:p1];
		prep=zeros(1,6,T/dt+1)+prep;
		p(dimension,:,:)=prep;
	end
	
	p0=beta0;
	p1=beta1;
	dimension=2;
	if p0==p1
		prep=zeros(1,6,T/dt+1)+p0;
		p(dimension,:,:)=prep;
	else
		prep=zeros(1,1,T/dt+1);
		prep(1,1,:)=[p0:(p1-p0)*dt/T:p1];
		prep=zeros(1,6,T/dt+1)+prep;
		p(dimension,:,:)=prep;
	end
	
	p0=gamma0;
	p1=gamma1;
	dimension=3;
	if p0==p1
		prep=zeros(1,6,T/dt+1)+p0;
		p(dimension,:,:)=prep;
	else
		prep=zeros(1,1,T/dt+1);
		prep(1,1,:)=[p0:(p1-p0)*dt/T:p1];
		prep=zeros(1,6,T/dt+1)+prep;
		p(dimension,:,:)=prep;
	end
else
	x0=[0:dt/T:1];
	if lin_or_sin~=1
		a0=1/(k^(n-1)*(2*k+n*(1-2*k)));
		x0=[0:dt/T:1];
		prey1=a0*x0.^n;
		prey1(x0>=k)=0;
		prey2=a0*k^n+n*a0*k^(n-1)*(x0-k);
		prey2(x0<k | x0>=1-k)=0;
		prey3=1-a0*(1-x0).^n;
		prey3(x0<1-k)=0;
		prey=prey1+prey2+prey3;
	else
		prey=(1/2)*(1-cos(pi*x0));
	end
	L0=theta0;
	L1=theta1;
	y=(L1-L0).*prey+L0;
	theta(1,1,:)=y;
	
	L0=phi0;
	L1=phi1;
	y=(L1-L0).*prey+L0;
	phi(1,1,:)=y;
	
	L0=tau0;
	L1=tau1;
	y=(L1-L0).*prey+L0;
	tau(1,1,:)=y;
	
	L0=alpha0;
	L1=alpha1;
	prep=zeros(1,1,T/dt+1);
	y=(L1-L0).*prey+L0;
	prep(1,1,:)=y;
	prep=zeros(1,6,T/dt+1)+prep;
	p(1,:,:)=prep;
	
	L0=beta0;
	L1=beta1;
	prep=zeros(1,1,T/dt+1);
	y=(L1-L0).*prey+L0;
	prep(1,1,:)=y;
	prep=zeros(1,6,T/dt+1)+prep;
	p(2,:,:)=prep;
	
	L0=gamma0;
	L1=gamma1;
	prep=zeros(1,1,T/dt+1);
	y=(L1-L0).*prey+L0;
	prep(1,1,:)=y;
	prep=zeros(1,6,T/dt+1)+prep;
	p(3,:,:)=prep;
end
%% Looping motion: theta rotation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=5000;							% (ms) time/one rotation cycle --> must be a multiple of dt!!!!!
theta0=0;
phi0=10;
tau0=0;
alpha0=0;
beta0=0;
gamma0=230;
one_or_negativeone=1;			% Direction of rotation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % Define previousSTATE and nextSTATE --> contains variables for the position and angle
% theta0=previousSTATE(1);
% phi0=previousSTATE(2);
% tau0=previousSTATE(3);
% alpha0=previousSTATE(4);
% beta0=previousSTATE(5);
% gamma0=previousSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

theta=zeros(1,1,T/dt+1);
theta(1,1,:)=one_or_negativeone.*[theta0:2*pi*dt/T:theta0+2*pi];


phi=zeros(1,1,T/dt+1)+phi0;

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,6,T/dt+1);
prep=[];
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;
%% Looping motion: tilt rotation FRONT BUFFER

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
theta0=0;
phi0=10;
tau0=10;
alpha0=0;
beta0=0;
gamma0=230;
one_or_negativeone=1;			% Direction of rotation
k=0.3;							% Buffer size
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(pi/180)*theta0;
theta1=theta0+2*pi;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

x0=[0:dt/T:1];
prey1=(1/2).*(x0.^2)./(k*(1-k/2));
prey1(x0>=k)=0;
prey2=k.*(x0-k/2)./(k*(1-k/2));
prey2(x0<k)=0;
prey=prey1+prey2;

theta=zeros(1,1,T/dt+1);
if k<=dt/T
	if theta0==theta1
		theta=zeros(1,1,T/dt+1)+theta0;
	else
		theta(1,1,:)=[theta0:(theta1-theta0)*dt/T:theta1];
	end
else
	L0=theta0;
	L1=theta1;
	y=(L1-L0).*prey+L0;
	theta(1,1,:)=y;
end

phi=zeros(1,1,T/dt+1)+phi0;

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,6,T/dt+1);
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;
%% Looping motion: tilt rotation BACK BUFFER

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
theta0=0;
phi0=10;
tau0=10;
alpha0=0;
beta0=0;
gamma0=230;
one_or_negativeone=1;			% Direction of rotation
k=0.3;							% Buffer size
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(pi/180)*theta0;
theta1=theta0+2*pi;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

x0=[0:dt/T:1];
prey2=x0./(1-k/2);
prey2(x0>=1-k)=0;
prey3=1-(1/2).*((1-x0).^2)./(k*(1-k));
prey3(x0<1-k)=0;
prey=prey2+prey3;

theta=zeros(1,1,T/dt+1);
if k<=dt/T
	if theta0==theta1
		theta=zeros(1,1,T/dt+1)+theta0;
	else
		theta(1,1,:)=[theta0:(theta1-theta0)*dt/T:theta1];
	end
else
	L0=theta0;
	L1=theta1;
	y=(L1-L0).*prey+L0;
	theta(1,1,:)=y;
end

phi=zeros(1,1,T/dt+1)+phi0;

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,6,T/dt+1);
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;
%% Looping motion: tau rotation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=0;
% phi0=0;
% tau0=10;
% alpha0=0;
% beta0=0;
% gamma0=230;
% one_or_negativeone=1;			% Direction of rotation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;


theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1)+phi0;

tau(1,1,:)=tau0.*cos(one_or_negativeone.*[0:2*pi*dt/T:2*pi]);

p=zeros(3,6,T/dt+1);
prep=[];
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;
%% Looping motion: phi tilting motion back and forth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=90;
% phi0=10;
% tau0=0;
% alpha0=0;
% beta0=0;
% gamma0=230;
% one_or_negativeone=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1);
phi(1,1,:)=phi0.*cos(one_or_negativeone.*[0:2*pi*dt/T:2*pi]);

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,6,T/dt+1);
prep=[];
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;
%% Looping motion: XYrot horizontal shift (circular) 
% The center of the platform (the origin of the platform coordinate) follows a circular trajectory parallel to the
% xy plane of the base coordinate. Its center of rotation is the z axis of
% the base coordinate.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=0;
% phi0=0;
% tau0=0;
% alpha0=30;
% beta0=0;
% gamma0=230;
% one_or_negativeone=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1)+phi0;

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,T/dt+1,1);
p=cos(one_or_negativeone.*[0:2*pi*dt/T:2*pi]).*[alpha0;beta0;0]+sin(one_or_negativeone.*[0:2*pi*dt/T:2*pi]).*[-beta0;alpha0;0]+[0;0;gamma0];
p=permute(p,[1,3,2]);
p=p+zeros(3,6,T/dt+1);
%% Looping motion: Zvert vertical shift
% The platform moves up and down in the z direction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=0;
% phi0=0;
% tau0=0;
% alpha0=0;
% beta0=0;
% gamma0=240;
% gamma1=210;
% one_or_negativeone=1;		% no function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1)+phi0;

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,1,T/dt+1);
p(3,1,:)=(gamma0+gamma1)/2+(gamma0-gamma1)/2.*cos([0:2*pi*dt/T:2*pi]);
p=p+zeros(3,6,T/dt+1);
%% Looping motion: YZrot (circular)
% The center of the platform (the origin of the platform coordinate) follows a circular trajectory parallel to the
% xy plane of the base coordinate. Its center of rotation is the z axis of
% the base coordinate.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=5000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=0;
% phi0=0;
% tau0=0;
% alpha0=0;
% beta0=0;
% gamma0=210;
% gamma1=240;
% one_or_negativeone=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
tau0=(pi/180)*tau0;

theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1)+phi0;

tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,T/dt+1,1);
p=cos(one_or_negativeone.*[0:2*pi*dt/T:2*pi]).*[0;beta0;(gamma0-gamma1)/2]+sin(one_or_negativeone.*[0:2*pi*dt/T:2*pi]).*[0;-(gamma0-gamma1)/2;beta0]+[0;0;(gamma0+gamma1)/2];
p=permute(p,[1,3,2]);
p=p+zeros(3,6,T/dt+1);
%% Random direction motion (linear, outdated,Inaccurate)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=4000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
theta0=90;
phi0=0;
phi1=10;
tau0=0;
alpha0=0;
beta0=0;
gamma0=230;
phi1=10;
random_deg=180*rand;	% [0,180)
k=0.5;			% buffer size
n=3;			% buffer power (only for line)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Define previousSTATE and nextSTATE --> contains variables for the position and angle
% theta0=previousSTATE(1);
% phi0=previousSTATE(2);
% tau0=previousSTATE(3);
% alpha0=previousSTATE(4);
% beta0=previousSTATE(5);
% gamma0=previousSTATE(6);
% phi1;
% random_deg;	% [0,180)
% k;			% buffer size
% n;			% buffer power (only for line)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(pi/180)*theta0;
phi0=(pi/180)*phi0;
phi1=(pi/180)*phi1;
tau0=(pi/180)*tau0;

theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1);
if k>0.5
	k=0.5;
end

acc=1/(k^(n-1)*(2*k+n*(1-2*k)));
x0=[0:dt/T:1];
prey1=acc*(x0.^n);
prey1(x0>=k)=0;
prey2=acc*k^n+n*acc*k^(n-1)*(x0-k);
prey2(x0<k | x0>1-k)=0;
prey3=1-acc*(1-x0).^n;
prey3(x0<=1-k)=0;
prey=prey1+prey2+prey3;

phi=zeros(1,1,T/dt+1);
if k<=dt/T
	phi(1,1,:)=[phi0:(phi1-phi0)*dt/T:phi1];
else
	L0=phi0;
	L1=phi1;
	y=(L1-L0).*prey+L0;
	phi(1,1,:)=y;
end


tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,6,T/dt+1);
prep=[];
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;
%% Random direction motion (linear + sine wave)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T=4000;							% (ms) time/one rotation cycle --> must be multiple of dt!!!!!
% theta0=90;
% phi0=0;
% phi1=10;
% tau0=0;
% alpha0=0;
% beta0=0;
% gamma0=230;
% phi1=10;
% random=180*rand;	% [0,180)
% n=3;			% buffer power (only for lin)
% k=0.3;			% spike position
% lin_or_sin=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define previousSTATE and nextSTATE --> contains variables for the position and angle
theta0=previousSTATE(1);	% this serves no function
phi0=previousSTATE(2);
tau0=previousSTATE(3);
alpha0=previousSTATE(4);
beta0=previousSTATE(5);
gamma0=previousSTATE(6);
dt;
T;
phi1;
random;	% [0,1)
n;			% buffer power (only for lin)
k;			% spike position
lin_or_sin;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta0=(2*pi)*random;
phi0=(pi/180)*phi0;
phi1=(pi/180)*phi1;
tau0=(pi/180)*tau0;

theta=[];
phi=[];
tau=[];
p=[];

theta=zeros(1,1,T/dt+1)+theta0;

phi=zeros(1,1,T/dt+1);
if k>0.5
	k=0.5;
end

phi=zeros(1,1,T/dt+1);
if lin_or_sin~=1
	if k<=dt/T
		phi(1,1,:)=[phi0:(phi1-phi0)*dt/T:phi1];
	else
		a1=2^(n-1)/k^n;
		a2=2^(n-1)/(1-k)^n;
		x0=[0:dt/T:1];
		prey1=a1*x0.^n;
		prey1(x0>=k/2)=0;
		prey2=1-a1*(k-x0).^n;
		prey2(x0<k/2 | x0>=k)=0;
		prey3=1-a2*(x0-k).^n;
		prey3(x0<k | x0>=(1+k)/2)=0;
		prey4=a2*(1-x0).^n;
		prey4(x0<(1+k)/2)=0;
		prey=prey1+prey2+prey3+prey4;
		L0=phi0;
		L1=phi1;
		y=(L1-L0).*prey+L0;
		phi(1,1,:)=y;
	end
else
	if k<=dt/T
		phi(1,1,:)=[phi0:(phi1-phi0)*dt/T:phi1];
	else
		x0=[0:dt/T:1];
		prey1=(1/2)*(1-cos(pi/k*x0));
		prey1(x0>=k)=0;
		prey2=(1/2)*(1-cos(pi/(1-k)*(x0+1-2*k)));
		prey2(x0<k)=0;
		prey=prey1+prey2;
		L0=phi0;
		L1=phi1;
		y=(L1-L0).*prey+L0;
		phi(1,1,:)=y;
	end
end


tau=zeros(1,1,T/dt+1)+tau0;

p=zeros(3,6,T/dt+1);
prep=[];
prep=zeros(1,6,T/dt+1)+alpha0;
p(1,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+beta0;
p(2,:,:)=prep;

prep=[];
prep=zeros(1,6,T/dt+1)+gamma0;
p(3,:,:)=prep;

%% Calculating motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Receive motion_seq and convert them to theta, phi, tau, and p in labview,
% and then input them to the matlab script node
% Required inputs to the node: theta, phi, tau, p, T, dt, Achievable, l, r, ballangle,
% diskofst, q0, L, qmax, qmin, distrev, pulserev, ArdNI_ratio, A, B
theta=reshape(theta,1,1,T/dt+1);
phi=reshape(phi,1,1,T/dt+1);
tau=reshape(tau,1,1,T/dt+1);
p=reshape(p,3,6,T/dt+1);

lmat=zeros(3,6,T/dt+1)+l;

N=zeros(3,1,T/dt+1);
N(1,1,:)=cos(theta).*sin(phi);
N(2,1,:)=sin(theta).*sin(phi);
N(3,1,:)=cos(phi);
N=N+zeros(3,6,T/dt+1);

prex=zeros(3,1,T/dt+1);
prex(1,1,:)=cos(phi);
prex(3,1,:)=-cos(theta).*sin(phi);
prex=prex+zeros(3,6,T/dt+1);
normprex=sqrt(dot(prex,prex,1));
x=prex./normprex;

y=cross(N,x,1);

x=cos(tau).*x+sin(tau).*y;
y=-sin(tau).*x+cos(tau).*y;

distp=zeros(T/dt+1,6);
disk=zeros(3,6,T/dt+1);
ball=zeros(3,6,T/dt+1);
pp=zeros(3,6,T/dt+1);

disk=r.*(x.*cos(ballangle)+y.*sin(ballangle))+p;
ball=r.*(x.*cos(ballangle)+y.*sin(ballangle))+p-diskofst.*N;
C=-r.*(x.*cos(ballangle)+y.*sin(ballangle))-p+diskofst.*N+q0;
b=dot(C,lmat,1);
c=dot(C,C,1)-L*L;
det=b.*b-c;
if all(det(:)>=0)
	distp=-b+sqrt(det);
	pp=l.*distp+q0;
	distp=permute(distp,[3,2,1]);
else
	Achievable=0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Checking configuration feasibility %%%%
% Checking if the range of prismatic joint is within the range
if Achievable==1
	if all(distp(:)<=qmax) && all(distp(:)>=qmin)
		Achievable=1*Achievable;
	else
		Achievable=0;
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Providing output to labview %%%%

dx=[];												% (mm) the distance traveled for each labview/arduino loop
v=[];												% the velocity
if Achievable==1
	dx=diff(distp,1,1);
	v=dx./dt;
	pulseinterval=[];								% (usec) the precise interval at which arduino has to send pulses
	direction=[];
	volt=[];
	abs_dx=abs(dx);
	pulseinterval=(10^3)*(distrev/pulserev)*dt./abs_dx;
	pulseinterval(pulseinterval>=TT)=TT;			% Making sure that pulseinterval does not exceed TT
	if all(pulseinterval(:)>=tt)
		Achievable=1*Achievable;
	else
		Achievable=0;								% Making sure that the speed is achievable (pulseinterval is less than or equal to tt)
	end
	volt=(5/(1024*ArdNI_ratio))*((B./pulseinterval)-A);
	direction=sign(dx);
	direction=(1/2)*(direction+1);
end

Defpos=distp(1,:);								% This is where the prismatic joints start their movement. So the prismatic joints have to be manually set to these precise values
AdjDefpos=Defpos-9.206;


%%%% Position relative to the initial position %%%%
Position=distp;

% max_tt=min(pulseinterval,[],"all");

% Total_pulse_count=sum(abs_dx./(distrev/pulserev),1);
%% plotting in 3d
n=T/dt;
mrgn=1;
wdth=2;
mrkrsize=100;
base=40;
t=0;
loop=4;
figure;
while t<=loop
for k=1:n
	k
cla
for c=1:5
hold on
% plotting the prismatic joint
plot3([q0(1,c),q0(1,c)+lgth*l(1,c)],[q0(2,c),q0(2,c)+lgth*l(2,c)],[q0(3,c),q0(3,c)+lgth*l(3,c)],'color','red','LineWidth',wdth);
% plotting arms
plot3([pp(1,c,k),ball(1,c,k)],[pp(2,c,k),ball(2,c,k)],[pp(3,c,k),ball(3,c,k)],'color','green','LineWidth',wdth);
% plotting balls
scatter3([pp(1,c,k),ball(1,c,k)],[pp(2,c,k),ball(2,c,k)],[pp(3,c,k),ball(3,c,k)],mrkrsize,'green','filled');
% connecting the balls to the surface of the disk
plot3([ball(1,c,k),disk(1,c,k)],[ball(2,c,k),disk(2,c,k)],[ball(3,c,k),disk(3,c,k)],'color','magenta','LineWidth',wdth);
% plotting the surface of the disk
plot3([disk(1,c,k),disk(1,c+1,k)],[disk(2,c,k),disk(2,c+1,k)],[disk(3,c,k),disk(3,c+1,k)],'color','blue','LineWidth',wdth);
% plotting the base vectors of the disk
plot3([p(1,c),base*N(1,1,k)+p(1,c)],[p(2,c),base*N(2,1,k)+p(2,c)],[p(3,c),base*N(3,1,k)+p(3,c)],'color','green','LineWidth',wdth);
plot3([p(1,c),base*x(1,1,k)+p(1,c)],[p(2,c),base*x(2,1,k)+p(2,c)],[p(3,c),base*x(3,1,k)+p(3,c)],'color','red','LineWidth',wdth);
plot3([p(1,c),base*y(1,1,k)+p(1,c)],[p(2,c),base*y(2,1,k)+p(2,c)],[p(3,c),base*y(3,1,k)+p(3,c)],'color','blue','LineWidth',wdth);
end
c=6;
plot3([q0(1,c),q0(1,c)+lgth*l(1,c)],[q0(2,c),q0(2,c)+lgth*l(2,c)],[q0(3,c),q0(3,c)+lgth*l(3,c)],'color','red','LineWidth',wdth);
plot3([pp(1,c,k),ball(1,c,k)],[pp(2,c,k),ball(2,c,k)],[pp(3,c,k),ball(3,c,k)],'color','green','LineWidth',wdth);
scatter3([pp(1,c,k),ball(1,c,k)],[pp(2,c,k),ball(2,c,k)],[pp(3,c,k),ball(3,c,k)],mrkrsize,'green','filled');
plot3([ball(1,c,k),disk(1,c,k)],[ball(2,c,k),disk(2,c,k)],[ball(3,c,k),disk(3,c,k)],'color','magenta','LineWidth',wdth);
plot3([disk(1,c,k),disk(1,1,k)],[disk(2,c,k),disk(2,1,k)],[disk(3,c,k),disk(3,1,k)],'color','blue','LineWidth',wdth);
plot3([p(1,c),base*N(1,1,k)+p(1,c)],[p(2,c),base*N(2,1,k)+p(2,c)],[p(3,c),base*N(3,1,k)+p(3,c)],'color','cyan','LineWidth',wdth);
plot3([p(1,c),base*x(1,1,k)+p(1,c)],[p(2,c),base*x(2,1,k)+p(2,c)],[p(3,c),base*x(3,1,k)+p(3,c)],'color','red','LineWidth',wdth);
plot3([p(1,c),base*y(1,1,k)+p(1,c)],[p(2,c),base*y(2,1,k)+p(2,c)],[p(3,c),base*y(3,1,k)+p(3,c)],'color','blue','LineWidth',wdth);
axis equal
xlim([q0(1,4)-(1/2)*lgth-mrgn,q0(1,1)+lgth+mrgn]);
ylim([q0(2,6)-(sqrt(3)/2)*lgth-mrgn,q0(2,3)+(sqrt(3)/2)*lgth+mrgn]);
zlim([0,L+diskofst+r+mrgn]);
hold off
view(3)
drawnow
% pause(0.02)
end
t=t+1;
end