% Initialization of data and random number generator
clear all;
close all;

% for k=1:100
stream = RandStream('mt19937ar','Seed',28);
% stream = RandStream('mt19937ar','Seed',k);
RandStream.setDefaultStream(stream);

% Size of data
train = 4000;
% test = 500+k*100;
test = 2000;
init = 800;
% init = k*10;

% in = 0; % set to 1 when using input weights 0 otherwise
% back = 1; % set to 1 for teacher forcing 0 otherwise

% if in == 1
	% input = load('input.txt');
% end
output = load('MackeyGlass_t17.txt');
% output = load('MGeq.txt');
% output1 = load('MackeyGlass_t17.txt');
% output = load('sin10.txt');
% output = output1 + output2(1:10000);

% Plot data
% if in == 1
	% figure(1);
	% plot(input(1:1000));
	% title('A sample of the input data');
% end

% figure(2);
% plot(output(1:1000));
% title('A sample of the driving data');

% Generate reservoir
% Define size and parameters
inSize = 1;
outSize = 1;
resSize = 100;
% resSize = k*10;
a = 0.8; % leaking rate
% a = k*0.01;
rho = 0.96; % spectral radius
% rho = k*0.01;
% Set initial weights as random from -1 to 1
Win = (rand(resSize, 1)-0.5)*2;
Wbias = (rand(resSize,1)-0.5)*2;
W = (rand(resSize)-0.5)*2;
% W = (sprand(resSize,resSize,10/resSize)-0.5)*2;
Wback = (rand(resSize,1)-0.5)*2;
% Normalize to spectral radius
opt.disp = 0;
% opt.tol = 1e-5;
rhoW = abs(eigs(W,1,'LM',opt));
W = W*(rho/rhoW);

% Sample network training dynamics
% Allocate memory for design matrices
M = zeros(resSize,train);
T = output(init+2:train+init+1)';
% Run reservoir with data
x = zeros(resSize,1);
for t = 1:train+init
	if t > 1
		u = output(t);
	else 
		u = 0;
	end
	% if in == 1
		% u = input(t);
	% else
		% u = 1;
	% end
	% u = output(t);
	% x = (1-a)*x + a*tanh(Win*[1;u]*in + W*x + Wback*[1;d] + wgn(resSize,1,0)/10000);
	% x = (1-a)*x + a*tanh(Win*[1;u]*in + W*x + Wback*[1;d]);
	x = (1-a)*x + a*tanh(W*x + Win*u + Wbias);
	if t > init
		M(:,t-init) = x;
	end
end

% Train the output
reg = 1e-4; % regularization coefficient
Wout = T*M'*inv(M*M'+reg*eye(resSize));

Y = zeros(1,test);
u = output(train+init+1);
y = 0;
for t = 1:test
	% x = (1-a)*x + a*tanh( Wback*[1;u] + W*x + wgn(resSize,1,0)/10000);
	x = (1-a)*x + a*tanh( Win*u + W*x + Wbias);
	y = Wout*x;
	Y(:,t) = y;
	u = y;
	% u = output(train+init+t+1);
end

% Calculate the mean squared error
target = output(train+init+2:train+init+test+1)';
totVar = (var(target,0,2)+var(Y,0,2))/2;
nrmse = sqrt(mean((target-Y).^2)/totVar)

% if k==1
	% error = nrmse;
% else
	% error = [error,nrmse];
% end
% k
% end

% [min,index] = min(error)

% figure(2);
% plot( error );
% axis tight;
% title('NRMSE for spectral radius of ESN');
% legend('NRMSE');

figure(3);
plot( output(train+init+2:train+init+test+1), 'g' );
hold on;
plot( Y', 'b' );
hold off;
axis tight;
title('Target and generated signals y(n) starting at n=0');
legend('Target signal', 'Free-running predicted signal');

figure(4);
bar( Wout' )
title('Output weights W^{out}');

resState = round((rand*10)^2);
if resState > 90
	resState = 90;
end
figure(5);
plot( M(resState:resState+10,1:500)' );
title('Some reservoir activations x(n)');