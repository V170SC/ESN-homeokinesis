% % Initialization of data and random number generator
clear all;
close all;

% for k =1:1000
stream = RandStream('mt19937ar','Seed',28);
RandStream.setDefaultStream(stream);

% % Size of data
train = 4000;
test = 2000;
init = 800;

% output1 = load('MackeyGlass_t17.txt');
% output2 = load('sin10.txt');
% output3 = output1 + output2(1:10000);
output4 = load('sin1.txt');
patts{1} = output4;
% patts{1} = output1;
% patts{2} = output2;
% patts{3} = output3;
% patts{4} = output4;
N = length(patts);


% % Generate reservoir
% % Define size and parameters
resSize = 100;
a = 0.8; % leaking rate
alpha = 250; % aperture of conceptor
% alpha = k;
% alpha = [261 1000 31 1000];
rho = 0.96; % spectral radius
% % Set initial weights as random from -1 to 1
Win = (rand(resSize, 1)-0.5)*2;
Wbias = (rand(resSize,1)-0.5)*2;
Wstar = (rand(resSize)-0.5)*2;
% % Normalize to spectral radius
opt.disp = 0;
rhoW = abs(eigs(Wstar,1,'LM',opt));
Wstar = Wstar*(rho/rhoW);

trainArgs = zeros(resSize,N*train);
trainOldArgs = zeros(resSize,N*train);
trainOutputs = zeros(1,N*train);
pattCollectors = cell(1,N);
SRCollectors = cell(1,N);
URCollectors = cell(1,N);
pattRs = cell(1,N);
trainX = cell(1,N);
trainP = cell(1,N);
startXs = zeros(resSize, N);

% % Sample network training dynamics
% % Run reservoir with data
for p = 1:N
	pattern = patts{p};
	x = zeros(resSize,1);
	M = zeros(resSize,train);
	oldCollector = zeros(resSize,train);
	T = zeros(1,train);
	for t = 1:train+init
		u = pattern(t);
		xOld = x;
		x = (1-a)*x + a*tanh(Win*u + Wstar*x + Wbias);
		if t > init
			M(:,t-init) = x;
			oldCollector(:,t-init) = xOld;
			T(t-init) = pattern(t+1);
		end
	end
	
	R = M*M'/train;
    pattRs{p} = R;
	startXs(:,p) = x;
	
	pattCollectors{1,p} = T;
	trainArgs(:,(p-1)*train+1:p*train)=M;
	trainOldArgs(:,(p-1)*train+1:p*train)=oldCollector;
	trainOutputs(1,(p-1)*train+1:p*train)=T;
	
end

% % Train the output

reg = 1e-4; % regularization coefficient
Wout = (inv(trainArgs*trainArgs'+reg*eye(resSize))*trainArgs*trainOutputs')';

% % Calculate the mean squared error
target = trainOutputs;
output = Wout*trainArgs;
totVar = (var(target,0,2)+var(output,0,2))/2;
nrmse_readout = sqrt(mean((target-output).^2)/totVar);

% % compute W
Wtargets = atanh(trainArgs)- repmat(Wbias,1,N*train);
W = (inv(trainOldArgs*trainOldArgs'+reg*eye(resSize))*trainOldArgs*Wtargets')';

% % Calculate the mean squared error
target = Wtargets;
output = W*trainOldArgs;
totVar = (var(target,0,2)+var(output,0,2))/2;
nrmse_W = sqrt(mean((target-output).^2,2)./totVar);
mean_nrmse_W = mean(nrmse_W);

% % loaded reservoir with messy output
% figure(10); clf;
% for p = 1:1
    % x = startXs(:,p);
    % messyOutPL = zeros(1,test);
    % for n = 1:test
        % x = tanh(W*x);
        % y = Wout * x;
        % messyOutPL(1,n) = y;
    % end
    % subplot(2,2,p);
    % plot(messyOutPL);
% end

% % compute projectors
Cs = cell(1,N);
for p = 1:N
	R = pattRs{p};
	[U S V] = svd(R);
	Snew = (S * inv(S + alpha(p)^(-2)*eye(resSize)));
	C = U*Snew*U';
	Cs{1,p} = C;
end

% % test with C
p_CTestPL = zeros(1, test, N);
for p = 1:N
    C = Cs{1, p};
    x = startXs(:,p);
    % x = 0.5*rand(resSize,1);
	% x = zeros(resSize,1);
    
    for n = 1:test
        x = tanh(W*x + Wbias);
        x = C*x;
        p_CTestPL(:,n,p) = Wout*x;		
    end
end

target = output4(train+init+2:train+init+test+1)';
output = p_CTestPL(:,:,1);
totVar = (var(target,0,2)+var(output,0,2))/2;
nrmse_MG = sqrt(mean((target-output).^2)/totVar)

figure(1);
plot( target, 'g' );
hold on;
plot( output, 'b' );
hold off;
axis tight;
title('Target and generated signals y(n) starting at n=0');
legend('Target signal', 'Free-running predicted signal');

% target = output1(train+init+2:train+init+test+1)';
% output = p_CTestPL(:,:,1);
% totVar = (var(target,0,2)+var(output,0,2))/2;
% nrmse_MG = sqrt(mean((target-output).^2)/totVar)

% figure(1);
% plot( target, 'g' );
% hold on;
% plot( output, 'b' );
% hold off;
% axis tight;
% title('Target and generated signals y(n) starting at n=0');
% legend('Target signal', 'Free-running predicted signal');

% target = output2(train+init+2:train+init+test+1)';
% output = p_CTestPL(:,:,2);
% totVar = (var(target,0,2)+var(output,0,2))/2;
% nrmse_HFsin = sqrt(mean((target-output).^2)/totVar)

% figure(2);
% plot( target, 'g' );
% hold on;
% plot( output, 'b' );
% hold off;
% axis tight;
% title('Target and generated signals y(n) starting at n=0');
% legend('Target signal', 'Free-running predicted signal');

% target = output3(train+init+2:train+init+test+1)';
% output = p_CTestPL(:,:,3);
% totVar = (var(target,0,2)+var(output,0,2))/2;
% nrmse_sum = sqrt(mean((target-output).^2)/totVar)

% figure(3);
% plot( target, 'g' );
% hold on;
% plot( output, 'b' );
% hold off;
% axis tight;
% title('Target and generated signals y(n) starting at n=0');
% legend('Target signal', 'Free-running predicted signal');

% target = output4(train+init+2:train+init+test+1)';
% output = p_CTestPL(:,:,4);
% totVar = (var(target,0,2)+var(output,0,2))/2;
% nrmse_LFsin = sqrt(mean((target-output).^2)/totVar)

% figure(4);
% plot( target, 'g' );
% hold on;
% plot( output, 'b' );
% hold off;
% axis tight;
% title('Target and generated signals y(n) starting at n=0');
% legend('Target signal', 'Free-running predicted signal');


% if k==1
	% error = nrmse_generate;
% else
	% error = [error,nrmse_generate];
% end

% k

% end

% [min,index] = min(error)

% figure(2);
% plot( error );
% axis tight;
% title('NRMSE for aperture adaptation for sum');
% legend('NRMSE');

