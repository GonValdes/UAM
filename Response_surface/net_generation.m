function net = net_generation(type,n_nodes,algorithm,t_limit)
%Generate the neural network

if strcmp(type,'feedforward')
    net = feedforwardnet(n_nodes);
elseif strcmp(type,'cascadeforward')
    net = cascadeforwardnet(n_nodes);
end

net.trainFcn = algorithm;
net.trainParam.epochs = 10000000;% Number of iterations/generations
net.trainParam.time = t_limit; %time limit(s)
net.trainParam.goal = 0;%Performance goal
net.trainParam.max_fail = 100;%Maximum validation failures/ number of iterations with the NN not improving performance

end