function y = AVL_regression_coupled(b,pars_total)
% Function to represent the regression model for the 'complex' model
% representation. Canard and wing coupled up to 2nd order terms

%Inputs:
%  b: Model parameters (theta). Order: linear terms, second order terms,
%       coupled terms and bias.
%  pars_total : Regressors (X)

%Output
%  y: vector with the estimation for each tested point with the current model parameters theta. (y = X*theta)

% Wing 1-root chord(m) 2-tip chord(m) 3-le sweep(deg) 4-root aoa(deg)
% 5-root max.camber(maxc/c*100) % 6-tip max.camber(maxc/c*100)
% Canard 7-root chord(m) 8-semispan(m)  9-root aoa(deg) 10-zle(m) 
% CG position-11(m) 

%Correct dimension for b
b_ordered=[];
b_ordered(1,:) = b;

for i=1:size(pars_total,1) % Evaluate each tested points
    pars=pars_total(i,:);
    
    semispan = 4.1;%(m)
    coupled_pars = 1:9; % Coupling between wing and canard terms
    CG_pos = 10; % CG position variable
    
    first_order_terms = 1:length(pars);
    second_order_terms = 1:length(pars);
    coupled_terms = [nchoosek([coupled_pars,CG_pos],2)];
    
    
    y(i) = b_ordered .* [pars(first_order_terms) , pars(second_order_terms).^2 , ...
        pars(coupled_terms(:,1)) .* pars(coupled_terms(:,2)) , 1] ...
        ./((pars(2)+pars(1))*semispan) ;%Divide by wing surface
end
y=y';
end