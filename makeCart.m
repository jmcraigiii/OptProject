function cart = makeCart(modelParams)
% Inputs = Basic Cart-Pole Model Parameters
% Output = Matlab Structure with Necessary Spatial_V2 setup
% As Per: http://royfeatherstone.org/spatial/v2/sysmodel.html
massCart = modelParams(1);
massPole= modelParams(2);
lengthPole= modelParams(3);
gravity= modelParams(4);

cart.NB = 1; 
cart.parent = [0];
cart.jtype{1} = 'Px';

cart.Xtree{1} = eye(1); % eye(1) is the identity;
cart.I{1} = mcI( massCart, [0 0 0], 0 );
cart.gravity = [0; -gravity; 0];
end