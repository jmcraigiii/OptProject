function cartPole = makeModel(modelParams)
% Inputs = Basic Cart-Pole Model Parameters
% Output = Matlab Structure with Necessary Spatial_V2 setup
% As Per: http://royfeatherstone.org/spatial/v2/sysmodel.html
massCart = modelParams(1);
massPole= modelParams(2);
lengthPole= modelParams(3);
gravity= modelParams(4);

cartPole.NB = 2; 
%the number of bodies in the tree, excluding the fixed base.  NB is also the number of joints.

cartPole.parent = [0 1];
%A vector which notes what body each link is connected to
%In this case body 1 is connected to link 0 (the fixed base)
%and body 2 is connected to link 1.            ("body" and "link" are used
%interchangeably

cartPole.jtype{1} = 'Px';
cartPole.jtype{2} = 'Rz';
%jtype describes the type of each joint.  Either [P]rismatic (sliding) or
%[R]otational, and the direction the motion is in.  This direction is based
%on the coordinate frame of the previous joint.
%Thus the first joint slides in the x direction of the fixed CS and the
%second joint rotates in the Z direction of J{1}

cartPole.Xtree{1} = eye(1); % eye(1) is the identity;
cartPole.Xtree{2} = eye(1); 
%The Xtree cell array contains part of the X-transform for each joint.  The
%actual joint motion of each joint is handled intrinsically by the jcalc()
%function in Spatial V2 and the jtype.
%Thus, the Xtree is essentially the coordinate transform to joint location 
%from the CS of the previous joint. here, all of the joints are intially 
% on top of one another

cartPole.I{1} = mcI( massCart, [0 0 0], 0 );
cartPole.I{2} = mcI( massPole, [0 -lengthPole/2 0], 0 );
%the I cell array is the inertial matrices for each of the joints in the
%model expressed in their own CS.  
%Here, the program makes use of the mcI function in Spatial V2
% I = mcI(massJoint, CoM, RotationalInertiaAboutCoM)
% In this case, the model places a point mass 1/2 a links length away from
% the center of the cart (DOWNWARDS). The Cart is a point mass also located at the
% center of the cart.

cartPole.gravity = [0; -gravity; 0];
%Sets the model gravity in the -y direction (FOR CS{0})




end