addpath('C:\Users\John Admin\Documents\MATLAB\Add-Ons\casadi-windows-matlabR2016a-v3.4.5')
import casadi.*

opti = casadi.Opti();

x = opti.variable();
y = opti.variable();

opti.minimize(  (y-4*x^2)^2   );
opti.subject_to( .4*x^2+2*y^2==1 );
opti.subject_to(     x+y>=1 );

opti.solver('ipopt');

sol = opti.solve();

sol.value(x)
sol.value(y)