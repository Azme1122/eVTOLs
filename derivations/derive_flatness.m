function result = derive_flatness
% Exercise 8.2(a,b,d): plant derivatives, state and input parameterization.
syms z1 dz1 d2z1 d3z1 d4z1 z2 dz2 d2z2 d3z2 d4z2 real
syms g positive
coordinates=[z1 dz1 d2z1 d3z1 z2 dz2 d2z2 d3z2];
rates=[dz1;d2z1;d3z1;d4z1;dz2;d2z2;d3z2;d4z2];
totalDerivative=@(expression) simplify(jacobian(expression,coordinates)*rates);
den=d2z1^2+(d2z2+g)^2;
u1=sqrt(den);
x5=atan2(-d2z1,d2z2+g);
x6=totalDerivative(x5);
u2=totalDerivative(x6);
du1=totalDerivative(u1);
expectedOmega=(d2z1*d3z2-(d2z2+g)*d3z1)/den;
expectedU2=2*(d2z1*d3z1+(d2z2+g)*d3z2) ...
    *(d3z1*(d2z2+g)-d2z1*d3z2)/den^2 ...
    -(d4z1*(d2z2+g)-d2z1*d4z2)/den;
assert(isAlways(simplify(x6-expectedOmega)==0));
assert(isAlways(simplify(u2-expectedU2)==0));
result=struct('x',[z1;dz1;z2;dz2;x5;x6],'u1',u1,'u2',u2, ...
    'du1',du1,'den',den,'coordinates',coordinates,'rates',rates, ...
    'reference',[z1;dz1;d2z1;d3z1;d4z1;z2;dz2;d2z2;d3z2;d4z2],'g',g);
% Input derivatives below are bookkeeping variables, not plant states.
syms x1 x2 x3 x4 x5 x6 u1 du1 d2u1 u2 real
plantCoordinates=[x1 x2 x3 x4 x5 x6 u1 du1];
plantRates=[x2;-u1*sin(x5);x4;u1*cos(x5)-g;x6;u2;du1;d2u1];
Z1=sym(zeros(5,1));Z2=sym(zeros(5,1));Z1(1)=x1;Z2(1)=x3;
for order=2:5
    Z1(order)=simplify(jacobian(Z1(order-1),plantCoordinates)*plantRates);
    Z2(order)=simplify(jacobian(Z2(order-1),plantCoordinates)*plantRates);
end
result.plantOutputs=struct('Z1',Z1,'Z2',Z2,'plant',plantRates(1:6),'g',g);
disp('Flat outputs and derivatives:');disp(Z1);disp(Z2);
disp('State parameterization psi_x:');disp(result.x);
disp('u1 =');disp(result.u1);disp('u2 =');disp(result.u2);
disp('du1 =');disp(result.du1);
end
