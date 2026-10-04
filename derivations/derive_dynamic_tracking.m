function result = derive_dynamic_tracking(flat)
% Exercise 8.2(e,f,g): thrust extension and fourth-order tracking inversion.
if nargin==0,flat=derive_flatness;end
% xb1=u1, xb2=du1; extended inputs are [ub1 u2], with ub1=d2u1.
syms x1 x2 x3 x4 x5 x6 xb2 ub1 u2 real
syms xb1 g positive
state=[x1 x2 x3 x4 x5 x6 xb1 xb2];
rates=[x2;-xb1*sin(x5);x4;xb1*cos(x5)-g;x6;u2;xb2;ub1];
Z1=sym(zeros(5,1));Z2=sym(zeros(5,1));Z1(1)=x1;Z2(1)=x3;
for order=2:5
    Z1(order)=simplify(jacobian(Z1(order-1),state)*rates);
    Z2(order)=simplify(jacobian(Z2(order-1),state)*rates);
end
snap=[Z1(5);Z2(5)];
matrix=simplify(jacobian(snap,[ub1 u2]));
drift=simplify(subs(snap,[ub1 u2],[0 0]));
syms v1 v2 real
control=simplify(matrix\([v1;v2]-drift));
assert(isAlways(simplify(det(matrix)-xb1)==0));
assert(all(isAlways(simplify(subs(snap,[ub1 u2],control.')-[v1;v2])==0)));
extended=struct('state',state,'rates',rates,'Z1',Z1,'Z2',Z2, ...
    'matrix',matrix,'determinant',simplify(det(matrix)), ...
    'drift',drift,'control',control,'v',[v1;v2],'g',g);
% Independently derive d2u1 from the flat parameterization.
thrustAcceleration=simplify(jacobian(flat.du1,flat.coordinates)*flat.rates);
a=flat.reference(3);b=flat.reference(8)+g;
j1=flat.reference(4);j2=flat.reference(9);
s1=flat.reference(5);s2=flat.reference(10);
expected=(j1^2+j2^2+a*s1+b*s2)/sqrt(a^2+b^2) ...
    -(a*j1+b*j2)^2/(a^2+b^2)^(sym(3)/2);
assert(isAlways(simplify(thrustAcceleration-expected)==0));
tracking=simplify(subs([thrustAcceleration;flat.u2],[s1 s2],[v1 v2]));
result=struct('xb1',flat.u1,'xb2',flat.du1,'ub1',thrustAcceleration, ...
    'u2',flat.u2,'tracking',tracking,'v',[v1;v2], ...
    'reference',flat.reference,'g',g,'extended',extended);
disp('Controller states [u1; du1]:');disp([flat.u1;flat.du1]);
disp('Decoupling matrix:');disp(matrix);disp('Determinant:');disp(extended.determinant);
disp('State-based [d2u1; u2] inversion:');disp(control);
disp('Flat-output inversion with snap replaced by [v1; v2]:');disp(tracking);
end
