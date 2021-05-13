clear all; clc; close all;

syms q1 q2 q3 q4 q5 q6 q7 d02 d24 d46 d67
T01 = dhparamtomatrix(q1,d02,0,-90);
T12 = dhparamtomatrix(q2,0,0,90);
T23 = dhparamtomatrix(q3,d24,0,90);
T34 = dhparamtomatrix(q4,0,0,-90);
T45 = dhparamtomatrix(q5,d46,0,-90);
T56 = dhparamtomatrix(q6,0,0,90);
T67 = dhparamtomatrix(q7,d67,0,0);
%Transformation from base to tip
T07 = T01*T12*T23*T34*T45*T56*T67
%Isolate the position vector of the Robot transformation matrix
pe = T07(1:3,4)
%Derive pe to get the top of Jacobian
J = jacobian(pe, [q1,q2,q3,q4,q5,q6,q7]);
J= subs(J,[q3,q5,q6,q7],[0,0,0,0])

T01_= subs(T01,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);
T02 = T01*T12;
T02_= subs(T02,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);
T03 = T02*T23;
T03_=subs(T03,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);
T04 = T03*T34;
T04_=subs(T04,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);
T05 = T04*T45;
T05_=subs(T05,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);
T06 = T05*T56;
T06_=subs(T06,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);
T07 = T06*T67;
T07_=subs(T07,[q1,q2,q3,q4,q5,q6,q7],[0,0,0,0,0,0,0]);

J(4:6,1)= T01_(1:3,3);
J(4:6,2)= T02_(1:3,3);
J(4:6,3)= T03_(1:3,3);
J(4:6,4)= T04_(1:3,3);
J(4:6,5)= T05_(1:3,3);
J(4:6,6)= T06_(1:3,3);
J(4:6,7)= T07_(1:3,3);

J
simplify(J)


%% Tests Jacobian Function Output
% Jt(1,1) = jacobian(pe(1,1),q1);
% Jt(2,1) = jacobian(pe(2,1),q1);
% Jt(3,1) = jacobian(pe(3,1),q1);
% Jt(1,2) = jacobian(pe(1,1),q2);
% Jt(2,2) = jacobian(pe(2,1),q2);
% Jt(3,2) = jacobian(pe(3,1),q2);
% Jt(1,3) = jacobian(pe(1,1),q3);
% Jt(2,3) = jacobian(pe(2,1),q3);
% Jt(3,3) = jacobian(pe(3,1),q3);
% Jt(1,4) = jacobian(pe(1,1),q4);
% Jt(2,4) = jacobian(pe(2,1),q4);
% Jt(3,4) = jacobian(pe(3,1),q4);
% Jt(1,5) = jacobian(pe(1,1),q5);
% Jt(2,5) = jacobian(pe(2,1),q5);
% Jt(3,5) = jacobian(pe(3,1),q5);
% Jt(1,6) = jacobian(pe(1,1),q6);
% Jt(2,6) = jacobian(pe(2,1),q6);
% Jt(3,6) = jacobian(pe(3,1),q6);
% Jt(1,7) = jacobian(pe(1,1),q7);
% Jt(2,7) = jacobian(pe(2,1),q7);
% Jt(3,7) = jacobian(pe(3,1),q7);