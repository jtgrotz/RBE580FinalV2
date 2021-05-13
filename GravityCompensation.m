clc; clear; close all;

% Joing variables
syms q_ qd_ qdd_ m_ [1 7]
q = q_; qd = qd_; qdd = qdd_;

% Offsets
syms d_ [1 3]
d = d_;
syms a d02 d24 d46 d67

g = 9.81;

% DH table
%                         d       theta           a alpha
dh_table = [ d02 q_1 0 -pi/2;
               0 q_2 0  pi/2;
             d24 q_3 0  pi/2;
               0 q_4 0 -pi/2;
             d46 q_5 0 -pi/2;
               0 q_6 0  pi/2;
             d67 q_7 0     0]

joints = size(dh_table);
joints = joints(1);

% Poses to plot mm  deg
joint_vals = [0 0 0 0 0 0 0];
           
poses = size(joint_vals);
poses = poses(1);

% Generate forward kinematics and individual transformations
% Create 4x4 identity matrix
FK = eye(4);

% Create an empty 4x4xn matrix to hold the tranformation matrices 
% for the base to each joint
FKs = zeros(4,4,joints,'Like',q_1);

% Create an empty 4x4xn matrix to hold the tranformation matrices 
% for the each joint n to the next joint n+1
Ts = zeros(4,4,joints,'Like',q_1);

% Create a for loop to populate the above matrices
for ii=1:joints
    % Call function dh defined at the end of this script
    T = dh(dh_table(ii,1), dh_table(ii,2), dh_table(ii,3), dh_table(ii,4));
    % Store (current to next)
    Ts(:,:,ii) = T;
    
    % Calculate base to current tranformation matrix
    FK = FK*T;
    % Store (base to current) 
    FKs(:,:,ii) = FK;
end

% Simplify the expressions to make them easier to understand and print

% Part 1 Problem 3
Ts = simplify(Ts); % All tranformation matrices
% Print each individual transformaltion matrix

FKs = simplify(FKs); % Base to each Joint
FK = simplify(FK) % Base to tip


%% Problem 2 Jacobian

j1 = diff(FK(1:3,4),q_1);
j2 = diff(FK(1:3,4),q_2);
j3 = diff(FK(1:3,4),q_3);
j4 = diff(FK(1:3,4),q_4);
j5 = diff(FK(1:3,4),q_5);
j6 = diff(FK(1:3,4),q_6);
j7 = diff(FK(1:3,4),q_7);
% Create Jv (top 3 rows of the jacobian)
Jv = simplify([j1, j2, j3, j4, j5, j6, j7]);
% Create Jw (bottom 3 rows of the jacobian)
Jw = [ Ts(1:3,3,1) Ts(1:3,3,2) Ts(1:3,3,3) Ts(1:3,3,4) Ts(1:3,3,5) Ts(1:3,3,6) Ts(1:3,3,7) ];
% Create the jacobian matrix
J = [Jv; Jw]


% Find potential engery of each joint
P1 = (m_1 * g * FKs(3,4,1));
P2 = (m_2 * g * FKs(3,4,2));
P3 = (m_3 * g * FKs(3,4,3));
P4 = (m_4 * g * FKs(3,4,4));
P5 = (m_5 * g * FKs(3,4,5));
P6 = (m_6 * g * FKs(3,4,6));
P7 = (m_7 * g * FKs(3,4,7));
P = simplify(subs(P1 + P2 + P3 + P4 + P5 + P6 + P7, [q_3, q_5], [0, 0]));

G = [ diff(P,q_1); diff(P,q_2); diff(P,q_3); diff(P,q_4); diff(P,q_5); diff(P,q_6); diff(P,q_7) ];
G = collect(expand(simplify(G)),[m_])

G = subs(G, [m_1 m_2 m_3 m_4 m_5 m_6 m_7 d67 d46 d24 d02], [.5 .5 .5 .5 .5 .5 .5 0.257 0.404 0.403 0.103]);
double(subs(G, q, [0 pi/2 0 0 0 0 0]))
double(subs(G, q, [0 0 0 0 0 0 0]))

%% Function to generate the tranformation matrices
% Input dh parameters d, theta, a, alpha
function [T] = dh(d, theta, a, alpha)
T = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
      sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
               0             sin(alpha)             cos(alpha)            d;
               0                      0                      0            1];
end