%% LMI for Non-rotating Missle Attitude Control (Pitch and Yaw/Roll Channels)
% We used the matrices A, B1, B2, C, D1, D2 for Yaw/Roll channel in this simulation

clc; clear; close all;

A = [-0.5 1 0 0 0
     -62 -0.16 0 0 30
     10 0 -50  40 5
     0  0  0  -40 0
     0 0   0   0 -20];
 B1 = [0 0 
     0 0
     0 0
     40 0
     0 20];
 B2 = [0.001 -0.05 0 0 0]';
 C = [1.5 0 0 0 0.8
      0 0 1 0 0 ];
  D1 = 0; D2 = [0.01 0]';
  
  
X = sdpvar(5,5);
W = sdpvar(2,5);
gamma = sdpvar(1);
Const = [];
eta = 0.00000001;
Const = [Const, X >= eta*eye(size(X))];
M = [(A*X+B1*W)'+(A*X+B1*W)   B2   (C*X+D1*W)'
        B2'      -gamma*eye(1)    D2'
        C*X+D1*W    D2      -gamma*eye(2)];
Const = [Const, M <= -eta*eye(size(M))];
optimize(Const, gamma);

W = value(W)
X = value(X)
K = W*pinv(X)
gamma = value(gamma)
