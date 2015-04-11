clc;
clear all;
close all;


s = sym('s');
v = sym('v');
w = sym('w');


F = [0 w 0; -w 0 v; 0 0 0];
G = [1 0; 0 0; 0 1];

C = [1 0 0 0 -w^2 v*w; ...
    0 0 -w v 0 0; ...
    0 1 0 0 0 0 ];

display('Rank of C');
rank(C)

% C1 = C(:,1:3);
% C2 = C(:,2:4);
% C3 = C(:,3:5);
% C4 = C(:,4:6);
% C5 = [C(:,1) C(:,5) C(:,6)];
% C6 = [C(:,1) C(:,2) C(:,6)];
% 
% det(C1)+det(C2)+det(C3)+det(C4)+det(C5)+det(C6)