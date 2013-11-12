clear; close all;

B = 1200;
A = 1500;
t1 = 1.2;
t2 = 0.6;
tau = 0.01:0.025:5;
syms x;
funk1 = ((A-B)*(1 - exp(-tau./t1)))+B;
funk2 = ((A-B)*(1 - exp(-tau./t2)))+B;

figure(1)
plot(tau, funk1, 'r')
hold on;
plot(tau, funk2)
axis on
grid on
axis([0, 5, 1200, 1500])

% format short
% y = solve(funkSolve == A-3, x);
% y2 = ((A-B)*(1 - exp(-1.5./t)))+B