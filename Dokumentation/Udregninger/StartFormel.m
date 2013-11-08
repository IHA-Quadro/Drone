
B = 1200;
A = 1450-B;
t = -2;
tau = 0.01:0.1:5;
funk = A*(1 - exp(tau *t) ) + B;

plot(tau, funk)
axis on
grid on
axis([0, 2.5, 1200, 1550])

format shortg
syms x
y = solve(A*(1- exp(x * t))+B == 1495, x)
y2 = A*(1- exp(2 * t))+B