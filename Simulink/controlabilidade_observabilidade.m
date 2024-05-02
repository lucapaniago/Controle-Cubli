%% Matrizes do Sistema
A = [0., 0., 0., 0.5, 0., 0.;
 0., 0., 0., 0., 0.5, 0.;
 0., 0., 0., 0., 0., 0.5;
 78.1896, 0., 0., -0.00129272, 0., 0.;
 0., 78.1896, 0., 0., -0.00129272, 0.;
 -264.624, -264.624, 0., 0., 0., -0.00437508];
B = [0.	0.	0.;
0.	0.	0.;
0.	0.	0.;
75.9085	0.	0.;
0.	75.9085	0.;
0.	0.	256.904];
C = [1	0	0	0	0	0;
    0	1	0	0	0	0;
    0	0	1	0	0	0];
D = zeros(3,3);
E = B;

%% Matriz de Controlabilidade
Q = ctrb(A,B);
posto_Q = rank(Q);

if posto_Q == size(A,1)
    disp(['O posto da matriz de controlabilidade é ', num2str(posto_Q), ...
        ' e A é uma matriz ', num2str(size(A,1)), 'x', num2str(size(A,1)), ...
        ', logo, o sistema é controlável']);
else
    disp(['O posto da matriz de controlabilidade é ', num2str(posto_Q), ...
        ' e A é uma matriz ', num2str(size(A,1)), 'x', num2str(size(A,1)), ...
        ', logo, o sistema não é controlável']);
end

%% Matriz de Observabilidade
N = obsv(A,C);
posto_N = rank(N);

if posto_N == size(A,1)
    disp(['O posto da matriz de observabilidade é ', num2str(posto_N), ...
        ' e A é uma matriz ', num2str(size(A,1)), 'x', num2str(size(A,1)), ...
        ', logo, o sistema é observável']);
else
    disp(['O posto da matriz de observabilidade é ', num2str(posto_N), ...
        ' e A é uma matriz ', num2str(size(A,1)), 'x', num2str(size(A,1)), ...
        ', logo, o sistema não é observável']);
end