M1 = 290;
M2 = 59;
K1 = 16182;
K2 = 190000;
C = 1000;    

    
    A = [0 0 1 0;
        0 0 0 1;
        -K1/M1 K1/M1 -C/M1 C/M1;
        K1/M2 -(K1+K2)/M2 C/M2 -C/M2];
    B = [0; 0; -1/M1; 1/M2];
    
    Q = 10^4 *diag([1 1 1 1]);
    R = 0.001;
    [K,S,P] = dlqr(A,B,Q,R);
    gain = K