dh = 1/4*[-1/5 1/5 1/5 -1/5; 1 1 1 1; -1 1 -1 1];
dthta = [-1.18 0.68 0.02 -0.52]';
Vb = dh*dthta
dQb = [Vb(1);(Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1);(Vb(3)*sin(Vb(1))+Vb(2)*(1-cos(Vb(1))))/Vb(1)];
q = dQb

w = 7.07213/2;

syms thta

T = [cos(thta) -sin(thta) -3*sin(thta);sin(thta) cos(thta) 2+3*cos(thta);0 0 1]

gradient(T,thta)

a =sqrt(13/2)*0.5