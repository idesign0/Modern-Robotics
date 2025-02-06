x1 = 0.235; y1 = 0.15; thta1 = -pi/4;
x2 = -0.235; y2 = 0.15; thta2 = pi/4;
x3 = -0.235; y3 = -0.15; thta3 = 3*pi/4;
x4 = 0.235; y4 = -0.15; thta4 = 5*pi/4;

h1 = 1/0.0475*[x1*sin(thta1)-y1*cos(thta1) cos(thta1) sin(thta1)]
h2 = 1/0.0475*[x2*sin(thta2)-y2*cos(thta2) cos(thta2) sin(thta2)]
h3 = 1/0.0475*[x3*sin(thta3)-y3*cos(thta3) cos(thta3) sin(thta3)]
h4 = 1/0.0475*[x4*sin(thta4)-y4*cos(thta4) cos(thta4) sin(thta4)]

h = [h1;h2;h3;h4]

piH = pinv(h)