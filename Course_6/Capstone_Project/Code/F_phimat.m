function Hu_vb = F_phimat(chssyangle)  
r = 0.0475; 
l=0.47/2;
w=0.15;
F = r/4*[-1/l+w 1/l+w 1/l+w -1/l+w; 1 1 1 1;-1 1 -1 1];

phiMat = [1 0 0; 0 cos(chssyangle) sin(chssyangle); 0 -sin(chssyangle) cos(chssyangle)]; % global to body twist
Hu_vb = pinv(phiMat)*F; % Mat for Du->Global Vbase
end