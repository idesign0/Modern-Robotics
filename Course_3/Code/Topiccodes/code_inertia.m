radius_spr = 0.1;
ro = 5600;
volume_spr = (4/3)*pi*radius_spr^3;
mass_spr = ro*volume_spr

I_spr = (2/5)*mass_spr*radius_spr^2;

I_zz_spr = 2*(I_spr)
I_yy_spr = 2*(I_spr + mass_spr*(0.2)^2);
I_xx_spr = I_yy_spr;

length_cyl = 0.2;
radius_cyl = 0.02;
volume_cyl = pi*radius_cyl^2*length_cyl;
mass_cyl = ro*volume_cyl;

I_zz_cyl = (1/2)*mass_cyl*radius_cyl^2;
I_yy_cyl = (1/12) * mass_cyl * (3*radius_cyl^2 + length_cyl^2);
I_xx_cyl = I_yy_cyl;


I_xx = I_xx_cyl + I_xx_spr
I_yy = I_yy_cyl + I_yy_spr
i_zz = I_zz_cyl + I_zz_spr

