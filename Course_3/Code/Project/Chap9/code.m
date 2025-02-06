s = QuinticTimeScaling(5,3);

Xstart = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
Xend = [0,0,1,1;1,0,0,2;0,1,0,3;0,0,0,1];

trjctory = ScrewTrajectory(Xstart,Xend,10,10,3);
carttrjctry = CartesianTrajectory(Xstart,Xend,10,10,5)
