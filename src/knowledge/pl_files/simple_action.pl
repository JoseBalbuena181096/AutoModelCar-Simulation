% Distance in meters
min_thresh(0.4).
max_thresh(1.0).

wichAction(short, stop).
wichAction(medium, slowdown).
wichAction(long, go).

howFar(Laserreading, Distance):- 
           min_thresh(Min), Laserreading =< Min, 
           Distance = short;
           min_thresh(Min), max_thresh(Max), Laserreading > Min, Laserreading =< Max, Distance = medium; 
           max_thresh(Max), Laserreading > Max, Distance = long.

% Laserreading is the minimum of the frontal laser beams
action(Laserreading, Action):- howFar(Laserreading, Distance),  wichAction(Distance, Action), format('~s ~w ~w', ['The distante to an obstacle is ', Distance, Action]), nl, !. 


/*


% Distance in meters
min_thresh(5).
max_thresh(10).

getAction(inFront, stop).
getAction(near, slowdown).
getAction(far, go).

isinFront(Laserreading):- min_thresh(X), Laserreading =< X.
isitNear(Laserreading):- min_thresh(Min), max_thresh(Max), Laserreading > Min, Laserreading =< Max.
isitFar(Laserreading):- max_thresh(X), Laserreading > X.

% Laserreading is an average of the frontal laser beams
action(Laserreading, Action):- isinFront(Laserreading), format('~s ', ['An obstacle is near']), nl, getAction(inFront, Action), !. 
action(Laserreading, Action):- isitNear(Laserreading),  format('~s ', ['Be careful']), nl, getAction(near, Action), !. 
action(Laserreading, Action):- isitFar(Laserreading),  format('~s ', ['No obstacle']), nl, getAction(far, Action). 

*/

