/* Hechos */
batt_ok.
liftable. 
/* Reglas */
moves :- batt_ok, liftable. 
/* Objetivo-Query */
:- moves.
