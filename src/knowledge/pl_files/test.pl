/* Hechos */
mama_de(lupe, maria).
/* Reglas */
es_hijo_de(X,Y):-papa_de(Y,X), format('~w es papá de ~w~n',[Y,X]),!.
es_hijo_de(X,Y):-mama_de(Y,X), format('~w es mamá de ~w~n',[Y,X]),!.

/*:-es_hijo_de(_X,_Y).*/ 
/*El tema es poner el goal, si no se pone no despliega nada al cargar el programa*/

