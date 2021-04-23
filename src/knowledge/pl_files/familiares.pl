/* Hechos */
la_casa_es_azul.
mi_perro_ladra_mucho.
la_tierra_gira_alrededor_del_sol.
mama_de(lupe, maria).
mama_de(ana, pepito).
papa_de(luis, maria).
papa_de(pedro, pepito).
primo_de(maria,pepito).
hermano_de(lupe,ana).
suma(10, 2, 12). 
/* Reglas */
es_hijo_de(X,Y):-papa_de(Y,X).
es_hijo_de(X,Y):-mama_de(Y,X).
hermanos(X,Y):-hermano_de(X,Y),!.
hermanos(X,Y):-hermano_de(Y,X),!.
/* Suponiendo un mundo feliz y perfecto, claro: */
esposos(X,Y):-papa_de(X,Z),mama_de(Y,Z),!. 
esposos(X,Y):-papa_de(Y,Z),mama_de(X,Z),!.
papas_de(X,Y,Z):-papa_de(X,Z),mama_de(Y,Z), format('~w y ~w son papá y mamá de ~w~n',[X,Y,Z]),!.   
papas_de(X,Y,Z):-papa_de(Y,Z),mama_de(X,Z), format('~w y ~w son mamá y papá de ~w~n',[X,Y,Z]),!.
saluda:-write('Hola, dime tu nombre: '), read(X), write('Hola '), write(X).
/* Objetivo-Query */
/* underscore es para evitar warning por variable singleton (aparece sólo una vez en una cláusula)*/
/*Al declarar targets-goals se ejecutan automaticamente, si no se ponen no despliega nada al cargar el programa si tienes writes */
/* Probar sin comentar y comentados las siguientes líneas */
/*:-es_hijo_de(_X,_Y). 
:-hermanos(_X,_Y).
:-esposos(_X,_Y).
:-papas_de(_X,_Y,_Z).*/
