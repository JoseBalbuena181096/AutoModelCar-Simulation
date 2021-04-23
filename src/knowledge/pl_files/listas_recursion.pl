factorial(X, Y):- X >= 1, format('~w ', [X]), nl, X1 is X - 1,  factorial(X1, Y1), Y is X*Y1, !. 
factorial(_X, Y):-  Y is 1, nl.
factorial(X):-factorial(X, Y), format('~s ~w ~s ~w ', ['El factorial de', X, 'es', Y]).

cuenta_regresiva(X):- X >= 1, format('~w ', [X]), nl, X1 is X - 1,  cuenta_regresiva(X1), !.
cuenta_regresiva(X):- X=:=0, format('~s', ["Ignición"]), nl.

despliega_lista([H|T]):- length([H|T], L), L > 1, format('~s ~w ~s ~w ', ['Head es', H, 'y cola es', T]), nl, despliega_lista(T), !.
despliega_lista([H|_]):- length([H|_], L), L =:= 1, format('~s ~w ', ['El último elemento es', H]), nl.

/* Sólo un ejemplo */
head_tail([H|T], H, T).

 
