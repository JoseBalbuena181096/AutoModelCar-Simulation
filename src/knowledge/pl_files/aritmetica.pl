
/* Prolog sigue negación por */
negacion(X):- not(X).
negacion1(X):- \+ X.
negacion2(X):- not(5 > X).
conjuncion(X,Y):- (X, Y).
disyuncion(X,Y):- (X ; Y).
siempre_verdad:- true.
siempre_falsa:- false.

/* Para desplegar X en términos de los functores +,-,/,*, */
/* Útil para revisar el orden de precedencia */
precedencia:-write(X is 10 + 5 * 6 / 3 - 1), write(' '), display(X is 10 + 5 * 6 / 3 - 1), nl, !.

/* Desigualdades */
/* =\= Verdad si las expresiones X y Y no evalúan a la misma cantidad*/
compara(X,Y) :- X =\= Y, format('~w ~s ~w', [X,'no es igual a', Y]),  fail.
compara(X,Y) :- X > Y, format(' ~s ~w ~s ~w', ['y', X,'es más grande que', Y]), nl, !.
compara(X,Y) :- X < Y, format(' ~s ~w ~s ~w', ['y', X,'es más pequeño que', Y]), nl, !.
compara(X,Y) :- X is Y, format('~w ~s ~w', [X,'es igual a', Y]), !.
/* Verdad si =:= las expresiones X y Y evaluan a la misma cantidad */
/* compara(X,Y) :- X =:= Y, format('~w ~s ~w', [X,'es igual a', Y]), !. */

comparacion:- write('Dame el primer número: '), read(X), nl, write('Dame el segundo número: '), read(Y), compara(X, Y), nl,!.

/* Operaciones aritmeticas */
resta :- write('Dame el primer número: '), read(X), nl, write('Dame el segundo número: '), read(Y), Z1 is X - Y, format('~s~w', ['El resultado es ', Z1]), write(' que es una salida diferente a '), Z2=X/Y, format('~w', [Z2]), nl, !.

division :- write('Dame el primer número: '), read(X), nl, write('Dame el segundo número: '), read(Y), Z1 is X / Y, format('~s~w', ['El resultado es ', Z1]), write(' que es una salida diferente a '), Z2=X/Y, format('~w', [Z2]), nl, !.

multiplicacion :- write('Dame el primer número: '), read(X), nl, write('Dame el segundo número: '), read(Y), Z1 is X * Y, format('~s~w', ['El resultado es ', Z1]), write(' que es una salida diferente a '), Z2=X*Y, format('~w', [Z2]), nl, !.

suma :- write('Dame el primer número: '), read(X), nl, write('Dame el segundo número: '), read(Y), Z1 is X + Y, format('~s~w', ['El resultado es ', Z1]), write(' que es una salida diferente a '), Z2=X+Y, format('~w', [Z2]), nl, !.

/* Ejemplos a probar.

:-3 = 3.
:-7 is 6.
:-2 = 3 - 1.
:-2 == 3 - 1.
:-2 is 3 - 1.

*/
