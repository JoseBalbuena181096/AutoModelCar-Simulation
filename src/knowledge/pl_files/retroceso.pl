alto(martin).
alto(lupe).
alto(maria).

medio(adrian).
medio(javier).
medio(luis).

bajo(israel).
bajo(jose).
bajo(ana).

muy_alto(carlos).
muy_alto(cecilia).

estatura(X,baja) :- X < 1.6, !.
estatura(X,media) :- X >= 1.6, X < 1.80, !.
estatura(X,alta) :- X >= 1.80, X < 1.90, !.
estatura(X,muy_alta) :- X >= 1.99, !.




