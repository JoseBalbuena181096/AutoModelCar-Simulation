articulo(el). 
articulo(la).
articulo(los).
articulo(las).
articulo(un).
articulo(una).
articulo(unos).
articulo(unas).

sustantivo(niño).
sustantivo(niños).
sustantivo(niña). 
sustantivo(niñas). 
sustantivo(perro). 
sustantivo(perros).
sustantivo(gato).
sustantivo(gatos).
sustantivo(paleta). 
sustantivo(paletas).
 
verbo(come).
verbo(alimenta).
verbo(ladra). 
verbo(duerme). 
verbo(llama). 
verbo(acaricia).
verbo(lame).
verbo(comen).
verbo(alimentan).
verbo(ladran). 
verbo(duermen). 
verbo(llaman). 
verbo(acarician).
verbo(lamen).

predicadoVerbal([Verbo | Fin], Fin) :- verbo(Verbo), !.
predicadoVerbal([Verbo | Resto], Fin) :- verbo(Verbo), sujeto(Resto, Fin).

sujeto([Articulo, Sustantivo | Fin], Fin) :- articulo(Articulo), sustantivo(Sustantivo), !.
sujeto([Sustantivo | Fin], Fin) :- sustantivo(Sustantivo).

oracion(Inicio, Fin) :- sujeto(Inicio, Resto), predicadoVerbal(Resto, Fin).
 
frase(X) :- oracion(X, [ ]). 

/*frase([los, niños, lamen, paletas]).*/
