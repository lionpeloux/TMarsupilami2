# TMasupilami

rhino & grasshopper set of tools for dynamic relaxation analysis

##Contributors

Lionel du Peloux


## WIP

### Prise en compte de l'état repos :
- à partir des frames ou des courbures généralisées au repos ??
- traitement de epsilon et tau : moyenné sur un edge ou définis au bords pour coder une variation linéaire

### Resultantes Rx / Ro
- reprendre le calcul final dans l'élément
- vérifier le fonctionnement des charges linéaires (m1, m2, f1, f2, f3)
- passer Ro en global et plus local. Idem pour Vo et Ao ?

### Automatic refine (IRefinable)
- mise en place de principe effectuée (ça marche)
- doit marcher avec les charges

### Lumped Mass
- passer la méthode en déléguée avec une méthode par défaut
- Laisser la possibilité de changer le mode de calcul des masses ficitves par élément.
- préciser le calcul des masses fictives
- mettre en place le système de transfert des masses fictives

### Section & Material (IBreakable)
- préciser les classes
- doivent implementer le refine
- comment surcharger les classes pour un affichage dans Gh (section)
- gestion d'événements pour la plasticité / rupture

### Préciser hierarchy des éléménts (IDRElement ??)
- un élément a des vertex en propre
- soit 3DOF soit 6DOF
- possède position, vitesses, accélérations
- quelle interface / classe abstraint avec le solver ??

### Support
- reprendre le calcul du moment induit comme pour la force (après le calcul de la resistance)
- introduire les supports groupés (pin / clamped) avec torseur résultant au point d'attache
- introduire les supports élastiques

### Links
- base implémentée avec des ressorts de rappel
- essayer la méthode de projection

### attractors / springs
- introduire une classe générique d'attractor
- les rendres compatibles via des callbacks avec les types grasshoppers (courbes / surfaces).
- trouver un système d'interaction par évènement poru du recalcul auto.

### Diagrams
- créer des diagrams normalisés pour un affichage plus simple.
- penser un affichage par code couleur / gradient.


### Mix elements
- implementer un cable
- tester la mixité des éléments 3DOF/6DOF
