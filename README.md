# TMasupilami

rhino & grasshopper set of tools for dynamic relaxation analysis

##Contributors

Lionel du Peloux


## WIP

modèle de poutre simple semble fonctionnel :

- les boundary conditions sont désormais basées sur des conditions en force et non plus en vitesse, ce qui les rend indépendantes du solveur

- la condition `FREE` n'a plus lieu d'être et doit être retirée (comportement par défaut d'un noeud)

- la condition `CLAMPED` fait apparaître un problème de définition de l'effort normal au niveau des noeuds, et donc de l'encastrement. Cf le cas de la poutre circulaire encastrée d'un côté libre de l'autre => la courbure transforme l'effort normal en effort tranchant et l'effort tranchant en effort normal. Et ce phénomène conduit à une approximation qui donne des résultantes sur appuis faussées (légèrement, selon le niveau de discrétisation)
- Intégrer un appuis élastique (sur la rotule puis l'encastrement)

- poutre courbe

- section variable

- reprendre au propre les champs & propriétés C#

## informations sur la branche :

branch_id = 1beam_master

Dans cette branche, on essai de donner l'implémentation la plus complète possible d'une poutre seule avec des conditions aux limites variées et des chargements extérieurs. Cette branche devrait servir de base à l'implémentation du modèle à N poutres interconnectées. Il s'agit également d'aborder les compatibilités et incompatibilités entre les différents conditions aux limites.

TYPE      | DESCRIPTION    | X | Y | Z | XX | YY | ZZ | Mext
----------|----------------|---|---|---|----|----|----|-----
`Free`    | appui libre    | 0 | 0 | 0 | 0  | 0  | 0  | M0
`Pinned`  | appui rotulé   | 1 | 1 | 1 | 0  | 0  | 0  | 0
`Clamped` | appui encastré | 1 | 1 | 1 | 1  | 1  | 1  |

- `PIN`: appui rotulé (x,y,z) bloqués et (xx, yy, zz) libres
- `ENC`: appuis encastré (x,y,z) bloqués (xx, yy, zz) libres
