using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore
{

    // definir une algèbre pour pouvoir combiner les charges : a1 * L1 + a2 * L2.
    // comment charger les poutres avec des ghosts nodes ??
    // un ghost node est "ghost", il doit être invisible pour le "chargeur".
    // ou bien c'est l'élement qui s'applique une Load.

    /* cette idée me semble bonne, dans la mesure où c'est l'élément, 
     * en fonction du modèle mécanique qu'il implémente, 
     * qui pourra refuser ou non de s'appliquer un effort.
     * Par exmple pour la poutre 3pts, pas de load de torsion possible.
     */
    public abstract class Load
    {
        /* 3 types de charge :
         * Fext, fext, Mext, mext
         * 
         * Selon 2 types de repères :
         * Global ou Local
         */


    }
}
