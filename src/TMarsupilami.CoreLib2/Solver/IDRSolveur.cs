using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    public interface IDRSolver : ISolveur
    {

        //public SolverConfig Config { get; set; }

        //// Je fais une deep copy du model pour faire mes calcul indépendament.
        //// Je peux retourner un objet Model à tout instant avec la position actuelle du solveur en appelant GetModel()
        //void Build(Model model, LoadCase loadCase, Conversion rules);

        //// A tout moment, je peux reconstruire un modèle dans l'état où est le solveur.
        //// Je travail sur un modèle indépendant.
        //Model GetModel();

        void Run();         // run one iteration
        void Run(int N);    // run N interations
        
         
    }

}
