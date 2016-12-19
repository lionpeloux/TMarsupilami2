using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore.Solver
{
    abstract class Solver_DR
    {
        public SolverConfig Config { get; set; }


        
        // Je fais une deep copy du model pour faire mes calcul indépendament.
        // Je peux retourner un objet Model à tout instant avec la position actuelle du solveur en appelant GetModel()
        public abstract void Build(Model model, LoadCase loadCase, Conversion rules);

        // A tout moment, je peux reconstruire un modèle dans l'état où est le solveur.
        // Je travail sur un modèle indépendant.
        public abstract Model GetModel();

        public abstract void Init();
        public abstract void Run();
        public abstract void Run(int N);
       


    }
}
