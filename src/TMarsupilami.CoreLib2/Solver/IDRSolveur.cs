using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    /// <summary>
    /// Tuyauterie pour faire fonctionner une contrainte dans un solveur de relaxation dynamique
    /// </summary>
    interface IDRConstraint
    {
        void Init();
        void Enforce_Mr();
        void Enforce_Qr();
        void Enforce_Fr();
    }

    /// <summary>
    /// Tuyauterie pour faire fonctionner un element (1D) dans un solveur de relaxation dynamique.
    /// </summary>
    interface IDRElement : ISolveur
    {
        /// <summary>
        /// Indicates wheter the element should be taken into account in the secondary relaxation process over θ.
        /// </summary>
        bool IsTorsionCapable { get; }

        /// <summary>
        /// Gets the number of element vertices.
        /// </summary>
        int Nv { get; }

        MVector[] Rx { get; }
        double[] Rθ { get; }

        // GEOMETRY
        /// <summary>
        /// Translates the actual vertices by dx (x = x + dx).
        /// </summary>
        /// <param name="dx">The translation vector.</param>
        void Move(MVector[] dx);

        /// <summary>
        /// Rotates the actual material frames by dθ around their ZAxis (θ = θ + dθ).
        /// </summary>
        /// <param name="dθ">The rotation angle in radians.</param>
        void Move(double[] dθ);

        void UpdateCenterlineProperties();
        void UpdateCurvatureBinormal();
        void UpdateMaterialFrame();

        // MOMENTS
        void UpdateBendingMoment();
        void UpdateTwistingMoment();
        void UpdateInternalNodalMoment();
        void UpdateResultantNodalMoment();

        // FORCES
        void UpdateAxialForce();
        void UpdateShearForce();
        void UpdateInternalNodalForce();
        void UpdateResultantNodalForce();

        // LUMPED MASSES
        void Update_lm_x(ref double[] lm_x);
        void Update_lm_θ(ref double[] lm_θ);
    }

    interface IDRSolver
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
