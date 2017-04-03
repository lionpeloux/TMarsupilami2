using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    /// <summary>
    /// Tuyauterie pour faire fonctionner un element (1D) dans un solveur de relaxation dynamique.
    /// </summary>
    public interface IDRElement
    {
        /// <summary>
        /// Indicates wheter the element should be taken into account in the secondary relaxation process over θ.
        /// </summary>
        bool IsTorsionCapable { get; }

        /// <summary>
        /// Gets the number of element vertices.
        /// </summary>
        int Nv { get; }
        int Ne { get; }

        MVector[] Rx { get; }
        double[] Rθ { get; }

        double[] LMx { get; }
        double[] LMθ { get; }
        MVector[] Vx { get; }
        MVector[] Vθ { get; }
        MVector[] Ax { get; }
        MVector[] Aθ { get; }

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

        // FOR CONSTRAINT INTERACTION
        double[] l { get; }
        double[] EI1 { get; }
        double[] EI2 { get; }
        MVector[] e { get; }
        MFrame[] MaterialFrames { get; }
        MVector[] Rx_int { get; }
        double[] Rθ_int { get; }
        MVector[] Fext_g { get; }
        MVector[] Mext_m { get; }

        MVector[] Fr_g { get; }
        MVector[] Mr_m { get; }

    }
}
