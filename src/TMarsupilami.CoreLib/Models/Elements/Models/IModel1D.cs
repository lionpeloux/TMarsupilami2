
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib.Elements.Models
{
    /// <summary>
    /// Provides the necessary interface for capable Dynamic Relaxation 1D Elements
    /// </summary>
    interface IModel1D : IModel
    {
        /// <summary>
        /// Gets the number of element vertices.
        /// </summary>
        int Nv { get; }

        /// <summary>
        /// Gets the number of element edges.
        /// </summary>
        int Ne { get; }

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

        // ENERGIES
        double GetAxialElasticEnergy();
        double GetBendingElasticEnergy();
        double GetTwistingElasticEnergy();
    }
}
