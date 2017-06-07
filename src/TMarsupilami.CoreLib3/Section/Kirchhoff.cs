using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{

    /// <summary>
    /// In this class, put all helping methods that relates to computing section properties from Kirchhoff rod theory
    /// </summary>
    /// 
    public static class Kirchhoff
    {
        /// <summary>
        /// Gets the local displacement vector for a material point (X1,X2).
        /// </summary>
        /// <param name="X">First material coordinate, along d1.</param>
        /// <param name="Y">Second material coordinate, along d2.</param>
        /// <param name="W">The warping function of the section.</param>
        /// <param name="v">The Poisson coefficient.</param>
        /// <param name="ε">The elongation factor.</param>
        /// <param name="ϰ">The material curvature vector in the actual configuration.</param>
        /// <param name="ϰ_0">The material curvature vector in the reference configuration.</param>
        /// <returns>The displacement vector (u1, u2, u3) in the material coordinate system.</returns>
        public static MVector GetLocalDisplacement( double X, double Y, Func<double, double, double> W, 
                                                    double v, double ε, MVector ϰ, MVector ϰ_0)
        {
            var XX = X * X;
            var YY = Y * Y;
            var XY = X * Y;

            var u1 = -v * (ε * X + (ϰ.X - ϰ_0.X) * XY - 0.5 * (ϰ.Y - ϰ_0.Y) * (XX - YY));
            var u2 = -v * (ε * Y - (ϰ.Y - ϰ_0.Y) * XY - 0.5 * (ϰ.X - ϰ_0.X) * (XX - YY));
            var u3 = (ϰ.Z - ϰ_0.Z) * W(X, Y);

            var u = new MVector(u1, u2, u3);
            return u;
        }

        /// <summary>
        /// Gets the components of the strain tensor.
        /// </summary>
        /// <param name="X">First material coordinate, along d1.</param>
        /// <param name="Y">Second material coordinate, along d2.</param>
        /// <param name="DXW">First partial derivative of the warping function (dW/dX).</param>
        /// <param name="DYW">Second partial derivative of the warping function (dW/dY).</param>
        /// <param name="v">The Poisson coefficient.</param>
        /// <param name="ε">The elongation factor.</param>
        /// <param name="ϰ_0">The material curvature vector in the reference configuration.</param>
        /// <param name="ϰ_0">The material curvature vector in the reference configuration.</param>
        /// <param name="ε11">Strain tensor component.</param>
        /// <param name="ε22">Strain tensor component.</param>
        /// <param name="ε33">Strain tensor component.</param>
        /// <param name="ε12">Strain tensor component. Recall that ε12 = ε21.</param>
        /// <param name="ε13">Strain tensor component. Recall that ε13 = ε31.</param>
        /// <param name="ε23">Strain tensor component. Recall that ε23 = ε32.</param>
        public static void GetStrains(double X, double Y, Func<double, double, double> DXW, Func<double, double, double> DYW,
                                                    double v, double ε, MVector ϰ, MVector ϰ_0, 
                                                    out double ε11, out double ε22, out double ε33,
                                                    out double ε12, out double ε13, out double ε23)
        {
            ε33 = ε - (ϰ.Y - ϰ_0.Y) * X + (ϰ.X - ϰ_0.X) * Y;
            ε22 = -v * ε33;
            ε11 = ε22;

            ε12 = 0;
            ε13 = 0.5 * (ϰ.Z - ϰ_0.Z) * (DXW(X, Y) - Y);
            ε23 = 0.5 * (ϰ.Z - ϰ_0.Z) * (DYW(X, Y) + X);
        }

        /// <summary>
        /// Gets the components of the stress tensor.
        /// </summary>
        /// <param name="ε13">Strain tensor component.</param>
        /// <param name="ε23">Strain tensor component.</param>
        /// <param name="ε33">Strain tensor component.</param>
        /// <param name="E">Axial modulus (GPa).</param>
        /// <param name="G">Shear modulus (GPa).</param>
        /// <param name="σ11">Stress tensor component.</param>
        /// <param name="σ22">Stress tensor component.</param>
        /// <param name="σ33">Stress tensor component.</param>
        /// <param name="σ12">Stress tensor component. Recall that σ12 = σ21.</param>
        /// <param name="σ13">Stress tensor component. Recall that σ13 = σ31.</param>
        /// <param name="σ23">Stress tensor component. Recall that σ23 = σ32.</param>
        public static void GetStresses( double ε13, double ε23, double ε33, double E, double G,
                                        out double σ11, out double σ22, out double σ33,
                                        out double σ12, out double σ13, out double σ23)
        {
            σ11 = 0;
            σ22 = 0;
            σ33 = E * ε33;

            σ12 = 0;
            σ13 = 2 * G * ε13;
            σ23 = 2 * G * ε23;
        }
    }
}
