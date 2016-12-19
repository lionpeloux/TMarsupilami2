using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore.Sections
{
    public enum SectionType
    {
        HollowCircular,
        SolidCircular,

        HollowSquare,
        SolidSquare,

        HollowRectangular,
        SolidRectangular,

        IPN,
        UPN,

        Custom
    }

    /// <summary>
    /// We assume that X is the first principal axis of inertia,
    /// and Y is the second principal axis of inertia.
    /// </summary>
    public struct Section
    {
        #region PROPERTIES

        /// <summary>
        /// Gets the type of this section.
        /// </summary>
        public SectionType Type { get; private set; }

        /// <summary>
        /// Returns true if I1 = I2.
        /// </summary>
        public bool HasIsotropicCrossSection { get; private set; }

        /// <summary>
        /// Gets the section area (m^2).
        /// </summary>
        public double S { get; private set; }

        /// <summary>
        /// Gets the first principal moment of inertia (m^4).
        /// </summary>
        public double I1 { get; private set; }

        /// <summary>
        /// Gets the second principal moment of inertia (m^4).
        /// </summary>
        public double I2 { get; private set; }

        /// <summary>
        /// Gets the St. Venant <a href="https://www.wikiwand.com/en/Torsion_constant">torsional constant</a> (m^4).
        /// </summary>
        public double J { get; private set; }

        /// <summary>
        /// Gets the wraping torsional constant (m^4).
        /// </summary>
        public double Cw { get; private set; }

        #endregion

        #region CONSTRUCTOR
        private Section(SectionType type, double S, double I1, double I2, double J)
        {
            if (Math.Abs(I1 - I2) < 1e-6)
            {
                HasIsotropicCrossSection = true;
            }
            else
            {
                HasIsotropicCrossSection = false;
            }

            this.Type = type;
            this.S = S;
            this.I1 = I1;
            this.I2 = I2;
            this.J = J;
        }
        #endregion

        #region FACTORY
        public static Section Make_SolidCircular(double R)
        {
            double R2 = R * R;
            double S = Math.PI * R2;
            double I = S * R2 / 4;
            double J = 2 * I;
            return new Section(SectionType.HollowCircular, S, I, I, J);
        }
        public static Section Make_HollowCircular(double Rint, double Rext)
        {
            double R2int = Rint * Rint;
            double R2ext = Rext * Rext;
            double S = Math.PI * (R2ext - R2int);
            double I = S * (R2ext + R2int) / 4;
            double J = 2 * I;
            return new Section(SectionType.HollowCircular, S, I, I, J);
        }
        public static Section Make_SolidRectangular(double b1, double b2)
        {
            // section properties
            double S = b1 * b2;
            double I1 = b1 * Math.Pow(b2, 3) / 12;
            double I2 = b2 * Math.Pow(b1, 3) / 12;

            double a = Math.Max(b1, b2);
            double b = Math.Min(b1, b2);
            double J = a * Math.Pow(b, 3) * (0.333 - 0.21 * (b / a) * (1 - Math.Pow(b / a, 4) / 12));

            if (Math.Abs(b1 - b2) < 1e-4)
            {
                return new Section(SectionType.SolidRectangular, S, I1, I2, J);
            }
            else
            {
                return new Section(SectionType.SolidSquare, S, I1, I2, J);
            }
        }
        #endregion

        #region INSTANCE METHODS
        // probably a lot of work here, but could lead to nice vizualization of stresses in the structure.
        // This is necessary to evaluate resitance criterion (with Von Mises or Tresca for instance).

        // return the axial strain (σ) in the section for a given point in the section
        public Vector σ(double N, double M1, double M2, double Q)
        {
            throw new NotImplementedException();
        }

        // return the shear stress (τ = (τ3, τ1, τ2)) in the secetion for a given point in the section
        public Vector τ(double N, double M1, double M2, double Q)
        {
            throw new NotImplementedException();
        }
        #endregion
    }

}
