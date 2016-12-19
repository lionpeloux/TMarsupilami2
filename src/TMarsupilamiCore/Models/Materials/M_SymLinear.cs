using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore.Materials
{
    partial class Material
    {
        /// <summary>
        /// Gets an ideal linear-elastic material. 
        /// Compressive and tension elastic modulus are identical.
        /// </summary>
        /// <remark>
        /// This class is marked private. It is nested inside the Material class.
        /// It can be instanciated via the Material class which behaves as a factory.
        /// Only materials of the parent type (Material) are manipulated by the end user.
        /// </remark>
        sealed private class M_SymLinear : Material
        {
            // Elastic modulus
            private double E0;

            // Shear modulus
            private double G0;

            // Coefficient of thermal expansion
            private double α0;

            public M_SymLinear(double density, double ν, double α0, double E0, double G0)
                :base(density, ν, ConstitutiveLawType.SymLinear, ConstitutiveLawType.SymLinear, ConstitutiveLawType.SymConstant)
            {
                this.E0 = E0;
                this.G0 = G0;
                this.α0 = α0;
            }

            public override double E(double ε)
            {
                return E0;
            }
            public override double σ(double ε)
            {
                return E0 * ε;
            }
            public override void σE(double ε, out double σ, out double E)
            {
                E = E0;
                σ = E0 * ε;
            }

            public override double G(double γ)
            {
                return G0;
            }
            public override double τ(double γ)
            {
                return G0 * γ;
            }
            public override void τG(double γ, out double τ, out double G)
            {
                G = G0;
                τ = G0 * γ;
            }

            public override double α(double T)
            {
                return α0;
            }
        }
    }
}
