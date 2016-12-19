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
        /// However, compressive and tension elastic modulus can be distinct.
        /// </summary>
        /// <remark>
        /// This class is marked private. It is nested inside the Material class.
        /// It can be instanciated via the Material class which behaves as a factory.
        /// Only materials of the parent type (Material) are manipulated by the end user.
        /// </remark>
        sealed private class M_ASymLinear : Material
        {
            // Private fields
            private double E0c, E0t;
            private double G0;
            private double α0;

            public M_ASymLinear(double density,  double ν, double α0, double E0c, double E0t, double G0)
                : base(density, ν, ConstitutiveLawType.AsymLinear, ConstitutiveLawType.SymLinear, ConstitutiveLawType.SymConstant)
            {                
                this.E0c = E0c;
                this.E0t = E0t;
                this.G0 = G0;
                this.α0 = α0;
            }

            public override double E(double ε)
            {
                if (ε >= 0) return E0c;
                else return E0t;         
            }
            public override double σ(double ε)
            {
                if (ε >= 0) return E0c * ε;
                else return E0t * ε;
            }
            public override void σE(double ε, out double σ, out double E)
            {
                if (ε >= 0)
                {
                    E = E0c;
                    σ = E0c * ε;
                }
                else
                {
                    E = E0t;
                    σ = E0t * ε;
                }          
            }

            public override double τ(double γ)
            {
                return G0 * γ;
            }
            public override double G(double γ)
            {
                return G0;
            }
            public override void τG(double γ, out double τ, out double G  )
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
