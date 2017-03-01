using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilamiCore.Materials;

namespace TMarsupilamiCore.Materials
{
    partial class Material
    {
        /// <summary>
        /// Gets a generic material. 
        /// </summary>
        /// <remark>
        /// This class is marked private. It is nested inside the Material class.
        /// It can be instanciated via the Material class which behaves as a factory.
        /// Only materials of the parent type (Material) are manipulated by the end user.
        /// </remark>
        sealed private class M_Generic : Material
        {
            private Law_F del_E;
            private Law_F del_σ;
            private Law_FdF del_σE;
            private Law_F del_G;
            private Law_F del_τ;
            private Law_FdF del_τG;
            private Law_F del_α;

            public M_Generic(
                double density, double ν,
                ConstitutiveLawType elasticLaw, Law_F σ, Law_F E, Law_FdF σE,
                ConstitutiveLawType shearLaw, Law_F τ, Law_F G, Law_FdF τG,
                ConstitutiveLawType thermalLaw, Law_F α
                ) : base(density, ν, elasticLaw, shearLaw, thermalLaw)
            {
                base.ν = ν;

                this.ElasticBehavior = elasticLaw;
                this.del_E = E;
                this.del_σ = σ;
                this.del_σE = σE;

                this.ShearBehavior = shearLaw;
                this.del_G = G;
                this.del_τ = τ;
                this.del_τG = τG;

                this.ThermalBehavior = thermalLaw;
                this.del_α = α;
            }

            // Elastic stress and modulus
            public override double σ(double ε)
            {
                double σ;
                del_σ(ε, out σ);
                return σ;
            }
            public override double E(double ε)
            {
                double E;
                del_E(ε, out E);
                return E;
            }
            public override void σE(double ε, out double σ, out double E)
            {
                del_σE(ε, out σ, out E);
            }

            // Shear stress and modulus
            public override double τ(double γ)
            {
                double τ;
                del_τ(γ, out τ);
                return τ;
            }
            public override double G(double γ)
            {
                double G;
                del_G(γ, out G);
                return G;
            }
            public override void τG(double γ, out double τ, out double G)
            {
                del_τG(γ, out τ, out G);
            }

            // Coefficient of thermal expansion
            public override double α(double T)
            {
                double α;
                del_α(T, out α);
                return α;
            }

                      
        }
    }
}
