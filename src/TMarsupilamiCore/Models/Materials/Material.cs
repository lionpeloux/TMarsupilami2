using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilamiCore;

namespace TMarsupilamiCore.Materials
{
    // Direct access to classic materials
    public enum MaterialPredefinedType
    {
        Steel_255,
        Steel_355,
        Aluminium,
        GFRP,
        Custom
    }

    /// <summary>
    /// Base class to represent any kind of material.
    /// </summary>
    /// <remarks>
    /// I could not find a way to enforce immutability of Material, 
    /// to prevent the construction of Material[] arrays where all items refer to the same object.
    /// This is prolematic when playing with a material for a given edge during the simulation process.
    /// Imagine you want to make an edge breakable : when this element becomes overloaded, 
    /// you can make its Young modulus null for the rest of the simulation.
    /// Its possible to make Material a struct but thus I could not derive specialized materials from it.
    /// More over, struct leads to boxing when accessed through the interface they implement.
    /// Is there any workaround ?? I don't think so ...
    /// </remarks>
    public abstract partial class Material
    {
        #region PROPERTIES

        /// <summary>
        /// Gets the type of elastic behavior this material is subject to.
        /// </summary>
        public ConstitutiveLawType ElasticBehavior { get; protected set; }

        /// <summary>
        /// Gets the type of shear behavior this material is subject to.
        /// </summary>
        public ConstitutiveLawType ShearBehavior { get; protected set; }

        /// <summary>
        /// Gets the type of thermal expansion behavior this material is subject to.
        /// </summary>
        public ConstitutiveLawType ThermalBehavior { get; protected set; }

        /// <summary>
        /// Just an idea. Migth be worth to implement it in a subclass BreakableMaterial ?
        /// /// </summary>
        public bool IsBreakable { get; protected set; }

        /// <summary>
        /// Gets Poisson's ratio.
        /// </summary>
        public double ν { get; protected set; }

        /// <summary>
        /// Gets the density.
        /// </summary>
        public double Density { get; protected set; }

        #endregion

        #region CONSTRUCTORS
        protected Material(double density, double ν,
            ConstitutiveLawType elasticLaw,
            ConstitutiveLawType shearLaw,
            ConstitutiveLawType thermalLaw)
        {
            IsBreakable = false;
            Density = density;
            this.ν = ν;
            this.ElasticBehavior = elasticLaw;
            this.ShearBehavior = shearLaw;
            this.ThermalBehavior = thermalLaw;
        }
        #endregion

        #region FACTORY
        /// <summary>
        /// Make a generic material as specified by the user.
        /// </summary>
        /// <param name="density">The density.</param>
        /// <param name="ν">Poisson's ration.</param>
        /// <param name="elasticLaw">The type of the elastic strain-stress law σ(ε).</param>
        /// <param name="σ">The law : ε -> σ(ε).</param>
        /// <param name="E">The law : ε -> E(ε)=σ'(ε).</param>
        /// <param name="σE">The law : ε -> (σ(ε),E(ε)).</param>
        /// <param name="shearLaw">The type of the shear strain-stress law τ(γ).</param>
        /// <param name="τ">The law : γ -> τ(γ).</param>
        /// <param name="G">The law : γ -> G(γ)=τ'(γ).</param>
        /// <param name="τG">The law : γ -> (τ(γ),G(γ)).</param>
        /// <param name="thermalLaw">The type of the thermal alpha-temperature law α(T).</param>
        /// <param name="α">The law : T -> α(T).</param>
        /// <returns>A Material.</returns>
        public static Material Make_Generic
            (double density, double ν,
            ConstitutiveLawType elasticLaw, Law_F σ, Law_F E,  Law_FdF σE,
            ConstitutiveLawType shearLaw, Law_F τ, Law_F G, Law_FdF τG,
            ConstitutiveLawType thermalLaw, Law_F α
            )
        {
            return new M_Generic(
                density, ν,
                elasticLaw, E, σ, σE,
                shearLaw, G, τ, τG,
                thermalLaw, α
                );
        }
        public static Material Make_SymLinear(double density, double ν, double α0, double E0, double G0)
        {
            return new M_SymLinear(density, ν, α0, E0, G0);
        }
        public static Material Make_ASymLinear(double density, double ν, double α0, double E0c, double E0t, double G0)
        {
            if (E0c == E0t)
            {
                return new M_SymLinear(density, ν, α0, E0c, G0);
            }
            else
            {
                return new M_ASymLinear(density, ν, α0, E0c, E0t, G0);
            }     
        }
        #endregion
        
        #region INSTANCE METHODS

        // ELASTIC BEHAVIOR

        /// <summary>
        /// Gets the axial stress (σ) for a given axial strain (ε).
        /// </summary>
        /// <param name="ε">The current axial strain in the material.</param>
        /// <returns>The actual axial stress [Pa] in the material.</returns>
        public abstract double σ(double ε);

        /// <summary>
        /// Gets the elastic modulus (E) for a given axial strain (ε).
        /// </summary>
        /// <param name="ε">The current axial strain in the material.</param>
        /// <returns>The elastic modulus [Pa].</returns>
        public abstract double E(double ε);

        /// <summary>
        /// Gets both the axial modulus (E) and the axial stress (σ) for a given axial strain (ε).
        /// </summary>
        /// <param name="ε">The current axial strain in the material.</param>
        /// <param name="σ">The axial stress [Pa].</param>
        /// <param name="E">The elastic modulus [Pa].</param>
        public abstract void σE(double ε, out double σ, out double E);

        // SHEAR BEHAVIOR

        /// <summary>
        /// Gets the shear stress (G) for a given shear strain (γ).
        /// </summary>
        /// <param name="γ">The actual shear strain in the material.</param>
        /// <returns>The actual shear stress (Pa) in the material.</returns>
        public abstract double τ(double γ);

        /// <summary>
        /// Gets the shear modulus (G) for a given shear strain (γ).
        /// </summary>
        /// <param name="γ">The current axial strain in the material.</param>
        /// <returns>The elastic modulus [Pa].</returns>
        public abstract double G(double γ);

        /// <summary>
        /// Gets both the shear modulus (G) and the shear stress for a given shear strain (γ).
        /// </summary>
        /// <param name="γ">The current shear strain in the material.</param>
        /// <param name="τ">The shear stress [Pa].</param>
        /// <param name="G">The shear modulus [Pa].</param>
        public abstract void τG(double γ, out double τ, out double G);

        // THERMAL BEHAVIOR
        /// <summary>
        /// Gets the coefficient of thermal expansion (α) for a given temperature (T).
        /// </summary>
        /// <param name="T">The current temperature in the material.</param>
        /// <returns>The coefficient of thermal expansion [1/K].</returns>
        public abstract double α(double T);

        #endregion
    }

}
