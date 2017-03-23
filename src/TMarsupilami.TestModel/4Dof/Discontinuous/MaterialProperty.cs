using System;
using System.Collections.Generic;
using System.Text;

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    /* TODO :
     * prendre en compte des lois de raideur du type E(ε). Le sens de parcours importe.
     */
    public enum StandardMaterials
    {
        Steel,
        Aluminium,
        Timber,
        GFRP
    }

    public sealed class MaterialProperty : Property
    {

        delegate double del_E(double  ε);   // specific stifness low with a delagate

        #region FIELDS
        private double _ρ;      // material density [kg/m3]
        private double _E;      // elastic modulus [Pa]
        private double _G;      // shear modulus [Pa]
        private double _ν;      // poisson's ratio
        private double _α;      // coefficient ok thermal expension [/°C]
        #endregion  

        #region PROPERTIES
        /// <summary>
        /// Material density in [kg/m3]
        /// </summary>
        public double ρ
        {
            get { return _ρ; }
            protected set { _ρ = value; }
        }
        
        /// <summary>
        /// Elastic modulus in [Pa]
        /// </summary>
        public double E
        {
            get { return _E; }
            protected set { _E = value; }
        }
        
        /// <summary>
        /// Shear modulus in [Pa]
        /// </summary>
        public double G
        {
            get { return _G; }
            set { _G = value; }
        }
        
        /// <summary>
        /// Poisson's ratio
        /// </summary>
        public double ν
        {
            get { return _ν; }
            set { _ν = value; }
        }
        
        /// <summary>
        /// Coefficient of thermal expansion in [/°C]
        /// </summary>
        public double α
        {
            get { return _α; }
            set { _α = value; }
        }
        
        #endregion

        #region CONSTRUCTORS
        public MaterialProperty(int id, string name, PropertyType type,
                                double ρ, double E, double G, double ν , double α)
            :base(id, name, type)
        {
            this.ρ = ρ;
            this.E = E;
            this.ν = ν;
            this.G = G;
            this.α = α; 
        }
        public MaterialProperty(StandardMaterials type)
            : base(0, PropertyType.Material)
        {
            switch (type)
            {
                case StandardMaterials.Steel:
                    Name = "Standard Steel";
                    ρ = 8000;
                    E = 210e9;
                    ν = 0.30;
                    G = E / (2 * (1 + ν));
                    α = 1.2e-5;
                    return;

                case StandardMaterials.Aluminium:
                    Name = "Standard Aluminium";
                    ρ = 2710;
                    E = 70e9;
                    ν = 0.34;
                    G = E / (2 * (1 + ν));
                    α = 2.3e-5;
                    return;

                case StandardMaterials.Timber:
                    Name = "Standard Wood (C14)";
                    ρ = 350;
                    E = 7e9;
                    ν = 0.30;
                    G = 0.44e9;
                    α = 5.0e-5;
                    return;

                case StandardMaterials.GFRP:
                    Name = "Standard GFRP";
                    ρ = 1800;
                    E = 26e9;
                    ν = 0.28;
                    G = 3e9;
                    α = 1.1e-5;
                    return;
            }
        }
        #endregion

        #region METHODS
        // FOR NON ELASTIC MATERIAL
        // Update E(ε) according to the deformation ε (only for pure axial behavior)
        // For flexural behavior how is this working (=> modified EI according to κ ? => moment plastique.
        // because κ => triangular ε ie E vary according to v, the position of the fiber)
        // For torsion it is ok directly with G(τ)
        // On peut aussi controler le sens de parcours sur la courbe et modifier le ε poru les déformations irréversibles => historique de ε.
        
        // en fait, si l'on regarde le bouquin de Frey p270 on pourrait s'en sortir avec le concept de rotule plastique localisée.
        // Quand M = Me en un noeud on coupe la poutre en 2 et on met un ressort entre les 2 extrémités pour poursuivre le chemin quasistatique.
        /// voir aussi : Évolutions quasi-statiques en élasto-plasticité infinitésimale
        public void UpdateE(ref double[] E, ref double[] ε)
        {
            throw new NotImplementedException();
        }
        public void UpdateE(double E, double ε)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
