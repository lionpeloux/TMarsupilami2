
using TMarsupilami.MathLib;

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    public enum ElementType
    {
        Beam,   // N nodes : tension & compression & bending & torsion
        Chain,  // N nodes : tension & compression
        Cable,  // N nodes : tension only
        Bar,    // 2 nodes : tension & compression
    }
    public enum BeamType
    {
        Straight,
        Curved,
        Closed
    }
    public enum ChainType
    {
        Simple,
    }
    public enum CableType
    {
        Simple,
        Sliding
    }
    public enum BarType
    {
        Simple,
        CompressionOnly,
        TensionOnly
    }

    public abstract class Element : Relation
    {
        #region FIELDS
        private ElementType _type;              // Type
        private bool _is_closed;                // is the element looping over itself ?

        private int nn;                        // number of nodes (>1)
        private int ne;                        // number of edges (>0)
        
        // Properties (uniformes sur la poutre pour l'instant)
        private SectionProperty[] _section_prop;      // One SectionProperties object per element
        private MaterialProperty _material_prop;    // One MaterialProperties object per element
        private double[] lm;                        // nodal lumped mass deduced from material density and section properties

        // REST CONFIGURATION
        public MFrame[] _mframe_0;      // material frames in actual (deformed) configuration
        public MPoint[] x_0;            // centerline
        public MVector[] e_0;           // edge
        public MVector[] t_0;           // tangent
        public MVector[] κb_0;          // curvature binormal
        public double[] l_0, ll_0;      // rest length of edges
        public double[] κ1_0, κ2_0;     // material curvature in rest configuration
        public double[] twist_0, τ_0;   // twist angle and rate of twist (τ) in rest configuration

        // DEFORMED CONFIGURATION
        private MFrame[] _mframe;       // material frames in actual (deformed) configuration
        public MVector[] d3_mid;        // d3 material axis evaluated at mid-edge.
        public MPoint[] x;              // centerline : x[i]
        public MVector[] e;             // edge : e[i] = x[i+1] - x[i]
        public MVector[] u;             // unit edge : u[i] = e[i]/l[i]

        public MVector[] t;             // tangent
        public MVector[] κb;            // curvature binormal at ghost node
        protected MVector[] κb_mid;     // curvature binormal evaluated at mid-edge.

        public double[] l;              // edge length : l[i] = |e[i]|
        protected double[] ll;          // ll[i] = |e[i] + e[i+1]|
        public double[] ε;              // axial strain : ε = l[i]/l[i]_0 - 1
        public double[] κ1, κ2;         // material curvature : κ1 =  κb.d1 | κ2 =  κb.d1
        public double[] twist, τ;       // twist angle and rate of twist : τ[i] = twist[i] / l[i]

        // EXTERNAL FORCES & MOMENTS
        private MVector[] _Fext_g;      // (Fx, Fy, Fz) given in the global coordinate system (x,y,z)
        private MVector[] _Mext_m;      // (M1, M2, Q) given in the material coordinate system(d1, d2, t)
        public MVector[] _fext_g;       // linear external moment
        public MVector[] _mext_m;       // linear external moment

        // REACTION MOMENT AND FORCES
        // take care about signs : these are the forces and moments applied
        // by the beam to the supports. 
        public MVector[] _Fr_g;         // (Frx, Fry, Frz) given in the global coordinate system
        public MVector[] _Mr_m;         //   (Mr1, Mr2, Qr) given in the material coordinate system

        // INTERNAL FORCES & MOMENTS
        private double[] _N;            // axial force : N = ES.ε
        private double[] _M1;           // bending moment around d1 material axis : M1 = EI1.(κ1-κ1_0)
        private double[] _M2;           // bending moment around d2 material axis : M2 = EI2.(κ2-κ2_0)
        private MVector[] _M;           // total bending moment : M = M1.d1 + M2.d2
        private double[] _Q;            // torsional moment : Q = GJ.(τ-τ_0)
        private MVector[] _V_M;         // shear force (bending contribution) : V_M = M'
        private MVector[] _V_Q;         // shear force (torsion contribution) : V_Q = Qκb
        private MVector[] _V;           // total shear force : V = V_M + V_Q
        
        // RESULTANTE FORCES (DR SPECIFIC ?? => si oui, à déplacer)
        public MVector[] Rx, Rx_int, Rx_axial, Rx_shear_M, Rx_shear_Q;
        public double[] Rθ, Rθ_int, Rθ_Q, Rθ_M;            

        #endregion

        #region PROPERTIES

        /// <summary>
        /// Element's type.
        /// </summary>
        public ElementType ElementType
        {
            get { return _type; }
            protected set { _type = value; }
        }

        /// <summary>
        /// Indicates wether the element is closed or looping over itself.
        /// </summary>
        public bool IsClosed
        {
            get { return _is_closed; }
            protected set { _is_closed = value; }
        }

        /// <summary>
        /// Get section properties.
        /// </summary>
        public SectionProperty[] SectionProperty
        {
            get { return _section_prop; }
            protected set { _section_prop = value; }
        }

        /// <summary>
        /// Get material properties.
        /// </summary>
        public MaterialProperty MaterialProperty
        {
            get { return _material_prop; }
            protected set { _material_prop = value; }
        }

        /// <summary>
        /// Get lumped mass. This is deduced from element's material density and section properties.
        /// </summary>
        public double[] LumpedMass
        {
            get { return lm; }
            protected set { lm = value; }
        }

        /// <summary>
        /// Number of nodes.
        /// </summary>
        public int Nn
        {
            get { return nn; }
            protected set { nn = value; }
        }

        /// <summary>
        /// Number of edges (Ne = Nn-1 for an open element | Ne = Nn for a closed element).
        /// </summary>
        public int Ne
        {
            get { return ne; }
            protected set { ne = value; }
        }

        /// <summary>
        /// Get material frames.
        /// </summary>
        public MFrame[] MaterialFrame
        {
            get { return _mframe; }
            protected set { _mframe = value; }
        }

        /// <summary>
        /// External forces applied to beam nodes. 
        /// Fext is given in the global coordinate system (x,y,z).
        /// </summary>
        public MVector[] Fext
        {
            get { return _Fext_g; }
            protected set { _Fext_g = value; }
        }
        public MVector[] fext
        {
            get { return _fext_g; }
            protected set { _fext_g = value; }
        }
        public MVector[] Fr
        {
            get { return _Fr_g; }
            protected set { _Fr_g = value; }
        }

        /// <summary>
        /// External moments applied to beam nodes. 
        /// Mext is given in the material coordinate system (d1,d2,t).
        /// Whenever Mext is modified, mext needs to be calculated again.
        /// This can be done by calling Update_mext(). A better solution would be not to give direct access
        /// </summary>
        public MVector[] Mext
        {
            get { return _Mext_m; }
            set { _Mext_m = value; }
        }
        public MVector[] mext
        {
            get { return _mext_m; }
            set { _mext_m = value; }
        }
        public MVector[] Mr
        {
            get { return _Mr_m; }
            protected set { _Mr_m = value; }
        }

        /// <summary>
        /// Internal axial force (applied from right to left).
        /// N is constant over each edge and is given at mid edge (i+1/2).
        /// </summary>
        public double[] N
        {
            get { return _N; }
            protected set { _N = value; }
        }

        /// <summary>
        /// Internal shear force | bending contribution. (applied from right to left).
        /// </summary>
        public MVector[] V_M
        {
            get { return _V_M; }
            protected set { _V_M = value; }
        }

        /// <summary>
        /// Internal shear force | torsion contribution (applied from right to left).
        /// </summary>
        public MVector[] V_Q
        {
            get { return _V_Q; }
            protected set { _V_Q = value; }
        }

        /// <summary>
        /// Internal total shear force (applied from right to left).
        /// </summary>
        public MVector[] V
        {
            get { return _V; }
            protected set { _V = value; }
        }

        /// <summary>
        /// Internal bending moment around d1 material axis (applied from right to left).
        /// </summary>
        public double[] M1
        {
            get { return _M1; }
            protected set { _M1 = value; }
        }

        /// <summary>
        /// Internal bending moment around d2 material axis (applied from right to left).
        /// </summary>
        public double[] M2
        {
            get { return _M2; }
            protected set { _M2 = value; }
        }

        /// <summary>
        /// Internal bending moment (applied from right to left).
        /// </summary>
        public MVector[] M
        {
            get { return _M; }
            protected set { _M = value; }
        }
        
        /// <summary>
        /// Internal torsional moment around d3 material axis (applied from right to left).
        /// </summary>
        public double[] Q
        {
            get { return _Q; }
            protected set { _Q = value; }
        }

        /// <summary>
        /// Axial elastic energy.
        /// </summary>
        public double Ea
        {
            get { return GetAxialElasticEnergy(); }
        }

        /// <summary>
        /// Bending elastic energy.
        /// </summary>
        public double Eb
        {
            get { return GetBendingElasticEnergy(); }
        }

        /// <summary>
        /// Twisting elastic energy.
        /// </summary>
        public double Et
        {
            get { return GetTwistingElasticEnergy(); }
        }

        #endregion

        #region CONSTRUCTORS
        protected Element(  int id, string name, ElementType type, 
                            SectionProperty[] section_prop, MaterialProperty material_prop,
                            bool is_closed, MFrame[] mframe_0, MFrame[] mframe_i)
        {
            Id = id;
            Name = name;
            ElementType = type;
            IsClosed = is_closed;

            Nn = mframe_0.Length;
            Ne = (is_closed) ? Nn : Nn - 1;

            SectionProperty = section_prop;
            MaterialProperty = material_prop;
            MaterialFrame = mframe_i;

            // REST CONFIGURATION
            _mframe_0 = new MFrame[Nn];
            x_0 = new MPoint[Nn];
            e_0 = new MVector[Ne];
            t_0 = new MVector[Nn];
            κb_0 = new MVector[Nn];
            
            l_0 = new double[Ne];
            ll_0 = new double[Ne - 1];  
            κ1_0 = new double[Nn];
            κ2_0 = new double[Nn];
            twist_0 = new double[Ne];
            τ_0 = new double[Ne];

            // DEFORMED CONFIGURATION
            _mframe = new MFrame[Nn];
            x = new MPoint[Nn];
            e = new MVector[Ne];
            t = new MVector[Nn];
            d3_mid = new MVector[Ne];
            κb = new MVector[Nn];
            κb_mid = new MVector[Ne];

            l = new double[Ne];
            ll = new double[Ne - 1];
            ε = new double[Ne];
            κ1 = new double[Nn];
            κ2 = new double[Nn];
            twist = new double[Ne];
            τ = new double[Ne];

            // EXTERNAL FORCES & MOMENTS
            Fext = new MVector[Nn];
            Mext = new MVector[Nn];
            mext = new MVector[Ne];
            fext = new MVector[Ne];

            // REACTION MOMENT AND FORCES
            Fr = new MVector[Nn];
            Mr = new MVector[Nn];

            // INTERNAL FORCES & MOMENTS
            N = new double[Ne];
            M1 = new double[Nn];
            M2 = new double[Nn];
            M = new MVector[Nn];
            Q = new double[Ne];
            V_M = new MVector[Ne];
            V_Q = new MVector[Ne];
            V = new MVector[Ne];

            // DR RESULTANT
            Rx = new MVector[Nn];
            Rx_int = new MVector[Nn];
            Rx_axial = new MVector[Nn];
            Rx_shear_M = new MVector[Nn];
            Rx_shear_Q = new MVector[Nn];
            Rθ = new double[Nn];
            Rθ_int = new double[Nn];
            Rθ_Q = new double[Nn];
            Rθ_M = new double[Nn];
        }
        
        #endregion

        #region METHODS
        
        public abstract override string ToString();

        protected abstract void SetRestConfig(ref MFrame[] mframe);
        protected virtual void SetRestConfig(double[] l_0, double[] κ1_0, double[] κ2_0, double[] τ_0)
        {
            this.l_0 = l_0;
            this.κ1_0 = κ1_0;
            this.κ2_0 = κ2_0;
            this.τ_0 = τ_0;
        }

        protected abstract void SetInitialConfig(ref MFrame[] mframe);

        // GEOMETRY
        /// <summary>
        /// Translate the element sections by dx (x = x + dx).
        /// WARNING : cette opération peut violer les conditions aux limites sur l'orientation des sections.
        /// Il convient donc de faire une validation à l'issu du déplacement.
        /// </summary>
        /// <param name="dx">The translation vector.</param>
        public virtual void Move(MVector[] dx)
        {
            // x = x + dx
            for (int i = 0; i < Nn; i++) 
            {
                x[i] = MaterialFrame[i].Origin + dx[i];
            }
        }

        /// <summary>
        /// Rotate the element sections by dθ (θ = θ + dθ).
        /// WARNING : cette opération peut violer les conditions aux limites sur l'orientation des sections.
        /// Il convient donc de faire une validation à l'issu du déplacement.
        /// </summary>
        /// <param name="dθ">The rotation angle in radians.</param>
        public virtual void Move(double[] dθ)
        {
            // θ = θ + dθ
            for (int i = 0; i < Nn; i++)
            {
                MaterialFrame[i].ZRotate(dθ[i]);
            }
        }

        // MOMENTS
        public abstract void GetBendingMoment();
        public abstract void GetTwistingMoment();
        public abstract void GetInternalNodalMoment();
        public abstract void GetResultantNodalMoment();

        // FORCES
        public abstract void GetAxialForce();
        public abstract void GetShearForce();
        public abstract void GetInternalNodalForce();
        public abstract void GetResultantNodalForce();

        // ENERGIES
        protected abstract double GetAxialElasticEnergy();
        protected abstract double GetBendingElasticEnergy();
        protected abstract double GetTwistingElasticEnergy();

        public virtual void Update_mext()
        {
            // convert Mext into mext
            // should be called whenever Mext is modified
            for (int i = 1; i < Nn - 1; i++)
            {
                MVector m = Mext[i] / (l_0[i - 1] + l_0[i]);
                mext[i - 1] += m;
                mext[i] = m;
            }
        }
        public virtual void Update_Fext()
        {
            // convert fext into Fext
            // should be called whenever Mext is modified
            for (int i = 0; i < Ne; i++)
            {
                MVector f = 0.5 * l[i] * fext[i];
                Fext[i] += f;
                Fext[i + 1] += f;
            }
        }
        #endregion
    }
}
