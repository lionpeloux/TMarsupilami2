using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum ElementStructure
    {
        Spring = 000,
        Link = 010,

        Tie = 100,
        Strut = 110,
        Bar = 120,
        Beam = 130,

        Membrane = 200,
        Plate = 210,
        Shell = 220,
    }
    public abstract class Element
    {
        public int Id { get; internal set; }
        public int Dimension { get; private set; }
        public ElementStructure Structure { get; private set; }

        // KINEMATIC CAPABILITIES
        private int _translationalDOF;
        public int TranslationalDOF
        {
            get
            {
                return _translationalDOF;
            }
            protected set
            {
                if (value < 0 || value > 3)
                    throw new ArgumentOutOfRangeException("TranslationalDOF", "The number of DOF must be set in [0,3]");
                else
                    _translationalDOF = value;
            }
        }
        private int _rotationalDOF;
        public int RotationalDOF
        {
            get
            {
                return _rotationalDOF;
            }
            protected set
            {
                if (value < 0 || value > 3)
                    throw new ArgumentOutOfRangeException("RotationalDOF", "The number of DOF must be set in [0,3]");
                else
                    _rotationalDOF = value;
            }
        }

        // MECHANICAL CAPABILITIES
        public bool IsCompressionCapable { get; protected set; }
        public bool IsTractionCapable { get; protected set; }
        public bool IsShearCapable { get; protected set; }
        public bool IsBendingCapable { get; protected set; }
        public bool IsTorsionCapable { get; protected set; }

        protected Element(ElementStructure structure, int dimension)
        {
            Dimension = dimension;
            Structure = structure;

            TranslationalDOF = 0;
            RotationalDOF = 0;

            IsCompressionCapable = false;
            IsTractionCapable = false;
            IsShearCapable = false;
            IsBendingCapable = false;
            IsTorsionCapable = false;
        }
    }
    public abstract class Beam : Element
    {
        // EVENT : TopologyChanged
        public event Action<int[]> TopologyChanged;
        protected virtual void OnTopologyChanged(int[] indexMap)
        {
            Action<int[]> handler = TopologyChanged;
            if (handler != null)
            {
                handler(indexMap);
            }
        }

        // EVENT : FramesTranslated
        public event Action<MVector[]> FramesTranslated;
        protected virtual void OnFramesTranslated(MVector[] dx)
        {
            Action<MVector[]> handler = FramesTranslated;
            if (handler != null)
            {
                handler(dx);
            }
        }

        // EVENT : FramesRotated
        public event Action<double[]> FramesRotated;
        protected virtual void OnFramesRotated(double[] dθ)
        {
            Action<double[]> handler = FramesRotated;
            if (handler != null)
            {
                handler(dθ);
            }
        }

        #region PRIVATE FIELDS

        // TOPOLOGY
        protected int ne;   // total number of edges
        protected int nv;   // total number of vertices
        protected int nv_h;  // number of handle vertices
        protected int nv_g;  // number of ghost vertices

        // SECTION AND MATERIAL
        protected Section[] sections;
        protected Material[] materials;
        public double[] ES;
        public double[] EI1;
        public double[] EI2;
        public double[] GJ;

        // GEOMETRIC CONFIGURATION
        protected MFrame[] mframes_0;
        protected MFrame[] mframes_i;
        protected MFrame[] mframes;

        // LUMPED MASS
        protected double[] lm_x;
        protected double[] lm_θ;

        // LUMPED DAMPING FACTOR
        protected double[] ld_x;
        protected double[] ld_θ;

        // VELOCITY
        protected MVector[] v_x;
        protected MVector[] v_θ;

        // ACCELERTION
        protected MVector[] a_x;
        protected MVector[] a_θ;

        public double[] l;            // edge length : l[i] = |e[i]|
        public MVector[] e;                // edge : e[i] = x[i+1] - x[i]

        // NODAL RESULTANT
        protected MVector[] R_x, Rint_x;
        protected double[] R_θ, Rint_θ;

        // EXTERNAL FORCES & MOMENTS
        public MVector[] Fext_g;
        public MVector[] Mext_m;
        public MVector[] fext_g;
        public MVector[] mext_m;

        // REACTION MOMENT AND FORCES
        public MVector[] Fr_g;
        public MVector[] Mr_m;

        //LOAD MANAGER
        protected BeamLoadManager loadManager;

        #endregion

        protected Beam(bool isClosed = false)
            : base(ElementStructure.Beam, 1)
        {
            IsClosed = isClosed;
            var options = new ParallelOptions();
        }

        // METHODS
        public int BoundaryToVertexIndex(Boundary boundary)
        {
            if (boundary == Boundary.Start)
            {
                return 0;
            }
            else
            {
                if (IsClosed)
                    return 0;
                else
                    return Nvh - 1;
            }
        }
        public int BoundaryToEdgeIndex(Boundary boundary)
        {
            if (boundary == Boundary.Start)
                return 0;
            else
                return Nvg - 1;
        }

        public int HandleToGlobalVertexIndex(int handleVertexIndex)
        {
            return 2 * handleVertexIndex;
        }

        public bool IsEdgeIndexValid(int edgeIndex, bool throwError = false)
        {
            if (edgeIndex < 0 || edgeIndex >= Nvg)
            {
                if (throwError)
                    throw new ArgumentOutOfRangeException("edgeIndex", "The edge index must be set in [0,Ne-1]");
                return false;
            }
            return true;
        }
        public bool IsVertexIndexValid(int vertexIndex, bool throwError = false)
        {
            if (vertexIndex < 0 || vertexIndex >= Nvh)
            {
                if (throwError)
                    throw new ArgumentOutOfRangeException("vertexIndex", "The vertex index must be set in [0,Nv-1]");
                return false;
            }
            return true;
        }

        #region POINTERS
        // GEOMETRIC CONFIGURATION
        internal MFrame[] RestConfiguration { get { return mframes_0; } }
        internal MFrame[] InitialConfiguration { get { return mframes_i; } }
        public MFrame[] ActualConfiguration { get { return mframes; } }

        // ACTUAL LUMPED MASS
        internal double[] Mx { get { return lm_x; } }
        internal double[] Mθ { get { return lm_θ; } }

        // ACTUAL LUMPED DAMPING FACTOR
        internal double[] Dx { get { return ld_x; } }
        internal double[] Dθ { get { return ld_θ; } }

        // ACTUAL VELOCITY
        internal MVector[] Vx { get { return v_x; } }
        internal MVector[] Vθ { get { return v_θ; } }

        // ACTUAL ACCELERATION
        internal MVector[] Ax { get { return a_x; } }
        internal MVector[] Aθ { get { return a_θ; } }

        internal MVector[] Rx_int { get { return this.Rint_x; } }
        internal double[] Rθ_int { get { return this.Rint_θ; } }
        internal MVector[] Rx { get { return this.R_x; } }
        internal double[] Rθ { get { return this.R_θ; } }


        #endregion

        #region PUBLIC INTERFACE

        // TOPOLOGY
        public int Ne { get { return ne; } }
        public int Nv { get { return nv; } }
        public int Nvh { get { return nv_h; } }
        public int Nvg { get { return nv_g; } }
        public bool IsClosed { get; private set; }
        public bool HasGhostVertices { get { return nv_g != 0; } }

        public virtual void Load(IEnumerable<BeamVectorLoad> loads)
        {
            loadManager.Fill(loads);
        }

        public abstract void Move(MVector[] dx);
        public abstract void Move(double[] dθ);

        public abstract void UpdateCenterlineProperties();
        public abstract void UpdateCurvatureBinormal();
        public abstract void UpdateMaterialFrame();

        // MOMENTS
        public abstract void UpdateBendingMoment();
        public abstract void UpdateTwistingMoment();
        public abstract void UpdateInternalNodalMoment();
        public abstract void UpdateResultantNodalMoment();

        // FORCES
        public abstract void UpdateAxialForce();
        public abstract void UpdateShearForce();
        public abstract void UpdateInternalNodalForce();
        public abstract void UpdateResultantNodalForce();

        // LUMPED MASSES
        public abstract void Update_lm_x(ref double[] lm_x);
        public abstract void Update_lm_θ(ref double[] lm_θ);
        #endregion
    }

}
