
using TMarsupilami.MathLib;

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    public enum BoundaryConditionType
    {
        Free,       // fully free end (x,y,z) & (xx,yy,zz) + eventual end moments
        Pinned,     // fixed (x,y,z) & (xx,yy,zz) + eventual end moments
        Clamped,    // fully fixed (x,y,z) & (xx,yy,zz)
        Elastic,    // end force and moment relative to a fixed position 
    }

    public enum Boundary
    {
        Start,
        End,
    }
    
    public abstract class BoundaryCondition : Constraint
    {
        #region FIELD
        private BoundaryConditionType _type;    // BC TYPE
        protected Boundary _boundary;       // is it start or end of element ? what about loop elements ??
        protected CurvedBeam element;           // holds a reference to the concerned element
        public int nj;                       // the concerned node
        protected MVector F;   // support force on the beam boundary
        protected MVector M;   // support moment on the beam boundary
        #endregion

        #region PROPERTIES

        /// <summary>
        /// Constraint type.
        /// </summary>
        public BoundaryConditionType Type
        {
            get { return _type; }
            protected set { _type = value; }
        }

        /// <summary>
        /// Boundary.
        /// </summary>
        public Boundary Boundary
        {
            get { return _boundary; }
            protected set { _boundary = value; }
        }

        #endregion

        #region CONSTRUCTORS
        protected BoundaryCondition(ref CurvedBeam element, BoundaryConditionType type, Boundary boundary) : base()
        {
            ConstraintType = ConstraintType.BoundaryCondition;
            this.element = element;
            this.Type = type;
            this.Boundary = boundary;
            if (Boundary == Boundary.Start) { nj = 0; }
            else { nj = element.Nn - 1; }
        }
        #endregion

        public static Constraint AddPinnedBoundaryCondition(ref CurvedBeam element, Boundary boundary)
        {
            return new Pinned(ref element, boundary);
        }
        public static Constraint AddClampedBoundaryCondition(ref CurvedBeam element, Boundary boundary)
        {
            return new Clamped(ref element, boundary);
        }
        
        // internal class
        private class Pinned : BoundaryCondition
        {
            public Pinned(ref CurvedBeam element, Boundary boundary)
                : base(ref element, BoundaryConditionType.Pinned, boundary)
            {}

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : pinned";
            }

            public override void Enforce_Fr()
            {
                MVector Fr = element.Rx_int[nj] + element.Fext[nj];
                element.Fr[nj] = Fr;
            }
        }
        private class Clamped : BoundaryCondition
        { 
            private MFrame clamped_frame;    // clamped configuration storage

            public Clamped(ref CurvedBeam element, Boundary boundary)
                : base(ref element, BoundaryConditionType.Clamped, boundary)
            {
            }

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : clamped";
            }

            public override void Init()
            {
                clamped_frame = element.MaterialFrame[nj];
            }
            public override void Enforce_Mr()
            {
                //element.t[nj] = clamped_frame.ZAxis;
                MVector κb;
                double κ1, κ2;
                double M1, M2;

                
                if (Boundary == Boundary.Start)
                {
                    // beam curvature regarding clamped bondary condition
                    κb = 2 / (element.l[0] * element.l[0]) * MVector.CrossProduct(clamped_frame.ZAxis, element.e[0]);
                    
                    // bending moment due to the clamped boundary
                    κ1 = κb * element.MaterialFrame[nj].XAxis;
                    κ2 = κb * element.MaterialFrame[nj].YAxis;
                    M1 = κ1 * element.EI1[0];
                    M2 = κ2 * element.EI2[0];

                    element.Mr[nj].X = M1;
                    element.Mr[nj].Y = M2;
                }
                else if (Boundary == Boundary.End)
                {
                    // beam curvature regarding clamped bondary condition
                    int n = element.Ne -1;
                    κb = 2 / (element.l[n] * element.l[n]) * MVector.CrossProduct(element.e[n], clamped_frame.ZAxis);
                    
                    // bending moment due to the clamped boundary
                    κ1 = κb * element.MaterialFrame[nj].XAxis;
                    κ2 = κb * element.MaterialFrame[nj].YAxis;
                    M1 = κ1 * element.EI1[n];
                    M2 = κ2 * element.EI2[n];

                    element.Mr[nj].X = -M1;
                    element.Mr[nj].Y = -M2;
                }
                else
                {

                }
            }
            public override void Enforce_Qr()
            {
                double Qr = element.Rθ_int[nj];
                element.Mr[nj].Z = Qr;
            }
            public override void Enforce_Fr()
            {
                MVector Fr = element.Rx_int[nj] + element.Fext[nj];
                element.Fr[nj] = Fr;
            }
        }
    }

    
}
