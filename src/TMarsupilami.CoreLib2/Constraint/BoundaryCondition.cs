
using System;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
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
    
    public abstract class BoundaryCondition : IJoint, IDRConstraint
    {
        #region PROPERTIES
        public MVector F { get; protected set; }
        public MVector M { get; protected set; }
        public IDRElement Element { get; protected set; }
        public BoundaryConditionType Type { get; protected set; }
        public Boundary Boundary { get; protected set; }
        public int VertexIndex { get; protected set; }

        #endregion

        #region CONSTRUCTORS
        protected BoundaryCondition(IDRElement element, BoundaryConditionType type, Boundary boundary) : base()
        {
            Element = element;
            Type = type;
            Boundary = boundary;
            if (Boundary == Boundary.Start) { VertexIndex = 0; }
            else { VertexIndex = element.Nv - 1; }
        }
        #endregion

        public static BoundaryCondition AddPinnedBoundaryCondition(IDRElement element, Boundary boundary)
        {
            return new Pinned(element, boundary);
        }
        public static BoundaryCondition AddClampedBoundaryCondition(IDRElement element, Boundary boundary)
        {
            return new Clamped(element, boundary);
        }

        public virtual void Init() { }
        public virtual void Enforce_Mr() { }
        public virtual void Enforce_Qr() { }
        public virtual void Enforce_Fr() { }

        // internal class
        private class Pinned : BoundaryCondition
        {
            public Pinned(IDRElement element, Boundary boundary)
                : base(element, BoundaryConditionType.Pinned, boundary)
            {}

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : pinned";
            }

            public override void Enforce_Fr()
            {
                MVector Fr = Element.Rx_int[VertexIndex] + Element.Fext_g[VertexIndex];
                Element.Fr_g[VertexIndex] = Fr;
            }
        }
        private class Clamped : BoundaryCondition
        { 
            private MFrame clamped_frame;    // clamped configuration storage

            public Clamped(IDRElement element, Boundary boundary)
                : base(element, BoundaryConditionType.Clamped, boundary)
            {
            }

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : clamped";
            }

            public override void Init()
            {
                clamped_frame = Element.MaterialFrames[VertexIndex];
            }
            public override void Enforce_Mr()
            {
                //element.t[VertexIndex] = clamped_frame.ZAxis;
                MVector κb;
                double κ1, κ2;
                double M1, M2;

                
                if (Boundary == Boundary.Start)
                {
                    // beam curvature regarding clamped bondary condition
                    κb = 2 / (Element.l[0] * Element.l[0]) * MVector.CrossProduct(clamped_frame.ZAxis, Element.e[0]);
                    
                    // bending moment due to the clamped boundary
                    κ1 = κb * Element.MaterialFrames[VertexIndex].XAxis;
                    κ2 = κb * Element.MaterialFrames[VertexIndex].YAxis;
                    M1 = κ1 * Element.EI1[0];
                    M2 = κ2 * Element.EI2[0];

                    Element.Mr_m[VertexIndex].X = M1;
                    Element.Mr_m[VertexIndex].Y = M2;
                }
                else if (Boundary == Boundary.End)
                {
                    // beam curvature regarding clamped bondary condition
                    int n = Element.Ne -1;
                    κb = 2 / (Element.l[n] * Element.l[n]) * MVector.CrossProduct(Element.e[n], clamped_frame.ZAxis);
                    
                    // bending moment due to the clamped boundary
                    κ1 = κb * Element.MaterialFrames[VertexIndex].XAxis;
                    κ2 = κb * Element.MaterialFrames[VertexIndex].YAxis;
                    M1 = κ1 * Element.EI1[n];
                    M2 = κ2 * Element.EI2[n];

                    Element.Mr_m[VertexIndex].X = -M1;
                    Element.Mr_m[VertexIndex].Y = -M2;
                }
                else
                {
                    // a écrire
                }
            }
            public override void Enforce_Qr()
            {
                double Qr = Element.Rθ_int[VertexIndex];
                Element.Mr_m[VertexIndex].Z = Qr;
            }
            public override void Enforce_Fr()
            {
                MVector Fr = Element.Rx_int[VertexIndex] + Element.Fext_g[VertexIndex];
                Element.Fr_g[VertexIndex] = Fr;
            }
        }
    }

    
}
