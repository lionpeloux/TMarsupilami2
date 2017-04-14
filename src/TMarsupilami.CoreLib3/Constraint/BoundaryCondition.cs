
using System;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum ConstraintLayoutType
    {
        BoundaryCondition = 000,
    }
    public enum SupportCondition
    {
        Free,
        Pinned,     // fixed (x,y,z) & free (xx,yy,zz) + eventual end moments
        Clamped,    // fully fixed (x,y,z) & (xx,yy,zz)
    }

    public abstract class BoundaryCondition
    {
        #region PROPERTIES
        public ConstraintLayoutType Type { get { return ConstraintLayoutType.BoundaryCondition; } }
        public SupportCondition SubType { get; protected set; }
        public MVector F { get; protected set; }
        public MVector M { get; protected set; }
        public Beam Beam { get; protected set; }
        public Boundary Boundary { get; protected set; }
        public int VertexIndex { get; protected set; }

        #endregion

        #region CONSTRUCTORS
        protected BoundaryCondition(Beam beam, SupportCondition subType, Boundary boundary) : base()
        {
            Beam = beam;
            SubType = subType;
            Boundary = boundary;
            VertexIndex = beam.BoundaryToVertexIndex(Boundary);
        }
        #endregion

        public static BoundaryCondition AddPinnedBoundaryCondition(Beam beam, Boundary boundary)
        {
            return new Pinned(beam, boundary);
        }
        public static BoundaryCondition AddClampedBoundaryCondition(Beam beam, Boundary boundary)
        {
            return new Clamped(beam, boundary);
        }

        public virtual void Init() { }
        public virtual void Enforce_Mr() { }
        public virtual void Enforce_Qr() { }
        public virtual void Enforce_Fr() { }

        // internal class
        private class Pinned : BoundaryCondition
        {
            public Pinned(Beam beam, Boundary boundary)
                : base(beam, SupportCondition.Pinned, boundary)
            {}

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : pinned";
            }

            public override void Enforce_Fr()
            {
                MVector Fr;

                if (Boundary == Boundary.Start)
                {
                    Fr = Beam.Rx_int[0] + Beam.Fext_g[0] + 0.5 * Beam.fext_g[0] * Beam.l[0];
                    Beam.Fr_g[0] = Fr;
                }
                else
                {
                    Fr = Beam.Rx_int[Beam.Nv - 1] + Beam.Fext_g[Beam.Nvh - 1] + 0.5 * Beam.fext_g[Beam.Nvg - 1] * Beam.l[2 * Beam.Nvg - 1];
                    Beam.Fr_g[VertexIndex] = Fr;
                }

            }
        }
        private class Clamped : BoundaryCondition
        { 
            private MFrame clamped_frame;    // clamped configuration storage

            public Clamped(Beam beam, Boundary boundary)
                : base(beam, SupportCondition.Clamped, boundary)
            {
            }

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : clamped";
            }

            public override void Init()
            {
                if (Boundary == Boundary.Start)
                {
                    clamped_frame = Beam.ActualConfiguration[0];
                }
                else
                {
                    clamped_frame = Beam.ActualConfiguration[Beam.Nv - 1];
                }
            }
            public override void Enforce_Mr()
            {
                //beam.t[VertexIndex] = clamped_frame.ZAxis;
                MVector κb;
                double κ1, κ2;
                double M1, M2;

                
                if (Boundary == Boundary.Start)
                {
                    // beam curvature regarding clamped bondary condition
                    κb = 2 / (Beam.l[0] * Beam.l[0]) * MVector.CrossProduct(clamped_frame.ZAxis, Beam.e[0]);
                    
                    // bending moment due to the clamped boundary
                    κ1 = κb * Beam.ActualConfiguration[0].XAxis;
                    κ2 = κb * Beam.ActualConfiguration[0].YAxis;
                    M1 = κ1 * Beam.EI1[0];
                    M2 = κ2 * Beam.EI2[0];

                    Beam.Mr_m[0].X = M1;
                    Beam.Mr_m[0].Y = M2;
                }
                else if (Boundary == Boundary.End)
                {
                    // beam curvature regarding clamped bondary condition
                    κb = 2 / (Beam.l[Beam.Ne - 1] * Beam.l[Beam.Ne - 1]) * MVector.CrossProduct(Beam.e[Beam.Ne - 1], clamped_frame.ZAxis);

                    // bending moment due to the clamped boundary
                    κ1 = κb * Beam.ActualConfiguration[Beam.Nv - 1].XAxis;
                    κ2 = κb * Beam.ActualConfiguration[Beam.Nv - 1].YAxis;
                    M1 = κ1 * Beam.EI1[Beam.Nvg - 1];
                    M2 = κ2 * Beam.EI2[Beam.Nvg - 1];

                    Beam.Mr_m[Beam.Nvh - 1].X = -M1;
                    Beam.Mr_m[Beam.Nvh - 1].Y = -M2;
                }
                else
                {
                    // a écrire
                }
            }
            public override void Enforce_Qr()
            {
                double Qr;

                if (Boundary == Boundary.Start)
                {
                    Qr = Beam.Rθ_int[0].Z;
                    Beam.Mr_m[0].Z = Qr;
                }
                else
                {
                    Qr = Beam.Rθ_int[Beam.Nv - 1].Z;
                    Beam.Mr_m[Beam.Nvh - 1].Z = Qr;
                }              
            }
            public override void Enforce_Fr()
            {
                MVector Fr;

                if (Boundary == Boundary.Start)
                {
                    Fr = Beam.Rx_int[0] + Beam.Fext_g[0] + 0.5 * Beam.fext_g[0] * Beam.l[0];
                    Beam.Fr_g[0] = Fr;
                }
                else
                {
                    Fr = Beam.Rx_int[Beam.Nv - 1] + Beam.Fext_g[Beam.Nvh - 1] + 0.5 * Beam.fext_g[Beam.Nvg - 1] * Beam.l[2 * Beam.Nvg - 1];
                    Beam.Fr_g[VertexIndex] = Fr;
                }
                
            }
        }
    }

    public abstract class Support
    {
        protected int nv, nv_h;
        protected Torsor torsor;
        protected MFrame frame;

        #region PROPERTIES
        public Beam Beam { get; protected set; }
        public int VertexIndex { get { return nv_h; }}
        public int GlobalVertexIndex { get { return nv; } }
        public SupportCondition Type { get; protected set; }

        /// <summary>
        /// The local frame of the support.
        /// </summary>
        public MFrame Frame { get { return frame; } }

        /// <summary>
        /// Support reaction force (Fr) towards the element.
        /// </summary>
        public MVector F { get; protected set; }

        /// <summary>
        /// Support reaction moment (Mr) towards the element. 
        /// </summary>
        public MVector M { get; protected set; }

        /// <summary>
        /// Gets the support reaction torsor (Tr) towards the element at the local frame.
        /// </summary>
        public Torsor Torsor
        {
            get
            {
                return GetTorsor();
            }
        }

        #endregion

        #region CONSTRUCTORS
        protected Support(Beam beam, int vertexIndex, SupportCondition type)
        {
            Beam = beam;
            Type = type;
            nv_h = vertexIndex;
            nv = Beam.HandleToGlobalVertexIndex(VertexIndex);
            frame = beam.ActualConfiguration[GlobalVertexIndex];
        }
        protected Support(Beam beam, int vertexIndex, SupportCondition type, MFrame supportFrame):this(beam, vertexIndex, type)
        {
            Beam = beam;
            Type = type;
            nv_h = vertexIndex;
            nv = Beam.HandleToGlobalVertexIndex(VertexIndex);
            frame = supportFrame;
        }
        protected Support(Beam beam, Boundary boundary, SupportCondition type)
        {
            Beam = beam;
            Type = type;
            nv_h = beam.BoundaryToVertexIndex(boundary);
            nv = Beam.HandleToGlobalVertexIndex(VertexIndex);
            frame = beam.ActualConfiguration[GlobalVertexIndex];
        }
        protected Support(Beam beam, Boundary boundary, SupportCondition type, MFrame supportFrame) : this(beam, boundary, type)
        {
            Beam = beam;
            Type = type;
            nv_h = beam.BoundaryToVertexIndex(boundary);
            nv = Beam.HandleToGlobalVertexIndex(VertexIndex);
            frame = supportFrame;
        }
        #endregion

        // FACTORY
        public static Support AddPinnedSupport(Beam beam, int vertexIndex)
        {
            return new Pinned(beam, vertexIndex);
        }
        public static Support AddPinnedSupport(Beam beam, Boundary boundary)
        {
            return new Pinned(beam, boundary);
        }
        public static Support AddPinnedSupport(Beam beam, int vertexIndex, MFrame supportFrame)
        {
            return new Pinned(beam, vertexIndex, supportFrame);
        }
        public static Support AddPinnedSupport(Beam beam, Boundary boundary, MFrame supportFrame)
        {
            return new Pinned(beam, boundary, supportFrame);
        }

        public static Support AddClampedSupport(Beam beam, int vertexIndex)
        {
            return new Clamped(beam, vertexIndex);
        }
        public static Support AddClampedSupport(Beam beam, Boundary boundary)
        {
            return new Clamped(beam, boundary);
        }
        public static Support AddClampedSupport(Beam beam, int vertexIndex, MFrame supportFrame)
        {
            return new Clamped(beam, vertexIndex, supportFrame);
        }
        public static Support AddClampedSupport(Beam beam, Boundary boundary, MFrame supportFrame)
        {
            return new Clamped(beam, boundary, supportFrame);
        }

        private Torsor GetTorsor()
        {
            var torsor = new Torsor(F, M, Beam.ActualConfiguration[nv].Origin);
            return torsor.Move(Frame);
        }

        public virtual void Init() { }
        public virtual void Enforce_t(MVector[] t) { }
        public virtual void Enforce_Mr(MVector[] Mr, MVector[] Rθ) { }
        public virtual void Enforce_Fr(MVector[] Fr, MVector[] Rx) { }

        // internal class
        private class Pinned : Support
        {

            private void Subscribe()
            {
                Init();
                Beam.ReactionForceUpdating += Enforce_Fr;
            }
            public Pinned(Beam beam, int vertexIndex) : base(beam, vertexIndex, SupportCondition.Pinned)
            {
                Subscribe();
            }
            public Pinned(Beam beam, int vertexIndex, MFrame supportFrame) : base(beam, vertexIndex, SupportCondition.Pinned, supportFrame)
            {
                Subscribe();
            }
            public Pinned(Beam beam, Boundary boundary) : base(beam, boundary, SupportCondition.Pinned)
            {
                Subscribe();
            }
            public Pinned(Beam beam, Boundary boundary, MFrame supportFrame) : base(beam, boundary, SupportCondition.Pinned, supportFrame)
            {
                Subscribe();
            }

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : pinned";
            }
            public override void Enforce_Fr(MVector[] Fr, MVector[] Rx)
            {
                F = -Rx[nv];    // réaction du support sur la poutre
                Rx[nv] += F;    // => Rx[nv] = 0
                Fr[nv_h] = F;
            }
        }
        private class Clamped : Support
        {
            private MFrame clamped_frame;    // clamped configuration storage

            private void Subscribe()
            {
                Init();
                Beam.TangentsUpdated += Enforce_t;
                Beam.ReactionForceUpdating += Enforce_Fr;
                Beam.ReactionMomentUpdating += Enforce_Mr;
            }
            public Clamped(Beam beam, int vertexIndex) : base(beam, vertexIndex, SupportCondition.Clamped)
            {
                Subscribe();
            }
            public Clamped(Beam beam, int vertexIndex, MFrame supportFrame) : base(beam, vertexIndex, SupportCondition.Clamped, supportFrame)
            {
                Subscribe();
            }
            public Clamped(Beam beam, Boundary boundary) : base(beam, boundary, SupportCondition.Clamped)
            {
                Subscribe();
            }
            public Clamped(Beam beam, Boundary boundary, MFrame supportFrame) : base(beam, boundary, SupportCondition.Clamped, supportFrame)
            {
                Subscribe();
            }

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : clamped";
            }

            public override void Init()
            {
                clamped_frame = Beam.ActualConfiguration[nv];
            }
            public override void Enforce_t(MVector[] t)
            {
                t[nv] = clamped_frame.ZAxis;
            }
            public override void Enforce_Mr(MVector[] Mr, MVector[] Rθ)
            {
                M = -Rθ[nv];    // réaction du support sur la poutre
                Rθ[nv] += M;    // => Rθ[nv] = 0
                Mr[nv_h] = M;
            }
            public override void Enforce_Fr(MVector[] Fr, MVector[] Rx)
            {
                F = -Rx[nv];    // réaction du support sur la poutre
                Rx[nv] += F;    // => Rx[nv] = 0
                Fr[nv_h] = F;
            }
        }
    }
}
