
using System;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum ConstraintLayoutType
    {
        BoundaryCondition = 000,
    }
    public enum BoundaryConditionType
    {
        Free,
        Pinned,     // fixed (x,y,z) & free (xx,yy,zz) + eventual end moments
        Clamped,    // fully fixed (x,y,z) & (xx,yy,zz)
    }

    public abstract class BoundaryCondition
    {
        #region PROPERTIES
        public ConstraintLayoutType Type { get { return ConstraintLayoutType.BoundaryCondition; } }
        public BoundaryConditionType SubType { get; protected set; }
        public MVector F { get; protected set; }
        public MVector M { get; protected set; }
        public Beam Beam { get; protected set; }
        public Boundary Boundary { get; protected set; }
        public int VertexIndex { get; protected set; }

        #endregion

        #region CONSTRUCTORS
        protected BoundaryCondition(Beam beam, BoundaryConditionType subType, Boundary boundary) : base()
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
                : base(beam, BoundaryConditionType.Pinned, boundary)
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
                : base(beam, BoundaryConditionType.Clamped, boundary)
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

    public abstract class BC
    {
        #region PROPERTIES
        public BoundaryConditionType Type { get; protected set; }
        public MVector F { get; protected set; }
        public MVector M { get; protected set; }
        public Beam Beam { get; protected set; }
        public Boundary Boundary { get; protected set; }
        public int VertexIndex { get; protected set; }

        #endregion

        #region CONSTRUCTORS
        protected BC(Beam beam, BoundaryConditionType type, Boundary boundary) : base()
        {
            Beam = beam;
            Type = type;
            Boundary = boundary;
            VertexIndex = beam.BoundaryToVertexIndex(Boundary);
        }
        #endregion

        public static BC AddPinnedBoundaryCondition(Beam beam, Boundary boundary)
        {
            return new Pinned(beam, boundary);
        }
        public static BC AddClampedBoundaryCondition(Beam beam, Boundary boundary)
        {
            return new Clamped(beam, boundary);
        }

        public virtual void Init() { }
        public virtual void Enforce_t(MVector[] t) { }
        public virtual void Enforce_Mr(MVector[] Mr, MVector[] Rθ) { }
        public virtual void Enforce_Fr(MVector[] Fr, MVector[] Rx) { }

        // internal class
        private class Pinned : BC
        {
            public Pinned(Beam beam, Boundary boundary)
                : base(beam, BoundaryConditionType.Pinned, boundary)
            {
                beam.ReactionForceUpdating += Enforce_Fr;
            }

            public override string ToString()
            {
                return "[BOUNDARY CONDITION] : pinned";
            }

            public override void Enforce_Fr(MVector[] Fr, MVector[] Rx)
            {
                if (Boundary == Boundary.Start)
                {
                    this.F = Rx[0];  // force appliquée par la poutre sur le support
                    Fr[0] = this.F;
                }
                else
                {
                    this.F = Rx[Beam.Nv - 1];  // force appliquée par la poutre sur le support
                    Fr[Beam.Nvh - 1] = this.F;
                }

            }
        }
        private class Clamped : BC
        {
            private MFrame clamped_frame;    // clamped configuration storage

            public Clamped(Beam beam, Boundary boundary)
                : base(beam, BoundaryConditionType.Clamped, boundary)
            {
                Init();
                beam.TangentsUpdated += Enforce_t;
                beam.ReactionForceUpdating += Enforce_Fr;
                beam.ReactionMomentUpdating += Enforce_Mr;
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
            public override void Enforce_t(MVector[] t)
            {
                if (Boundary == Boundary.Start)
                {
                    t[0] = clamped_frame.ZAxis;
                }
                else if (Boundary == Boundary.End)
                {
                    t[Beam.Nv - 1] = clamped_frame.ZAxis;
                }
                else
                {
                }
            }
            public override void Enforce_Mr(MVector[] Mr, MVector[] Rθ)
            {
                if (Boundary == Boundary.Start)
                {
                    this.M = Rθ[0];  // force appliquée par la poutre sur le support
                    Mr[0] = this.M; 
                }
                else
                {
                    this.M = Rθ[Beam.Nv - 1];  // force appliquée par la poutre sur le support
                    Mr[Beam.Nvh - 1] = this.M; 
                }
            }
            public override void Enforce_Fr(MVector[] Fr, MVector[] Rx)
            {
                if (Boundary == Boundary.Start)
                {
                    this.F = Rx[0];  // force appliquée par la poutre sur le support
                    Fr[0] = this.F;
                }
                else
                {
                    this.F = Rx[Beam.Nv - 1];  // force appliquée par la poutre sur le support
                    Fr[Beam.Nvh - 1] = this.F;
                }
            }
        }
    }
}
