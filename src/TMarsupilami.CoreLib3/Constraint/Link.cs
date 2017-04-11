using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public abstract class Link
    {
        public abstract bool IsTorsionCapable { get; }

        public virtual void Init() { }
        public virtual void Update_x() { }
        public virtual void Update_θ() { }
        public virtual void Enforce_x() { }
        public virtual void Enforce_t() { }
        public virtual void Enforce_frames() { }
        public virtual void Transfer_Rx() { }
        public virtual void Transfer_Rθ() { }
        public virtual void Transfer_Mx() { }
        public virtual void Transfer_Mθ() { }

        public static Link CreateClampedLink(Beam b1, int n1, Beam b2, int n2)
        {
            return new ClampedLink(b1, n1, b2, n2);
        }
        public static Link CreateElasticPinnedLink(Beam b1, int n1, Beam b2, int n2, double K)
        {
            return new ElasticPinnedLink(b1, n1, b2, n2, K);
        }
        public static Link CreateElasticSwivelLink(Beam b1, int n1, Beam b2, int n2, double K, double C)
        {
            return new ElasticSwivelLink(b1, n1, b2, n2, K, C);
        }

    }

    public class ElasticPinnedLink : Link
    {
        private Beam b1, b2;
        private int n1, n2;

        public double lm_x;
        public double K;
        public MVector u, F12, F21;

        public override bool IsTorsionCapable { get { return false; } }

        public ElasticPinnedLink(Beam b1, int n1, Beam b2, int n2, double K)
        {
            this.b1 = b1;
            this.b2 = b2;
            this.n1 = b1.HandleToGlobalVertexIndex(n1);
            this.n2 = b2.HandleToGlobalVertexIndex(n2);
            this.K = K;
        }


        public override void Update_x()
        {
            var u = new MVector(b1.x[n1], b2.x[n2]);

            if (u.LengthSquared() == 0)
            {
                F12 = MVector.Zero;
                F21 = MVector.Zero;
            }
            else
            {
                F12 = -K * u;
                F21 = -F12;
            }
        }
        public override void Transfer_Rx()
        {
            b1.Fr_g[n1/2] = -F21;
            b2.Fr_g[n2/2] = -F12;
        }
        public override void Transfer_Mx()
        {
            lm_x = K;
            lm_x += b1.lm_x[n1];
            lm_x += b2.lm_x[n2];

            b1.lm_x[n1] = lm_x;
            b2.lm_x[n2] = lm_x;
        }
    }

    public class ElasticSwivelLink : Link
    {
        private Beam b1, b2;
        private int n1, n1_h, n2, n2_h;

        public double C, K;
        public double lm_x, lm_θ;
        public MVector M12, M21;
        public MVector u, F12, F21;

        public override bool IsTorsionCapable { get { return true; } }

        public ElasticSwivelLink(Beam b1, int n1, Beam b2, int n2, double K, double C)
        {
            this.b1 = b1;
            this.b2 = b2;
            this.n1_h = n1;
            this.n2_h = n2;
            this.n1 = b1.HandleToGlobalVertexIndex(n1_h);
            this.n2 = b2.HandleToGlobalVertexIndex(n2_h);
            this.K = K;
            this.C = C;
        }

        public override void Update_x()
        {
            var u = new MVector(b1.x[n1], b2.x[n2]);

            if (u.LengthSquared() == 0)
            {
                F12 = MVector.Zero;
                F21 = MVector.Zero;
            }
            else
            {
                F12 = -K * u;
                F21 = -F12;
            }
        }
        public override void Update_θ()
        {
            var v1 = b1.mframes[n1].XAxis;
            var v2 = b2.mframes[n2].XAxis;
            M21 = C * MVector.CrossProduct(v1, v2);
            M12 = -M21;
        }

        public override void Transfer_Rx()
        {
            b1.Fr_g[n1 / 2] = -F21;
            b2.Fr_g[n2 / 2] = -F12;
        }
        public override void Transfer_Rθ()
        {
            b1.Mr_m[n1_h] = b1.ToMaterialCoordinateSystem(-M21, n1_h);
            b2.Mr_m[n2_h] = b1.ToMaterialCoordinateSystem(-M12, n2_h);
        }

        public override void Transfer_Mx()
        {
            lm_x = K;
            lm_x += b1.lm_x[n1];
            lm_x += b2.lm_x[n2];

            b1.lm_x[n1] = lm_x;
            b2.lm_x[n2] = lm_x;
        }
        public override void Transfer_Mθ()
        {
            lm_θ = C;
            lm_θ += b1.lm_θ[n1];
            lm_θ += b2.lm_θ[n2];

            b1.lm_θ[n1] = lm_θ;
            b2.lm_θ[n2] = lm_θ;
        }

    }

    public class ClampedLink : Link
    {
        private Beam b1, b2;
        private int n1, n2;

        private MFrame pf1, pf2, rf, rf0;
        private MVector u1, u2;
        private MVector xaxis_1, yaxis_1, xaxis_2, yaxis_2;

        public override bool IsTorsionCapable { get { return true; } }

        public ClampedLink(Beam b1, int n1, Beam b2, int n2)
        {
            this.b1 = b1;
            this.b2 = b2;
            this.n1 = b1.HandleToGlobalVertexIndex(n1);
            this.n2 = b2.HandleToGlobalVertexIndex(n2);
        }

        public override void Init()
        {
            var f1 = b1.RestConfiguration[n1];
            var f2 = b2.RestConfiguration[n2];

            var rf = GetMeanFrame(f1, f2);

            u1 = new MVector(rf.Origin, f1.Origin);
            xaxis_1 = new MVector(f1.XAxis * rf.XAxis, f1.XAxis * rf.YAxis, f1.XAxis * rf.ZAxis); // in rf
            yaxis_1 = new MVector(f1.YAxis * rf.XAxis, f1.YAxis * rf.YAxis, f1.YAxis * rf.ZAxis); // in rf
            u1 = new MVector(u1 * rf.XAxis, u1 * rf.YAxis, u1 * rf.ZAxis); // in rf

            u2 = new MVector(rf.Origin, f2.Origin);
            xaxis_2 = new MVector(f2.XAxis * rf.XAxis, f2.XAxis * rf.YAxis, f2.XAxis * rf.ZAxis); // in rf
            yaxis_2 = new MVector(f2.YAxis * rf.XAxis, f2.YAxis * rf.YAxis, f2.YAxis * rf.ZAxis); // in rf
            u2 = new MVector(u2 * rf.XAxis, u2 * rf.YAxis, u2 * rf.ZAxis); // in rf

            this.rf = rf;
            this.rf0 = rf;
            pf1 = f1;
            pf2 = f2;
        }
        public override void Update_x()
        {
            var x1 = b1.x[n1];
            var x2 = b2.x[n2];
            rf.Origin = 0.5 * (x1 + x2);

            pf1.Origin = rf.Origin + (u1.X * rf.XAxis + u1.Y * rf.YAxis + u1.Z * rf.ZAxis);
            pf2.Origin = rf.Origin + (u2.X * rf.XAxis + u2.Y * rf.YAxis + u2.Z * rf.ZAxis);
        }
        public override void Update_θ()
        {
            var f1 = b1.ActualConfiguration[n1];
            var f2 = b2.ActualConfiguration[n2];

            rf = GetMeanFrame(f1, f2);

            //MPoint o1 = rf.Origin + (u1.X * rf.XAxis + u1.Y * rf.YAxis + u1.Z * rf.ZAxis);
            MVector x1 = xaxis_1.X * rf.XAxis + xaxis_1.Y * rf.YAxis + xaxis_1.Z * rf.ZAxis;
            MVector y1 = yaxis_1.X * rf.XAxis + yaxis_1.Y * rf.YAxis + yaxis_1.Z * rf.ZAxis;
            pf1.XAxis = x1;
            pf1.YAxis = y1;
            pf1.ZAxis = MVector.CrossProduct(x1, y1);

            //MPoint o2 = rf.Origin + (u2.X * rf.XAxis + u2.Y * rf.YAxis + u2.Z * rf.ZAxis);
            MVector x2 = xaxis_2.X * rf.XAxis + xaxis_2.Y * rf.YAxis + xaxis_2.Z * rf.ZAxis;
            MVector y2 = yaxis_2.X * rf.XAxis + yaxis_2.Y * rf.YAxis + yaxis_2.Z * rf.ZAxis;
            pf2.XAxis = x2;
            pf2.YAxis = y2;
            pf2.ZAxis = MVector.CrossProduct(x2, y2);
        }
        public override void Enforce_x()
        {
            b1.x[n1] = pf1.Origin;
            b2.x[n2] = pf2.Origin;
        }
        public override void Enforce_t()
        {
            b1.t[n1] = pf1.ZAxis;
            b2.t[n2] = pf2.ZAxis;
        }
        public override void Enforce_frames()
        {
            b1.mframes[n1] = pf1;
            b2.mframes[n2] = pf2;
        }

        public void Transfert_Rx()
        {
            b1.x[n1] = pf1.Origin;
            b2.x[n2] = pf2.Origin;
        }


        private MFrame GetMeanFrame(MFrame f1, MFrame f2)
        {
            MVector d3, d1, d2;
            double l1, l2, l3;

            d3 = f1.ZAxis + f2.ZAxis;
            l3 = d3.LengthSquared();

            d1 = f1.XAxis + f2.XAxis;
            l1 = d1.LengthSquared();

            d2 = f1.YAxis + f2.YAxis;
            l2 = d2.LengthSquared();

            if (l3 != 0)
            {
                d3 = (1 / Math.Sqrt(l3)) * d3;
                if (l1 != 0) // l1 != 0
                {
                    d1 = d1 - (d1 * d3) * d3;
                    d1.Normalize();
                    d2 = MVector.CrossProduct(d3, d1);
                }
                else if (l2 != 0) // l2 != 0 && l1 = 0
                {
                    d2 = d2 - (d2 * d3) * d3;
                    d2.Normalize();
                    d1 = MVector.CrossProduct(d2, d3);
                }
                else // l1 = 0 && l2 = 0 : cas dégénéré
                {
                    d1 = f1.XAxis;
                    d2 = f2.YAxis;
                }
            }
            else
            {
                if (l1 != 0 && l2 != 0)
                {
                    d1 = (1 / Math.Sqrt(l1)) * d1;
                    d2 = d2 - (d2 * d1) * d1;
                    d2.Normalize();
                    d3 = MVector.CrossProduct(d1, d2);
                }
                else
                {
                    d1 = f1.XAxis;
                    d2 = f1.YAxis;
                    d3 = f1.ZAxis;
                }
            }

            rf.XAxis = d1;
            rf.YAxis = d2;
            rf.ZAxis = d3;
            //var rf = new MFrame(origin, d1, d2, d3, false);
            return rf;
        }

        public override void Transfer_Rx()
        {
        }
        public override void Transfer_Rθ()
        {
        }
        public override void Transfer_Mx()
        {
        }
        public override void Transfer_Mθ()
        {
        }
    }
}
