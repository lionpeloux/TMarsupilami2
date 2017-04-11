using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;
using TMarsupilami.Gh.Parameter;

namespace TMarsupilami.Gh.Component
{
    public class Comp_OrientLink : GH_Component
    {

        public Comp_OrientLink()
          : base("Orient a Link", "Orient",
              "Orients a link represented by two frames to a target reference frame.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.quarternary;
            }
        }
        //protected override System.Drawing.Bitmap Icon
        //{
        //    get
        //    {
        //        return Resources.ZRotatePlane;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{0DCD5170-0CB0-4427-A34A-F2ADBFD0C1DE}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "First Frame", "F1", "The first frame of the link.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MFrame(), "Second Frame", "F2", "The second frame of the link.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MFrame(), "Reference Frame", "F", "The target reference frame to orient the link to.", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "rF", "The initial reference frame of the link.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Frame", "F1", "The oriented first frame of the link.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Frame", "F2", "The oriented second frame of the link.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var f1 = new MFrame();
            var f2 = new MFrame();
            var f = new MFrame();

            if (!DA.GetData(0, ref f1)) { return; }
            if (!DA.GetData(1, ref f2)) { }
            if (!DA.GetData(2, ref f)) { return; }

            MVector d3, d1, d2;
            double l1, l2, l3;

            MFrame rf, of1, of2;
         
            var origin = 0.5 * (f1.Origin + f2.Origin);

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

            rf = new MFrame(origin, d1, d2, d3, false);

            var u1 = new MVector(rf.Origin, f1.Origin);
            var xaxis_1 = new MVector(f1.XAxis * rf.XAxis, f1.XAxis * rf.YAxis, f1.XAxis * rf.ZAxis); // in rf
            var yaxis_1 = new MVector(f1.YAxis * rf.XAxis, f1.YAxis * rf.YAxis, f1.YAxis * rf.ZAxis); // in rf
            u1 = new MVector(u1 * rf.XAxis, u1 * rf.YAxis, u1 * rf.ZAxis); // in rf

            var u2 = new MVector(rf.Origin, f2.Origin);
            var xaxis_2 = new MVector(f2.XAxis * rf.XAxis, f2.XAxis * rf.YAxis, f2.XAxis * rf.ZAxis); // in rf
            var yaxis_2 = new MVector(f2.YAxis * rf.XAxis, f2.YAxis * rf.YAxis, f2.YAxis * rf.ZAxis); // in rf
            u2 = new MVector(u2 * rf.XAxis, u2 * rf.YAxis, u2 * rf.ZAxis); // in rf


            MPoint o1 = f.Origin + (u1.X * f.XAxis + u1.Y * f.YAxis + u1.Z * f.ZAxis);
            MVector x1 = xaxis_1.X * f.XAxis + xaxis_1.Y * f.YAxis + xaxis_1.Z * f.ZAxis;
            MVector y1 = yaxis_1.X * f.XAxis + yaxis_1.Y * f.YAxis + yaxis_1.Z * f.ZAxis;
            of1 = new MFrame(o1, x1, y1);

            MPoint o2 = f.Origin + (u2.X * f.XAxis + u2.Y * f.YAxis + u2.Z * f.ZAxis);
            MVector x2 = xaxis_2.X * f.XAxis + xaxis_2.Y * f.YAxis + xaxis_2.Z * f.ZAxis;
            MVector y2 = yaxis_2.X * f.XAxis + yaxis_2.Y * f.YAxis + yaxis_2.Z * f.ZAxis;
            of2 = new MFrame(o2, x2, y2);

            DA.SetData(0, rf);
            DA.SetData(1, of1);
            DA.SetData(2, of2);

        }

        //protected override void SolveInstance(IGH_DataAccess DA)
        //{
        //    var f1 = new MFrame();
        //    var f2 = new MFrame();
        //    var f = new MFrame();

        //    if (!DA.GetData(0, ref f1)) { return; }
        //    if (!DA.GetData(1, ref f2)) { }
        //    if (!DA.GetData(2, ref f)) { return; }


        //    MVector a1, a2;
        //    MVector u1, u2;
        //    double sinα_1, cosα_1, sinθ_1, cosθ_1;
        //    double sinα_2, cosα_2, sinθ_2, cosθ_2;

        //    var origin = 0.5 * (f1.Origin + f2.Origin);

        //    var t = f1.ZAxis + f2.ZAxis;
        //    t.Normalize();

        //    var n = MVector.CrossProduct(f1.ZAxis, f2.ZAxis);
        //    n.Normalize();

        //    var rf = new MFrame(origin, n, MVector.CrossProduct(t, n), t, false);

        //    // On détermine les 2 rotations pour passer de f à f1
        //    // aligne tref avec t1 par une rotation autour de tref x t1
        //    a1 = MVector.CrossProduct(rf.ZAxis, f1.ZAxis);
        //    cosα_1 = MVector.DotProduct(rf.ZAxis, f1.ZAxis);
        //    sinα_1 = a1.Length();

        //    var θ1 = - Rotation.ZAngle_Rotation(rf, rf.ZAxis, f1, f1.ZAxis);
        //    cosθ_1 = Math.Cos(θ1);
        //    sinθ_1 = Math.Sin(θ1);

        //    a1 = (1 / sinα_1) * a1;
        //    u1 = new MVector(rf.Origin, f1.Origin);

        //    // convert to local
        //    a1 = new MVector(a1 * rf.XAxis, a1 * rf.YAxis, 0);
        //    u1 = new MVector(u1 * rf.XAxis, u1 * rf.YAxis, u1 * rf.ZAxis);

        //    // On détermine les 2 rotations pour passer de f à f2
        //    // aligne tref avec t2 par une rotation autour de tref x t2
        //    a2 = MVector.CrossProduct(rf.ZAxis, f2.ZAxis);
        //    cosα_2 = MVector.DotProduct(rf.ZAxis, f2.ZAxis);
        //    sinα_2 = a2.Length();

        //    var θ2 = - Rotation.ZAngle_Rotation(rf, rf.ZAxis, f2, f2.ZAxis);
        //    cosθ_2 = Math.Cos(θ2);
        //    sinθ_2 = Math.Sin(θ2);

        //    a2 = (2 / sinα_2) * a2;
        //    u2 = new MVector(rf.Origin, f2.Origin);

        //    // convert to local
        //    a2 = new MVector(a2 * rf.XAxis, a2 * rf.YAxis, 0);
        //    u2 = new MVector(u2 * rf.XAxis, u2 * rf.YAxis, u2 * rf.ZAxis);

        //    // Orient f1 and f2 to the target frame
        //    a1 = a1.X * f.XAxis + a1.Y * f.YAxis + a1.Z * f.ZAxis;
        //    var of1 = Rotation.Rotate(f, sinα_1, cosα_1, a1);
        //    of1 = Rotation.ZRotate(of1, sinθ_1, cosθ_1);

        //    u1 = u1.X * f.XAxis + u1.Y * f.YAxis + u1.Z * f.ZAxis;
        //    of1.Origin += u1;


        //    a2 = a2.X * f.XAxis + a2.Y * f.YAxis + a2.Z * f.ZAxis;
        //    var of2 = Rotation.Rotate(f, sinα_2, cosα_2, a2);
        //    of2 = Rotation.ZRotate(of2, sinθ_2, cosθ_2);

        //    u2 = u2.X * f.XAxis + u2.Y * f.YAxis + u2.Z * f.ZAxis;
        //    of2.Origin += u2;

        //    DA.SetData(0, rf);
        //    DA.SetData(1, of1);
        //    DA.SetData(2, of2);
        //}
    }
}
