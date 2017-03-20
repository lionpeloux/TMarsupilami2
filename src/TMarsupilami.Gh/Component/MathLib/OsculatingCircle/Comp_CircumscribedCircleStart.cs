using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_CircumscribedCircleStart : GH_Component
    {

        public Comp_CircumscribedCircleStart()
          : base("Circumscribed Circle - Start", "Circumscribed Circle (S)",
              "Circle passing through 2 points and tangent to a vector.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.primary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.CircumscribedCircleStart;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{76D1B5D6-AEA8-4B3E-B133-D58DC288C68A}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MVector(), "ts", "ts", "Tangent vector at start (must be a unit vector).", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "Ps", "Ps", "Start point.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "P", "P", "Second point", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature",GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Curvature Binormal Vector", "kb", "Circle curvature bionormal vector.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.item);
            pManager.AddNumberParameter("fs", "fs", "Turning angle between (ts,t).", GH_ParamAccess.item);
            pManager.AddGeometryParameter("Circle", "C", "Circle passing through Ps, P and tangent to ts at Ps. Can be a line if points are aligned.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var ts = new MVector();
            var ps = new MPoint();
            var p = new MPoint();

            if (!DA.GetData(0, ref ts)) { return; }
            if (!DA.GetData(1, ref ps)) { return; }
            if (!DA.GetData(2, ref p)) { return; }

            double κ;
            MVector κb;
            MVector t;
            double fs;

            if (ps == p) // ps = p
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "(Ps, P) must be disjoints.");
                return; 
            }
            else
            {
                ts.Normalize();
                OsculatingCircle.CircumscribedCircle_Start(ts, ps, p, out κ, out κb, out t, out fs);

                if (κ == 0) // it's a line
                {
                    var line = new Line(ps.Cast(), t.Cast());
                    DA.SetData(0, κ);
                    DA.SetData(1, κb);
                    DA.SetData(2, t);
                    DA.SetData(3, fs);
                    DA.SetData(4, line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MVector.CrossProduct(b, t);
                    var center = p + r * n;
                    var frame = new MFrame(center, t, n);
                    var circle = new Circle(frame.Cast(), center.Cast(), r);
                    DA.SetData(0, κ);
                    DA.SetData(1, κb);
                    DA.SetData(2, t);
                    DA.SetData(3, fs);
                    DA.SetData(4, circle);
                }
            }            
        }
    }
}
