using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_InscribedCircleStart : GH_Component
    {

        public Comp_InscribedCircleStart()
          : base("Inscribed Circle - Start", "Inscribed Circle (S)",
              "Circle tangent to two edges.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.secondary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.InscribedCircleStart;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{58D4E950-6C6D-4F6F-91D0-3407219D4EDA}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MVector(), "ts", "ts", "Tangent vector at start (must be a unit vector).", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "Ps", "Ps", "Start point.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "P", "P", "Second point", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "κ", "Circle curvature", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Curvature Binormal Vector", "κb", "Circle curvature bionormal vector.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.item);
            pManager.AddNumberParameter("φs", "φs", "Turning angle between (ts,t).", GH_ParamAccess.item);
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

            if (ps.Cast() == p.Cast()) // ps = p
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "(Ps, P) must be disjoints.");
                return;
            }
            else
            {
                ts.Normalize();
                MathLib.OsculatingCircle.InscribedCircle_Start(ts, ps, p, out κ, out κb, out fs);

                if (κ == 0) // it's a line
                {
                    var line = new Line(ps.Cast(), ts.Cast());
                    DA.SetData(0, κ);
                    DA.SetData(1, κb);
                    DA.SetData(2, ts);
                    DA.SetData(3, fs);
                    DA.SetData(4, line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MVector.CrossProduct(b, ts);
                    var center = ps + (r / Math.Cos(fs)) * n;
                    var frame = new MFrame(center, ts, n);
                    var circle = new Rhino.Geometry.Circle(frame.Cast(), center.Cast(), r);
                    DA.SetData(0, κ);
                    DA.SetData(1, κb);
                    DA.SetData(2, ts);
                    DA.SetData(3, fs);
                    DA.SetData(4, circle);
                }
            }
        }
    }
}
