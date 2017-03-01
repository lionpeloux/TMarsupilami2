using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;

namespace TMarsupilami.Gh
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
            pManager.AddVectorParameter("ts", "ts", "Tangent vector at start (must be a unit vector).", GH_ParamAccess.item);
            pManager.AddPointParameter("Ps", "Ps", "Start point.", GH_ParamAccess.item);
            pManager.AddPointParameter("P", "P", "Second point", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature", GH_ParamAccess.item);
            pManager.AddVectorParameter("Curvature Binormal Vector", "kb", "Circle curvature bionormal vector.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.item);
            pManager.AddNumberParameter("fs", "fs", "Turning angle between (ts,t).", GH_ParamAccess.item);
            pManager.AddGeometryParameter("Circle", "C", "Circle passing through Ps, P and tangent to ts at Ps. Can be a line if points are aligned.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Vector3d ts = new Vector3d();
            Point3d ps = new Point3d();
            Point3d p = new Point3d();

            if (!DA.GetData(0, ref ts)) { return; }
            if (!DA.GetData(1, ref ps)) { return; }
            if (!DA.GetData(2, ref p)) { return; }

            double κ;
            MathLib.Vector κb;
            MathLib.Vector t;
            double fs;

            if (ps == p) // ps = p
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "(Ps, P) must be disjoints.");
                return;
            }
            else
            {
                ts.Unitize();
                MathLib.Circle.InscribedCircle_Start(ts.Cast(), ps.Cast(), p.Cast(), out κ, out κb, out fs);

                if (κ == 0) // it's a line
                {
                    var line = new Line(ps, ts);
                    DA.SetData(0, κ);
                    DA.SetData(1, κb.Cast());
                    DA.SetData(2, ts);
                    DA.SetData(3, fs);
                    DA.SetData(4, line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MathLib.Vector.CrossProduct(b, ts.Cast());
                    var center = ps.Cast() + (r / Math.Cos(fs)) * n;
                    var plane = new Plane(center.Cast(), ts, n.Cast());
                    var circle = new Circle(plane, center.Cast(), r);
                    DA.SetData(0, κ);
                    DA.SetData(1, κb.Cast());
                    DA.SetData(2, ts);
                    DA.SetData(3, fs);
                    DA.SetData(4, circle);
                }
            }
        }
    }
}
