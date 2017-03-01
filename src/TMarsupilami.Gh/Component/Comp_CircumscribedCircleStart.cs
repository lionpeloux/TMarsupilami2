using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace TMarsupilami.Gh
{
    public class Comp_CircumscribedCircleStart : GH_Component
    {

        public Comp_CircumscribedCircleStart()
          : base("Circumscribed Circle - Start", "Circle Start",
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

        public override Guid ComponentGuid
        {
            get { return new Guid("{76D1B5D6-AEA8-4B3E-B133-D58DC288C68A}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddVectorParameter("ts", "ts", "Tangent vector at start (must be a unit vector).", GH_ParamAccess.item);
            pManager.AddPointParameter("Ps", "Ps", "Start point.", GH_ParamAccess.item);
            pManager.AddPointParameter("P", "P", "Second point", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature",GH_ParamAccess.item);
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
                MathLib.Circle.CircumscribedCircle_Start(ts.Cast(), ps.Cast(), p.Cast(), out κ, out κb, out t, out fs);

                if (κ == 0) // it's a line
                {
                    var line = new Line(ps, t.Cast());
                    DA.SetData(0, κ);
                    DA.SetData(1, κb.Cast());
                    DA.SetData(2, t.Cast());
                    DA.SetData(3, fs);
                    DA.SetData(4, line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MathLib.Vector.CrossProduct(b, t);
                    var center = p.Cast() + r * n;
                    var plane = new Plane(center.Cast(), t.Cast(), n.Cast());
                    var circle = new Circle(plane, center.Cast(), r);
                    DA.SetData(0, κ);
                    DA.SetData(1, κb.Cast());
                    DA.SetData(2, t.Cast());
                    DA.SetData(3, fs);
                    DA.SetData(4, circle);
                }
            }            
        }
    }
}
