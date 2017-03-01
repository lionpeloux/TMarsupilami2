using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace TMarsupilami.Gh
{
    public class Comp_InscribedCircleCurrent : GH_Component
    {

        public Comp_InscribedCircleCurrent()
          : base("Inscribed Circle - Current", "Circle Current",
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

        public override Guid ComponentGuid
        {
            get { return new Guid("{2409612D-18F1-4B37-895F-4F4A091BEDFE}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Ps", "Ps", "Start point.", GH_ParamAccess.item);
            pManager.AddPointParameter("P", "P", "Mid point", GH_ParamAccess.item);
            pManager.AddPointParameter("Pe", "Pe", "End point", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature",GH_ParamAccess.item);
            pManager.AddVectorParameter("Curvature Binormal Vector", "kb", "Circle curvature bionormal vector.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Unit Tangent Vector at Ps", "ts", "Circle unit tangent vector at Ps.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Unit Tangent Vector at Pe", "te", "Circle unit tangent vector at Pe.", GH_ParamAccess.item);
            pManager.AddNumberParameter("fs", "fs", "Turning angle between (ts,t).", GH_ParamAccess.item);
            pManager.AddNumberParameter("Turning Angle (e1,e2)", "f", "Turning angle batween (e1,e2) (f = fs+fe).", GH_ParamAccess.item);
            pManager.AddNumberParameter("fe", "fe", "Turning angle between (t,te).", GH_ParamAccess.item);
            pManager.AddGeometryParameter("Circle", "C", "Circle passing through Ps, P, Pe. Can be a line if points are aligned.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Point3d ps = new Point3d();
            Point3d p = new Point3d();
            Point3d pe = new Point3d();

            if (!DA.GetData(0, ref ps)) { return; }
            if (!DA.GetData(1, ref p)) { return; }
            if (!DA.GetData(2, ref pe)) { return; }

            double κ;
            MathLib.Vector κb;
            MathLib.Vector ts, t, te;
            double fs, f, fe;

            var b1 = (ps == p);
            var b2 = (pe == p);

            if (b1 || b2) // ps = p or pe : p
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "(Ps, P) and (P, Pe) must be disjoints.");
                return; 
            }
            else
            {
                MathLib.Circle.InscribedCircle_Current(ps.Cast(), p.Cast(), pe.Cast(), out κ, out κb, out t, out f);

                if (κ == 0) // it's a line
                {
                    var line = new Line(p, t.Cast());
                    DA.SetData(0, 0);
                    DA.SetData(1, κb.Cast());
                    DA.SetData(2, t.Cast());
                    DA.SetData(3, t.Cast());
                    DA.SetData(4, t.Cast());
                    DA.SetData(5, 0);
                    DA.SetData(6, f);
                    DA.SetData(7, 0);
                    DA.SetData(8, line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MathLib.Vector.CrossProduct(b, t);
                    var center = p.Cast() + (r/Math.Cos(f/2)) * n;
                    var plane = new Plane(center.Cast(), t.Cast(), n.Cast());
                    var circle = new Circle(plane, center.Cast(), r);
                    DA.SetData(0, κ);
                    DA.SetData(1, κb.Cast());
                    DA.SetData(2, t.Cast());
                    DA.SetData(3, t.Cast());
                    DA.SetData(4, t.Cast());
                    DA.SetData(5, 0);
                    DA.SetData(6, f);
                    DA.SetData(7, 0);
                    DA.SetData(8, circle);
                }
            }            
        }
    }
}
