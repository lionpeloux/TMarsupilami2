using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_CircumscribedCircleEnd : GH_Component
    {

        public Comp_CircumscribedCircleEnd()
          : base("Circumscribed Circle - End", "Circumscribed Circle (E)",
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
                return Resources.CircumscribedCircleEnd;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{E28C84F6-9857-4002-93C1-E338498B0D5E}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MPoint(), "P", "P", "Previous point.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "Pe", "Pe", "End point", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "te", "te", "Tangent vector at end (must be a unit vector).", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature",GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Curvature Binormal Vector", "kb", "Circle curvature bionormal vector.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.item);
            pManager.AddNumberParameter("fe", "fe", "Turning angle between (e,t).", GH_ParamAccess.item);
            pManager.AddGeometryParameter("Circle", "C", "Circle passing through P, Pe and tangent to te at Pe. Can be a line if points are aligned.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var p = new MPoint();
            var pe = new MPoint();
            var te = new MVector();

            if (!DA.GetData(0, ref p)) { return; }
            if (!DA.GetData(1, ref pe)) { return; }
            if (!DA.GetData(2, ref te)) { return; }

            double κ;
            MVector κb;
            MVector t;
            double fe;

            if (pe == p) // ps = p
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "(P, Pe) must be disjoints.");
                return; 
            }
            else
            {
                te.Normalize();
                OsculatingCircle.CircumscribedCircle_End(p, pe, te, out κ, out κb, out t, out fe);

                if (κ == 0) // it's a line
                {
                    var line = new Line(p.Cast(), t.Cast());
                    DA.SetData(0, κ);
                    DA.SetData(1, κb);
                    DA.SetData(2, t);
                    DA.SetData(3, fe);
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
                    DA.SetData(3, fe);
                    DA.SetData(4, circle);
                }
            }            
        }
    }
}
