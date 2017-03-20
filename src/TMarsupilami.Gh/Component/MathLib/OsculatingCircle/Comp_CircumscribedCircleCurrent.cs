﻿using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_CircumscribedCircleCurrent : GH_Component
    {

        public Comp_CircumscribedCircleCurrent()
          : base("Circumscribed Circle - Current", "Circumscribed Circle (C)",
              "Circle passing through 3 points.",
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
                return Resources.CircumscribedCircleCurrent;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{7735c658-cb3a-4615-a2d3-e87077fcc1f9}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MPoint(), "Ps", "Ps", "Start point.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "P", "P", "Mid point", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "Pe", "Pe", "End point", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature",GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Curvature Binormal Vector", "kb", "Circle curvature bionormal vector.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Unit Tangent Vector at Ps", "ts", "Circle unit tangent vector at Ps.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Unit Tangent Vector at Pe", "te", "Circle unit tangent vector at Pe.", GH_ParamAccess.item);
            pManager.AddNumberParameter("fs", "fs", "Turning angle between (ts,t).", GH_ParamAccess.item);
            pManager.AddNumberParameter("Turning Angle (e1,e2)", "f", "Turning angle batween (e1,e2) (f = fs+fe).", GH_ParamAccess.item);
            pManager.AddNumberParameter("fe", "fe", "Turning angle between (t,te).", GH_ParamAccess.item);
            pManager.AddGeometryParameter("Circle", "C", "Circle passing through Ps, P, Pe. Can be a line if points are aligned.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var ps = new MPoint();
            var p = new MPoint();
            var pe = new MPoint();

            if (!DA.GetData(0, ref ps)) { return; }
            if (!DA.GetData(1, ref p)) { return; }
            if (!DA.GetData(2, ref pe)) { return; }

            double κ;
            MVector κb;
            MVector ts, t, te;
            double fs, f, fe;

            var b1 = (ps == p);
            var b2 = (pe == p);

            if (b1 || b2) // ps = p or pe : p
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "(Ps, P) and (P, Pe) mus be disjoints.");
                return; 
            }
            else
            {
                OsculatingCircle.CircumscribedCircle_Current(ps, p, pe, out κ, out κb, out ts, out t, out te, out fs, out f, out fe);

                if (κ == 0) // it's a line
                {
                    var line = new Line(p.Cast(), t.Cast());
                    DA.SetData(0, 0);
                    DA.SetData(1, κb);
                    DA.SetData(2, ts);
                    DA.SetData(3, t);
                    DA.SetData(4, te);
                    DA.SetData(5, 0);
                    DA.SetData(6, 0);
                    DA.SetData(7, 0);
                    DA.SetData(8, line);
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
                    DA.SetData(2, ts);
                    DA.SetData(3, t);
                    DA.SetData(4, te);
                    DA.SetData(5, fs);
                    DA.SetData(6, f);
                    DA.SetData(7, fe);
                    DA.SetData(8, circle);
                }
            }            
        }
    }
}
