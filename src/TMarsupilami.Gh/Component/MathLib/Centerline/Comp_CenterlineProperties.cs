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
    public class Comp_CenterlineProperties : GH_Component
    {

        public Comp_CenterlineProperties()
          : base("Centerline Properties - Circumbscribed / Rotation", "Centerline",
              "Gets the geometric properties of a centerline. Relies on the circumbscribed osculating circle for curvature and on the rotation method for parallel tranport.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.quinary;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{5CE8FB49-2FDF-4892-8993-9080DC506C28}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "Initial frame to parallel transport.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Closed", "B", "True if the centerline is closed. False otherwise.", GH_ParamAccess.item, false);
            pManager[1].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Centerline", "C", "The centerline as a polyline", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The frames, realigned with the computed tangent vectors (t).", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "Curvature", "K", "The curvature binormal (κb) at vertices (x).", GH_ParamAccess.list);
            pManager.AddComplexNumberParameter("Twist", "T", "The rate of twist (τ) over edges (e).", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames_tmp = new List<MFrame>();
            var isClosed = false;

            if (!DA.GetDataList(0, frames_tmp)) { return; }
            if (frames_tmp.Count < 3) return;

            DA.GetData(1, ref isClosed);

            var frames = frames_tmp.ToArray();
            int nv = frames.Length;
            int ne = isClosed ? nv : nv - 1;

            var x = new MPoint[nv];
            var e = new MVector[ne];
            var u = new MVector[ne];
            var t = new MVector[nv];
            var κb = new MVector[nv];
            var l = new double[ne];
            var τ = new double[ne];

            Point3d[] points;

            Centerline.GetCurvature(frames, x, e, u, l, t, κb, isClosed);
            Centerline.ZAlignFrames(frames, t);
            Centerline.GetTwist(frames, l, τ, isClosed);

            if (isClosed)
            {
                points = new Point3d[nv+1];
                for (int i = 0; i < nv; i++)
                {
                    points[i] = frames[i].Origin.Cast();
                }
                points[nv] = points[0];
            }
            else
            {
                points = new Point3d[nv];
                for (int i = 0; i < nv; i++)
                {
                    points[i] = frames[i].Origin.Cast();
                }
            }
            
            var centerline = new Polyline(points);

            DA.SetData(0, centerline);
            DA.SetDataList(1, frames);
            DA.SetDataList(2, κb);
            DA.SetDataList(3, τ);
        }
    }
}
