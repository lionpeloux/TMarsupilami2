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
    public class Comp_InterpolateFrames : GH_Component
    {

        public Comp_InterpolateFrames()
          : base("Interpolate Frames - Circumbscribed / Rotation", "Interpolate",
              "Interpolate a frame between each pair of consecutive frames. Relies on the circumbscribed osculating circle for curvature and on the rotation method for parallel tranport.",
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
            get { return new Guid("{600EE9D0-5D72-48EE-98F1-5A8C3CB54D3A}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "Centerline frames to interpolate.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Closed", "B", "True if the centerline is closed. False otherwise.", GH_ParamAccess.item, false);
            pManager.AddIntegerParameter("Recursion Count", "f", "How many times the interpolation is recursively applied (>= 1).", GH_ParamAccess.item, 1);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "A finer centerline with one more frame per edge.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames_tmp = new List<MFrame>();
            var isClosed = false;
            var recursionCount = 1;

            if (!DA.GetDataList(0, frames_tmp)) { return; }
            if (frames_tmp.Count < 3) return;
            DA.GetData(1, ref isClosed);
            DA.GetData(2, ref recursionCount);

            if (recursionCount < 1)
            {
                recursionCount = 1;
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Recursion factor must be >= 1. The factor has been set to 1.");
            }

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

            MFrame[] framesInterp;

            Centerline.GetCurvature(frames, x, e, u, l, t, κb, τ, isClosed);
            Centerline.ZAlignFrames(frames, t);
            Centerline.GetTwist(frames, l, τ, isClosed);
            framesInterp = Centerline.Interpolate(frames, κb, τ, isClosed, recursionCount);

            DA.SetDataList(0, framesInterp);
        }
    }
}
