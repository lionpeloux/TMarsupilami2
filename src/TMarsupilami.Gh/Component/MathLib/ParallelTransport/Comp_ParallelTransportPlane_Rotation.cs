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
    public class Comp_ParallelTransportPlane_Rotation : GH_Component
    {

        public Comp_ParallelTransportPlane_Rotation()
          : base("Parallel Transport a Frame - Rotation", "PT (Rot)",
              "Parallel transports a frame through a list of (Point, Direction) tuples. Relies on the rotation method.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.tertiary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.ParallelTransportPlane_Rotation;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{971D1601-FF8F-48DE-945F-A33916C45E03}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "Initial frame to parallel transport.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "Target Point(s)", "P", "Point(s) to parallel transport to.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "Target Direction(s)", "V", "Vector(s) to parallel transport to.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The parallel transported frames.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frame = new MFrame();
            var points = new List<MPoint>();
            var directions = new List<MVector>();

            if (!DA.GetData(0, ref frame)) { return; }
            if (!DA.GetDataList(1, points)) { return; }
            if (!DA.GetDataList(2, directions)) { return; }

            int n = points.Count;

            if (n != directions.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Points and Directions list must have the same number of items.");
                return;
            }

            if (n < 2) // we move 
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Points and Directions lists must have at least 2 items.");
                return;
            }

            for (int i = 0; i < n; i++)
            {
                directions[i].Normalize();
            }

            var frames = new MFrame[n];

            var watch = Stopwatch.StartNew();


            // First frame
            frames[0] = frame;

            // Next frames
            for (int i = 1; i < points.Count; i++)
            {
                frames[i - 1].ParallelTransport_Rotation(directions[i - 1], points[i], directions[i], ref frames[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.Elapsed.TotalMilliseconds + " ms");

            DA.SetDataList(0, frames.Cast());
        }
    }
}
