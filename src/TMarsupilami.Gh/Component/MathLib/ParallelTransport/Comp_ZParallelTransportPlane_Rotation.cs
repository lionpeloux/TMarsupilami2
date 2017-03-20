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
    public class Comp_ZParallelTransportPlane_Rotation : GH_Component
    {

        public Comp_ZParallelTransportPlane_Rotation()
          : base("Z Parallel Transport a Frame - Rotation", "Z-PT (Rot)",
              "Parallel transports a frame from it's origin and ZAxis through a list of (Point, Z-Direction) tuples. Relies on the rotation method.",
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
                return Resources.ZParallelTransportPlane_Rotation;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{02A24AE6-1F21-43EB-A25A-5FD9E9C6D53F}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "Initial frame to parallel transport.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MPoint(), "Target Point(s)", "P", "Point(s) to parallel transport to.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "Target Direction(s)", "V", "Vector(s) to parallel transport to.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Include", "inc", "If True, includes the initial frame in the output list.", GH_ParamAccess.item, false);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The parallel transported frames.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool isIncluded = false;
            var frame = new MFrame();
            var points = new List<MPoint>();
            var directions = new List<MVector>();

            if (!DA.GetData(0, ref frame)) { return; }
            if (!DA.GetDataList(1, points)) { return; }
            if (!DA.GetDataList(2, directions)) { return; }
            if (!DA.GetData(3, ref isIncluded)) { return; }

            int n = points.Count;

            if (n != directions.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Points and Directions list must have the same number of items.");
                return;
            }

            for (int i = 0; i < n; i++)
            {
                directions[i].Normalize();
            }

            MFrame[] frames;
            var watch = Stopwatch.StartNew();

            if (isIncluded)
            {
                frames = new MFrame[n + 1];

                // First frame
                frames[0] = frame;

                // Second frame
                frames[0].ZParallelTransport_Rotation(frame.ZAxis, points[0], directions[0], ref frames[1]);

                // Next frames
                for (int i = 1; i < points.Count; i++)
                {
                    frames[i].ZParallelTransport_Rotation(directions[i - 1], points[i], directions[i], ref frames[i + 1]);
                }
            }
            else
            {
                frames = new MFrame[n];

                // First frame
                frame.ZParallelTransport_Rotation(frame.ZAxis, points[0], directions[0], ref frames[0]);

                // Next frames
                for (int i = 1; i < points.Count; i++)
                {
                    frames[i - 1].ZParallelTransport_Rotation(directions[i - 1], points[i], directions[i], ref frames[i]);
                }
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.Elapsed.TotalMilliseconds + " ms");

            DA.SetDataList(0, frames);
        }
    }
}
