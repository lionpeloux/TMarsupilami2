using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZParallelTransportPlane_Rotation : GH_Component
    {

        public Comp_ZParallelTransportPlane_Rotation()
          : base("Z Parallel Transport a Plane - Rotation", "ZPT (Rot)",
              "Parallel transports a plane from it's origin and Z vector through a list of (P,t) tuples. Use the rotation method.",
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
            pManager.AddPlaneParameter("Plane", "Pl", "Initial plane to parallel transport.", GH_ParamAccess.item);
            pManager.AddPointParameter("Target Point(s)", "P", "Points to parallel transport to.", GH_ParamAccess.list);
            pManager.AddVectorParameter("Target Direction(s)", "v", "Vectors to parallel transport to.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Include", "inc", "If True, includes the initial plane in the output list.", GH_ParamAccess.item, false);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "Pl", "The parallel transported planes.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool isIncluded = false;
            var plane = new Plane();
            List<Point3d> point_list = new List<Point3d>();
            List<Vector3d> direction_list = new List<Vector3d>();

            if (!DA.GetData(0, ref plane)) { return; }
            if (!DA.GetDataList(1, point_list)) { return; }
            if (!DA.GetDataList(2, direction_list)) { return; }
            if (!DA.GetData(3, ref isIncluded)) { return; }

            int n = point_list.Count;

            if (n != direction_list.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Points and Directions list must have the same number of items.");
                return;
            }

            // Cast from GH to Marsupilami types
            var frame = plane.Cast();
            var points = new MPoint[n];
            var directions = new MVector[n];
            for (int i = 0; i < n; i++)
            {
                points[i] = point_list[i].Cast();
                direction_list[i].Unitize(); // make sure vertors are of unit length
                directions[i] = direction_list[i].Cast();
                Rhino.RhinoApp.WriteLine("" + directions[i].LengthSquared());
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
                for (int i = 1; i < point_list.Count; i++)
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
                for (int i = 1; i < point_list.Count; i++)
                {
                    frames[i - 1].ZParallelTransport_Rotation(directions[i - 1], points[i], directions[i], ref frames[i]);
                }
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.ElapsedMilliseconds + " ms");

            DA.SetDataList(0, frames.Cast());
        }
    }
}
