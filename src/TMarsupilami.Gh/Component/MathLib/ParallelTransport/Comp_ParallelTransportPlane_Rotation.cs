using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ParallelTransportPlane_Rotation : GH_Component
    {

        public Comp_ParallelTransportPlane_Rotation()
          : base("Parallel Transport a Plane - Rotation", "PT (Rot)",
              "Parallel transports a plane through a list of (P,t) tuples. Use the rotation method.",
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
            pManager.AddPlaneParameter("Plane", "Pl", "Initial plane to parallel transport.", GH_ParamAccess.item);
            pManager.AddPointParameter("Target Point(s)", "P", "Points to parallel transport to.", GH_ParamAccess.list);
            pManager.AddVectorParameter("Target Direction(s)", "v", "Vectors to parallel transport to.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "Pl", "The parallel transported planes.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var plane = new Plane();
            List<Point3d> point_list = new List<Point3d>();
            List<Vector3d> direction_list = new List<Vector3d>();

            if (!DA.GetData(0, ref plane)) { return; }
            if (!DA.GetDataList(1, point_list)) { return; }
            if (!DA.GetDataList(2, direction_list)) { return; }


            int n = point_list.Count;

            if (n != direction_list.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Points and Directions list must have the same number of items.");
                return;
            }

            if (n < 2) // we move 
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Points and Directions lists must have at least 2 items.");
                return;
            }

            // Cast from GH to Marsupilami types
            var frame = plane.Cast();
            var points = new MPoint[n];
            var directions = new MVector[n];
            for (int i = 0; i < n; i++)
            {
                points[i] = point_list[i].Cast();
                direction_list[i].Unitize(); // make sure vectors are of unit length
                directions[i] = direction_list[i].Cast();
            }

            var frames = new MFrame[n];

            var watch = Stopwatch.StartNew();


            // First frame
            frames[0] = frame;

            // Next frames
            for (int i = 1; i < point_list.Count; i++)
            {
                frames[i - 1].ParallelTransport_Rotation(directions[i - 1], points[i], directions[i], ref frames[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.ElapsedMilliseconds + " ms");

            DA.SetDataList(0, frames.Cast());
        }
    }
}
