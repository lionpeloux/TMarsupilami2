using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ParallelTransportPlane_Reflection : GH_Component
    {

        public Comp_ParallelTransportPlane_Reflection()
          : base("Parallel Transport a Plane (Reflection)", "PT (Ref)",
              "Parallel transport a plane through a list of (P,t) tuples.",
              "TMarsupilami", "Parallel Transport")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.secondary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.DisplayPlane;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{A62F17CD-49F1-4759-BB9F-F59CE5824A7D}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "Pl", "Initial plane to parallel transport.", GH_ParamAccess.item);
            pManager.AddPointParameter("Target Point(s)", "P", "Points to parallel transport to.", GH_ParamAccess.list);
            pManager.AddVectorParameter("Target Direction(s)", "t", "Vectors to parallel transport to.", GH_ParamAccess.list);
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

            var planes_pt = new Plane[n];
            MFrame frame;

            // First frame
            frame = plane.Cast();
            direction_list[0].Unitize();
            planes_pt[0] = frame.Cast();

            // Next frames
            for (int i = 1; i < point_list.Count; i++)
            {
                direction_list[i].Unitize();
                frame = frame.ParallelTransport_Reflection(direction_list[i-1].Cast(), point_list[i].Cast(), direction_list[i].Cast());
                planes_pt[i] = frame.Cast();
            }
            
            DA.SetDataList(0, planes_pt);
        }
    }
}
