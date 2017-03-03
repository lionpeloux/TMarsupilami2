using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZParallelTransportPlane_Reflection : GH_Component
    {

        public Comp_ZParallelTransportPlane_Reflection()
          : base("Z Parallel Transport a Plane (Reflection)", "ZPT (Ref)",
              "Parallel transport a plane from it's origin and Z vector through a list of (P,t) tuples.",
              "TMarsupilami", "Parallel Transport")
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
                return Resources.DisplayPlane;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{8ACD41AD-B9DE-4CB4-99D5-2CAC70355D35}"); }
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
            frame = frame.ParallelTransport_Reflection(frame.ZAxis, point_list[0].Cast(), direction_list[0].Cast());
            planes_pt[0] = frame.Cast();

            // Next frames
            for (int i = 1; i < point_list.Count; i++)
            {
                direction_list[i].Unitize();
                frame = frame.ParallelTransport_Reflection(planes_pt[i-1].ZAxis.Cast(), point_list[i].Cast(), direction_list[i].Cast());
                planes_pt[i] = frame.Cast();
            }
            
            DA.SetDataList(0, planes_pt);
        }
    }
}
