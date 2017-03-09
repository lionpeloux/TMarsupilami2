using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZParallelTransportPlane_Rotation : GH_Component
    {

        public Comp_ZParallelTransportPlane_Rotation()
          : base("Z Parallel Transport a Plane (Rotation)", "ZPT (Rot)",
              "Parallel transports a plane from it's origin and Z vector through a list of (P,t) tuples. Use the rotation method.",
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

            Plane[] planes_pt;
            MFrame frame;

            if (isIncluded)
            {
                planes_pt = new Plane[n+1];

                // First frame
                frame = plane.Cast();
                planes_pt[0] = frame.Cast();


                // Next frames
                for (int i = 0; i < point_list.Count; i++)
                {
                    direction_list[i].Unitize();
                    frame.ZParallelTransport_Rotation(point_list[i].Cast(), direction_list[i].Cast());
                    planes_pt[i+1] = frame.Cast();
                }
            }
            else
            {
                planes_pt = new Plane[n];

                // First frame
                frame = plane.Cast();

                // Next frames
                for (int i = 0; i < point_list.Count; i++)
                {
                    direction_list[i].Unitize();
                    frame.ZParallelTransport_Rotation(point_list[i].Cast(), direction_list[i].Cast());
                    planes_pt[i] = frame.Cast();
                }
            }
                    
            DA.SetDataList(0, planes_pt);
        }
    }
}
