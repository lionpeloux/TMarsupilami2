using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZRotatePlan : GH_Component
    {

        public Comp_ZRotatePlan()
          : base("Z Rotate a Plane", "RZ",
              "Rotates a plane around its ZAxis.",
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
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.ZRotatePlane;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{36D0A27F-A7C1-491E-B360-E50EC508F662}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane(s)", "Pl", "Plane(s) to rotate around their ZAxis.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Rotation Angle(s)", "θ", "Oriented angle(s) of rotation around the ZAxis.", GH_ParamAccess.list, new List<double>() { 0 });
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "Pl", "The rotated planes.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var plane_list = new List<Plane>();
            var angle_list = new List<double>();

            if (!DA.GetDataList(0, plane_list)) { return; }
            if (!DA.GetDataList(1, angle_list)) { return; }


            int n = plane_list.Count;

            if (angle_list.Count > 1 && n != angle_list.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Plane(s) and Angle(s) lists must have the same number of items.");
                return;
            }

            var planes_rot = new Plane[n];
            MFrame frame;

            if (angle_list.Count == 1) // apply the same rotation angle to every frames
            {
                double θ = angle_list[0];
                for (int i = 0; i < plane_list.Count; i++)
                {
                    frame = plane_list[i].Cast();
                    frame.ZRotate(θ);
                    planes_rot[i] = frame.Cast();
                }
            }
            else
            {
                for (int i = 0; i < plane_list.Count; i++)
                {
                    frame = plane_list[i].Cast();
                    frame.ZRotate(angle_list[i]);
                    planes_rot[i] = frame.Cast();
                }
            }
           
            DA.SetDataList(0, planes_rot);
        }
    }
}
