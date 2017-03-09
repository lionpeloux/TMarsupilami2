using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZRotatePlane_Diff : GH_Component
    {

        public Comp_ZRotatePlane_Diff()
          : base("Z Rotate a Plane - Diff Taylor 3", "DRZ",
              "Rotates a plane around its ZAxis. Assumes the rotation angle is closed to zero. Use a Taylor developement of order 3.",
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
                return Resources.ZRotatePlane_Diff;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{A2FE0527-56AF-4A9D-AA33-8A02554F2690}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane(s)", "Pl", "Plane(s) to rotate around their ZAxis.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Rotation Angle(s)", "dθ", "Small oriented angle(s) of rotation around the ZAxis.", GH_ParamAccess.list, new List<double>() { 0 });
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
                double dθ = angle_list[0];
                for (int i = 0; i < plane_list.Count; i++)
                {
                    frame = plane_list[i].Cast();
                    frame.ZDiffRotate(dθ);
                    planes_rot[i] = frame.Cast();
                }
            }
            else
            {
                for (int i = 0; i < plane_list.Count; i++)
                {
                    frame = plane_list[i].Cast();
                    frame.ZDiffRotate(angle_list[i]);
                    planes_rot[i] = frame.Cast();
                }
            }
           
            DA.SetDataList(0, planes_rot);
        }
    }
}
