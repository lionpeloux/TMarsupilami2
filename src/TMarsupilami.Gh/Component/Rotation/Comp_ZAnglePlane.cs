using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZAnglePlane : GH_Component
    {

        public Comp_ZAnglePlane()
          : base("Z Angle between Planes", "AZ",
              "Gets the Z angle (or minimal twist angle along the ZAxis) two align two planes after parallel transport.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.senary;
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
            get { return new Guid("{A12F00FE-DD07-44C9-B2EA-D954292DBDAE}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "Pl", "Planes to mesure the Z angle in between.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Z Angle", "Az", "The Z angle between pairs of planes.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var plane_list = new List<Plane>();

            if (!DA.GetDataList(0, plane_list)) { return; }

            int n = plane_list.Count;

            if (plane_list.Count < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The list of planes must have at least 2 items.");
                return;
            }

            var angles = new double[n - 1];

            for (int i = 1; i < plane_list.Count; i++)
            {
                angles[i-1] = Rotation.ZAngle(plane_list[i - 1].Cast(), plane_list[i].Cast());
            }

            DA.SetDataList(0, angles);
        }
    }
}
