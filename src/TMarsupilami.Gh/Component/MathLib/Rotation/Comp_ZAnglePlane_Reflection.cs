using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZAnglePlane_Reflection : GH_Component
    {

        public Comp_ZAnglePlane_Reflection()
          : base("Z Angle between Planes - Reflection", "AZ",
              "Gets the Z angle (or minimal twist angle along the ZAxis) two align two planes after parallel transport. Relies on the double reflection method.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.quarternary;
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
            get { return new Guid("{164D4A40-2BE0-4CFF-B412-F605A54AFAB5}"); }
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

            // Cast from GH to Marsupilami types
            var frames = plane_list.Cast();

            // Get unit ZAxis
            var zaxis = new MVector[n];
            for (int i = 0; i < frames.Count; i++)
            {
                zaxis[i] = plane_list[i].ZAxis.Cast();
            }

            var angles = new double[n - 1];

            var watch = Stopwatch.StartNew();
            for (int i = 1; i < plane_list.Count; i++)
            {
                angles[i - 1] = Rotation.ZAngle_Reflection(frames[i - 1], zaxis[i - 1], frames[i], zaxis[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.ElapsedMilliseconds + " ms");

            DA.SetDataList(0, angles);
        }
    }
}
