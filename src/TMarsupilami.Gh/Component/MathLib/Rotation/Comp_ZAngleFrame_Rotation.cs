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
    public class Comp_ZAngleFrame_Rotation : GH_Component
    {

        public Comp_ZAngleFrame_Rotation()
          : base("Z Twist Angle between Frames - Rotation", "θz",
              "Gets the Z twist angle (or minimal twist angle along the ZAxis) to align two frames after parallel transport. Relies on the rotation method.",
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
                return Resources.ZAngle_Rotation;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{A12F00FE-DD07-44C9-B2EA-D954292DBDAE}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The frames to mesure the Z twist angle in between.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Z Twist Angle(s)", "θz", "The Z twist angle(s) between consecutive pair(s) of planes.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();

            if (!DA.GetDataList(0, frames)) { return; }

            int n = frames.Count;

            if (frames.Count < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The list of frames must have at least 2 items.");
                return;
            }


            // Get unit ZAxis
            var zaxis = new MVector[n];
            for (int i = 0; i < frames.Count; i++)
            {
                zaxis[i] = frames[i].ZAxis;
                zaxis[i].Normalize();
            }

            var angles = new double[n - 1];

            var watch = Stopwatch.StartNew();
            for (int i = 1; i < frames.Count; i++)
            {
                angles[i - 1] = Rotation.ZAngle_Rotation(frames[i - 1], zaxis[i - 1], frames[i], zaxis[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.Elapsed.TotalMilliseconds + " ms");

            DA.SetDataList(0, angles);
        }
    }
}
