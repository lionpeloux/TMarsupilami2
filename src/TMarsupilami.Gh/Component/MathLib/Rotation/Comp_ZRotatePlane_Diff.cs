using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;
using TMarsupilami.Gh.Type;
using TMarsupilami.Gh.Parameter;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ZRotatePlane_Diff : GH_Component
    {

        public Comp_ZRotatePlane_Diff()
          : base("Z Rotate a Plane - Diff Taylor 3", "DRz",
              "Rotates a plane around its ZAxis. Assumes the rotation twist angle is closed to zero. Use a Taylor developement of order 3.",
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
                return Resources.ZRotatePlane_Diff;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{A2FE0527-56AF-4A9D-AA33-8A02554F2690}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame(s)", "F", "The frame(s) to rotate around their ZAxis.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Z Twist Angle(s)", "dθz", "The small oriented twist angle(s) of rotation around the frame(s) ZAxis.", GH_ParamAccess.list, new List<double>() { 0 });
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame(s)", "Pl", "The rotated frame(s).", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var twistAngles = new List<double>();

            if (!DA.GetDataList(0, frames)) { return; }
            if (!DA.GetDataList(1, twistAngles)) { return; }


            int n = frames.Count;

            if (twistAngles.Count > 1 && n != twistAngles.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Frame(s) and Twist Angle(s) lists must have the same number of items.");
                return;
            }


            var watch = Stopwatch.StartNew();

            if (twistAngles.Count == 1) // apply the same rotation angle to every frames
            {
                double dθ = twistAngles[0];
                for (int i = 0; i < frames.Count; i++)
                {
                    frames[i].ZDiffRotate(dθ);
                }
            }
            else
            {
                for (int i = 0; i < frames.Count; i++)
                {
                    frames[i].ZDiffRotate(twistAngles[i]);
                }
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.Elapsed.TotalMilliseconds + " ms");

            DA.SetDataList(0, frames);
        }
    }
}
