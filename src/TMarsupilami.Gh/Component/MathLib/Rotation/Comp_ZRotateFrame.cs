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
    public class Comp_ZRotateFrame : GH_Component
    {

        public Comp_ZRotateFrame()
          : base("Z Rotate a Frame", "Rz",
              "Rotates a frame around its ZAxis.",
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
            get { return new Guid("{36D0A27F-A7C1-491E-B360-E50EC508F662}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame(s)", "F", "The frames(s) to rotate around their ZAxis.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Z Twist Angle(s)", "θz", "The oriented twist angle(s) of rotation around the frame(s) ZAxis.", GH_ParamAccess.list, new List<double>() { 0 });
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame(s)", "F", "The rotated frame(s).", GH_ParamAccess.list);
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
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Frame(s) and Twist Angle(s) lists must have the same number of items.");
                return;
            }

            var frames = plane_list.Cast();

            var watch = Stopwatch.StartNew();

            if (angle_list.Count == 1) // apply the same rotation angle to every frames
            {
                double θ = angle_list[0];
                for (int i = 0; i < plane_list.Count; i++)
                {
                    var frameROT = new MFrame();
                    frames[i].ZRotate(θ, ref frameROT);
                    frames[i] = frameROT;
                }
            }
            else
            {
                for (int i = 0; i < plane_list.Count; i++)
                {
                    var frame = new MFrame();
                    frames[i].ZRotate(angle_list[i], ref frame);
                    frames[i] = frame;
                }
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.Elapsed.TotalMilliseconds + " ms");

            DA.SetDataList(0, frames.Cast());
        }
    }
}
