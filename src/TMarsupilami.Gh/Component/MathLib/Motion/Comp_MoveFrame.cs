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
    public class Comp_MoveFrame : GH_Component
    {

        public Comp_MoveFrame()
          : base("Move a Frame", "Move",
              "Moves a frame by integrating its velocity vectors Vx and Vθ over dt.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.tertiary;
            }
        }
        //protected override System.Drawing.Bitmap Icon
        //{
        //    get
        //    {
        //        return Resources.ParallelTransportPlane_Rotation;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{FEC05EA6-FAFD-44D7-B392-03CC307E2717}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "Initial frame to move.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Parametric Distance", "dt", "The distance in the parameter space between two keyframes.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "Translational Velocity", "VX", "The velocity vector of the frame origin (dx/dt = Vx). Expressed in local coordinate system.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "Rotational Velocity", "Vθ", "The spin vector of the frame (ddk/dt = Vθ x dk). Expressed in local coordinate system.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The motion of the frame.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frame = new MFrame();
            var dt = new List<double>();
            var VX = new List<MVector>();
            var Vθ = new List<MVector>();

            if (!DA.GetData(0, ref frame)) { return; }
            if (!DA.GetDataList(1, dt)) { return; }
            if (!DA.GetDataList(2, VX)) { return; }
            if (!DA.GetDataList(3, Vθ)) { return; }

            int n = dt.Count;

            var frames = new MFrame[n + 1];
            

            var watch = Stopwatch.StartNew();

            frames[0] = frame;
            for (int i = 0; i < n; i++)
            {
                frames[i + 1] = Motion.Move(frames[i], dt[i], VX[i], Vθ[i]);
            }

            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.Elapsed.TotalMilliseconds + " ms");

            DA.SetDataList(0, frames);
        }
    }
}
