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
    public class Comp_RotateFrame : GH_Component
    {

        public Comp_RotateFrame()
          : base("Rotate a Frame", "R",
              "Rotates a frame around an Axis.",
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
        //protected override System.Drawing.Bitmap Icon
        //{
        //    get
        //    {
        //        return Resources.ZRotatePlane;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{D5632355-7B09-4D90-A715-B0F1CB88ADE9}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "The frame to rotate around the axis.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Rotation Angle", "θ", "The oriented rotation angle around the axis.", GH_ParamAccess.item, 0);
            pManager.AddParameter(new Param_MVector(), "Rotation Axis", "A", "The rotation axis.", GH_ParamAccess.item);

            pManager[1].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "The rotated frame.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frame = new MFrame();
            var axis = new MVector();
            double angle = 0;

            if (!DA.GetData(0, ref frame)) { return; }
            if (!DA.GetData(1, ref angle)) { }
            if (!DA.GetData(2, ref axis)) { return; }

            frame.Rotate(angle, axis);

            DA.SetData(0, frame);
        }
    }
}
