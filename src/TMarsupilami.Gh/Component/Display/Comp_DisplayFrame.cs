using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using System.Drawing;
using GH_IO.Types;
using Grasshopper;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayFrame : GH_Component
    {
        private Plane plane;
        private bool isNull;

        public Comp_DisplayFrame()
          : base("Frame Display", "FDis",
              "Preview a frame in the viewport.",
              "TMarsupilami", "Display")
        {
            isNull = true;
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
                return Resources.DisplayPlane;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{C211215F-DDFD-4D8E-9285-E1BECBF0539F}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("F", "F", "The frame to preview.", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        protected override void BeforeSolveInstance()
        {
            isNull = true;
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frame = new MFrame();
            if (DA.GetData(0, ref frame)) { isNull = false; }
            plane = frame.Cast();
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (!isNull && plane.IsValid)
            {
                double l = CentralSettings.PreviewPlaneRadius;
                var Origin = plane.Origin;
                var t_line = new Line(Origin, plane.ZAxis, l);
                var d1_line = new Line(Origin, plane.XAxis, l);
                var d2_line = new Line(Origin, plane.YAxis, l);

                if (Attributes.Selected)
                {
                    //args.Display.DrawLineArrow(t_line, Color.Blue, 2, t_line.Length * 0.2);
                    args.Display.DrawArrow(t_line, Color.Yellow);
                    args.Display.DrawArrow(d1_line, Color.Green);
                    args.Display.DrawArrow(d2_line, Color.Green);
                }
                else
                {
                    //args.Display.DrawLineArrow(t_line, Color.Blue, 2, t_line.Length * 0.2);
                    args.Display.DrawArrow(t_line, Color.Blue);
                    args.Display.DrawArrow(d1_line, Color.Red);
                    args.Display.DrawArrow(d2_line, Color.Red);
                }
            }              
        }
    }
}
