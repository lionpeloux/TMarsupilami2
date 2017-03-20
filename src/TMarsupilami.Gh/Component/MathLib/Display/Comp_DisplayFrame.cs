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
using TMarsupilami.Gh.Type;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayFrame : GH_Component
    {
        private List<MFrame> frames;
        private bool isNull;
        private double scaleFactor;
        List<int> linesWidth;
        List<Color> linesColor;

        Color[] linesColorDefault = new Color[3] { Color.Blue, Color.Red, Color.Lime }; // Z, X, Y
        int[] linesWidthDefault = new int[3] { 2, 1, 1 }; // Z, X, Y


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
        protected override Bitmap Icon
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
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "The frame to preview.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Scale Factor", "scale", "Scale factor.", GH_ParamAccess.item, 1.0);
            pManager.AddIntegerParameter("Line Width", "width", "Width of vector lines", GH_ParamAccess.list, linesWidthDefault);
            pManager.AddColourParameter("Color", "color", "Color of vector lines", GH_ParamAccess.list, linesColorDefault);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
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
            frames = new List<MFrame>();
            linesWidth = new List<int>();
            linesColor = new List<Color>();

            if (DA.GetDataList(0, frames)) { isNull = false; }
            if (DA.GetData(1, ref scaleFactor))
            if (DA.GetDataList(2, linesWidth))
            if (DA.GetDataList(3, linesColor))

            if (linesWidth.Count == 1)
            {
                linesWidth.Add(linesWidth[0]);
                linesWidth.Add(linesWidth[0]);
            }

            if (linesColor.Count == 1)
            {
                linesColor.Add(linesColor[0]);
                linesColor.Add(linesColor[0]);
            }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (!isNull && !Locked) // && plane.IsValid
            {
                for (int i = 0; i < frames.Count; i++)
                {
                    var frame = frames[i];
                    double l = CentralSettings.PreviewPlaneRadius * scaleFactor;
                    double h = l * 0.15; // arrow head size
                    var Origin = frame.Origin.Cast();
                    var t_line = new Line(Origin, frame.ZAxis.Cast(), l);
                    var d1_line = new Line(Origin, frame.XAxis.Cast(), l);
                    var d2_line = new Line(Origin, frame.YAxis.Cast(), l);

                    if (Attributes.Selected)
                    {
                        Color color = args.WireColour_Selected;
                        args.Display.DrawLineArrow(t_line, Color.Yellow, linesWidth[0], h);
                        args.Display.DrawLineArrow(d1_line, color, linesWidth[1], h);
                        args.Display.DrawLineArrow(d2_line, color, linesWidth[2], h);
                    }
                    else
                    {
                        args.Display.DrawLineArrow(t_line, linesColor[0], linesWidth[0], h);
                        args.Display.DrawLineArrow(d1_line, linesColor[1], linesWidth[1], h);
                        args.Display.DrawLineArrow(d2_line, linesColor[2], linesWidth[2], h);
                    }
                }            
            }              
        }
    }
}
