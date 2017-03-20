using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using System.Drawing;
using GH_IO.Types;
using Grasshopper;
using TMarsupilami.Gh.Type;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayVector : GH_Component
    {

        private GH_MPoint point;
        private GH_MVector vector;
        private bool isNull;

        public Comp_DisplayVector()
          : base("Vector Display", "VDis",
              "Preview a vector in the viewport.",
              "TMarsupilami", "Display")
        {
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
            get { return new Guid("{67E5F56F-904B-4936-AC5B-E3B5B7E8C2FA}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Point", "P", "Start point where to draw the vector.", GH_ParamAccess.item);
            pManager.AddGenericParameter("Vector", "V", "Vector to preview.", GH_ParamAccess.item);
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
            point = new GH_MPoint();
            vector = new GH_MVector();
            if (DA.GetData(0, ref point) && DA.GetData(1, ref vector)) { isNull = false; }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (!isNull)
            {
                var origin = point.Value.Cast();
                var line = new Line(origin, origin + vector.Value.Cast());

                if (Attributes.GetTopLevel.Selected)
                {
                    //args.Display.DrawLineArrow(t_line, Color.Blue, 2, t_line.Length * 0.2);
                    args.Display.DrawArrow(line, args.WireColour_Selected);
                }
                else
                {
                    //args.Display.DrawLineArrow(t_line, Color.Blue, 2, t_line.Length * 0.2);
                    args.Display.DrawArrow(line, args.WireColour);
                }
            }
        }
    }
}
