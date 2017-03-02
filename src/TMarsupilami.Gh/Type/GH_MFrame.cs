using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Display;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Type
{
    public class GH_MFrame : GH_Goo<MFrame>, IGH_PreviewData
    {

        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Frame."; } }

        public override string TypeName { get { return "MFrame"; } }


        #endregion

        #region CONSTRUCTOR
        public GH_MFrame() { }
        public GH_MFrame(GH_MFrame frame)
        {
            this.Value = frame.Value;
        }
        public GH_MFrame(MFrame frame)
        {
            this.Value = frame;
        }
        public GH_MFrame(GH_Plane plane)
        {
            this.Value = plane.Value.Cast();
        }
        public GH_MFrame(Plane plane)
        {
            this.Value = plane.Cast();
        }

        #endregion

        #region INSTANCE METHODS

        public override object ScriptVariable()
        {
            return this.Value;
        }
        public override string ToString()
        {
            return string.Format("O({0:0.00},{1:0.00},{2:0.00}) Z({3:0.00},{4:0.00},{5:0.00})", new object[] {
                this.Value.Origin.X, this.Value.Origin.Y, this.Value.Origin.Z, this.Value.ZAxis.X, this.Value.ZAxis.Y, this.Value.ZAxis.Z });
        }
        public override IGH_Goo Duplicate()
        {
            return new GH_MFrame(this);
        }

        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            if (source.GetType() == typeof(GH_Plane))
            {
                this.Value = ((GH_Plane)source).Value.Cast();
                return true;
            }
            return false;
        }

        // This function is called when Grasshopper needs to convert this 
        // instance of GH_MPoint into some other type T.
        public override bool CastTo<T>(ref T target)
        {
            //First, see if T is similar to the Point3d primitive.
            if (typeof(T).IsAssignableFrom(typeof(Plane)))
            {
                object ptr = this.Value;
                target = (T)ptr;
                return true;
            }

            //Then, see if T is similar to the GH_Point type.
            if (typeof(T).IsAssignableFrom(typeof(GH_Plane)))
            {
                object ptr = new GH_Plane(this.Value.Cast());
                target = (T)ptr;
                return true;
            }

            return false;
        }

        public BoundingBox Boundingbox
        {
            get
            {
                double x = Math.Sqrt(2.0 * Math.Pow(CentralSettings.PreviewPlaneRadius, 2.0));
                Vector3d vectord = new Vector3d(x, x, x);
                Vector3d vectord2 = new Vector3d(x, x, x);
                return new BoundingBox(this.Value.Origin.Cast() - vectord, this.Value.Origin.Cast() + vectord2);
            }
        }
        public BoundingBox ClippingBox
        {
            get
            {
                return this.Boundingbox;
            }
        }
        public void DrawViewportMeshes(GH_PreviewMeshArgs args)
        {
        }
        public void DrawViewportWires(GH_PreviewWireArgs args)
        {
            // this is used to draw parameter input/ouput data in the viewport 
            DrawMFrame(args.Pipeline, this, CentralSettings.PreviewPlaneRadius, args.Color, Color.DarkRed, Color.DarkGreen);
        }
        public void DrawMFrame(DisplayPipeline display, GH_MFrame frame_gh, double size, Color zColor, Color xColor, Color yColor)
        {
            var frame = frame_gh.Value;
            double l = size;
            var Origin = frame.Origin.Cast();
            var t_line = new Line(Origin, frame.ZAxis.Cast(), l);
            var d1_line = new Line(Origin, frame.XAxis.Cast(), l);
            var d2_line = new Line(Origin, frame.YAxis.Cast(), l);

            display.DrawArrow(t_line, zColor);
            display.DrawArrow(d1_line, xColor);
            display.DrawArrow(d2_line, yColor);
        }

        #endregion

    }
}
