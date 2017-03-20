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
            return Value.ToString();
        }
        public override IGH_Goo Duplicate()
        {
            return new GH_MFrame(this);
        }
        public GH_MFrame DuplicateMFrame()
        {
            return new GH_MFrame(this);
        }

        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();
            if (type == typeof(MFrame))
            {
                this.Value = (MFrame)source;
                return true;
            }
            if (type == typeof(GH_Plane))
            {
                this.Value = ((GH_Plane)source).Value.Cast();
                return true;
            }

            return false;
        }
        public override bool CastTo<T>(ref T target)
        {
            var type = typeof(T);

            if (type.IsAssignableFrom(typeof(MFrame)))
            {
                object ptr = new MFrame(this.Value);
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(Plane)))
            {
                object ptr = this.Value.Cast();
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(GH_Plane)))
            {
                object ptr = new GH_Plane(this.Value.Cast());
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(MPoint)))
            {
                object ptr = new MPoint(this.Value.Origin);
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(GH_MPoint)))
            {
                object ptr = new GH_MPoint(this.Value.Origin);
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(Point3d)))
            {
                object ptr = this.Value.Origin.Cast();
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(GH_Point)))
            {
                object ptr = new GH_Point(this.Value.Origin.Cast());
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
            DrawMFrame(args.Pipeline, CentralSettings.PreviewPlaneRadius, args.Color, Color.DarkRed, Color.DarkGreen);
        }
        public void DrawMFrame(DisplayPipeline display, double size, Color zColor, Color xColor, Color yColor)
        {
            var frame = this.Value;
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
