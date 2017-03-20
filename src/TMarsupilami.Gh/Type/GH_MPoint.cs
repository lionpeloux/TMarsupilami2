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
    public class GH_MPoint : GH_Goo<MPoint>, IGH_PreviewData
    {

        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Point."; } }

        public override string TypeName { get { return "MPoint"; } }

        #endregion

        #region CONSTRUCTOR
        public GH_MPoint() { }
        public GH_MPoint(GH_MPoint point)
        {
            this.Value = point.Value;
        }
        public GH_MPoint(MPoint point)
        {
            this.Value = point;
        }
        public GH_MPoint(GH_Point point)
        {
            this.Value = point.Value.Cast();
        }
        public GH_MPoint(Point3d point)
        {
            this.Value = point.Cast();
        }

        #endregion

        #region INSTANCE METHODS

        public override object ScriptVariable()
        {
            return this.Value;
        }
        public override string ToString()
        {
            return string.Format("{{{0}, {1}, {2}}}", GH_Format.FormatDouble(this.Value.X), GH_Format.FormatDouble(this.Value.Y), GH_Format.FormatDouble(this.Value.Z));
        }
        public override IGH_Goo Duplicate()
        {
            return new GH_MPoint(this);
        }
        public GH_MPoint DuplicateMPoint()
        {
            return new GH_MPoint(this);
        }

        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();
            if (type == typeof(GH_Point))
            {
                this.Value = ((GH_Point)source).Value.Cast();
                return true;
            }

            if (type == typeof(GH_MFrame))
            {            
                this.Value = ((GH_MFrame)source).Value.Origin;
                return true;
            }

            if (type == typeof(GH_Plane))
            {
                this.Value = ((GH_Plane)source).Value.Origin.Cast();
                return true;
            }

            return false;
        }

        // This function is called when Grasshopper needs to convert this 
        // instance of GH_MPoint into some other type T.
        public override bool CastTo<T>(ref T target)
        {
            //First, see if T is similar to the Point3d primitive.
            if (typeof(T).IsAssignableFrom(typeof(Point3d)))
            {
                object ptr = this.Value;
                target = (T)ptr;
                return true;
            }

            //Then, see if T is similar to the GH_Point type.
            if (typeof(T).IsAssignableFrom(typeof(GH_Point)))
            {
                object ptr = new GH_Point(this.Value.Cast());
                target = (T)ptr;
                return true;
            }

            return false;
        }

        public BoundingBox Boundingbox
        {
            get
            {
                var point = this.Value.Cast();
                return new BoundingBox(point, point);
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
            DrawMPoint(args.Pipeline, 5, args.Color);
        }
        public void DrawMPoint(DisplayPipeline display, int radius, Color color)
        {
            PointStyle previewPointStyle = CentralSettings.PreviewPointStyle;
            display.DrawPoint(this.Value.Cast(), previewPointStyle, radius, color);
        }
        #endregion

    }
}
