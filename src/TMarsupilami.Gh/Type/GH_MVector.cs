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
    public class GH_MVector : GH_Goo<MVector>
    {

        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Vector."; } }

        public override string TypeName { get { return "MVector"; } }

        #endregion

        #region CONSTRUCTOR
        public GH_MVector() { }
        public GH_MVector(GH_MVector vector)
        {
            this.Value = vector.Value;
        }
        public GH_MVector(MVector vector)
        {
            this.Value = vector;
        }
        public GH_MVector(GH_Vector vector)
        {
            this.Value = vector.Value.Cast();
        }
        public GH_MVector(Vector3d vector)
        {
            this.Value = vector.Cast();
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
            return new GH_MVector(this);
        }
        public GH_MVector DuplicateMVector()
        {
            return new GH_MVector(this);
        }

        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();

            if (type == typeof(MVector))
            {
                this.Value = (MVector)source;
                return true;
            }

            if (type == typeof(Vector3d))
            {
                this.Value = ((Vector3d)source).Cast();
                return true;
            }

            return false;
        }
        public override bool CastTo<T>(ref T target)
        {
            if (typeof(T).IsAssignableFrom(typeof(MVector)))
            {
                object ptr = new MVector(this.Value);
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(Vector3d)))
            {
                object ptr = this.Value.Cast();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Vector)))
            {
                object ptr = new GH_Vector(this.Value.Cast());
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(double)))
            {
                object ptr = this.Value.Length();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Number)))
            {
                object ptr = new GH_Number(this.Value.Length());
                target = (T)ptr;
                return true;
            }

            return false;
        }

        // PREVIEW
        public void DrawMVector(DisplayPipeline display, Point3d origin, double size, Color color)
        {
            double l = size;
            var line = new Line(origin, this.Value.Cast(), l);
            display.DrawArrow(line, color);
        }
        #endregion

    }
}
