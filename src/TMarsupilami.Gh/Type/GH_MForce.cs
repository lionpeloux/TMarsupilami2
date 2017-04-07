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
using TMarsupilami.CoreLib3;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Type
{
    public class GH_MForce : GH_Goo<Force>
    {
        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Concentrated Force."; } }

        public override string TypeName { get { return "MForce"; } }

        #endregion

        #region CONSTRUCTOR
        public GH_MForce() { }
        public GH_MForce(GH_MForce ghForce)
        {
            this.Value = ghForce.Value;
        }
        public GH_MForce(Force force)
        {
            this.Value = force;
        }
        #endregion

        #region INSTANCE METHODS
        public override object ScriptVariable()
        {
            return this.Value;
        }
        public override string ToString()
        {
            return this.Value.ToString();
        }
        public override IGH_Goo Duplicate()
        {
            return new GH_MForce(this);
        }
        
        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();

            if (type == typeof(Force))
            {
                this.Value = (Force)source;
                return true;
            }

            if (type == typeof(GH_MForce))
            {
                this.Value = ((GH_MForce)source).Value;
                return true;
            }

            return false;
        }
        public override bool CastTo<T>(ref T target)
        {
            if (typeof(T).IsAssignableFrom(typeof(MVector)))
            {
                object ptr = this.Value.ValueInGCS;
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_MVector)))
            {
                object ptr = new GH_MVector(this.Value.ValueInGCS);
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(Vector3d)))
            {
                object ptr = this.Value.ValueInGCS.Cast();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Vector)))
            {
                object ptr = new GH_Vector(this.Value.ValueInGCS.Cast());
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(double)))
            {
                object ptr = this.Value.ValueInGCS.Length();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Number)))
            {
                object ptr = new GH_Number(this.Value.ValueInGCS.Length());
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(Plane)))
            {
                object ptr = this.Value.LocalFrame.Cast();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Plane)))
            {
                object ptr = new GH_Plane(this.Value.LocalFrame.Cast());
                target = (T)ptr;
                return true;
            }

            return false;
        }

        // PREVIEW
        public void DrawForce(MVector vector, DisplayPipeline display, Color color, double scale = 1, double arrowSize = 0.15, int lineWidth = 1)
        {
            var origin = Value.LocalFrame.Origin.Cast();
            var force = vector.Cast();
            double l = scale * force.Length;
            var line = new Line(origin, force, l);
            double h = l * arrowSize; // arrow head size

            if (l > 1e-3)
            {
                if (arrowSize != 0)
                    display.DrawLineArrow(line, color, lineWidth, h);
                else
                    display.DrawLine(line, color, lineWidth);

                display.Draw2dText(String.Format("{0:E3} N", force.Length), color, line.From + 1.2 * line.Direction, true);
            }
            
        }
    }
        #endregion
}
