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
    public class GH_MDForce : GH_Goo<DForce>
    {
        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Distributed Force."; } }

        public override string TypeName { get { return "MDForce"; } }

        #endregion

        #region CONSTRUCTOR
        public GH_MDForce() { }
        public GH_MDForce(GH_MDForce ghForce)
        {
            this.Value = ghForce.Value;
        }
        public GH_MDForce(DForce force)
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
            return new GH_MDForce(this);
        }
        
        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();

            if (type == typeof(DForce))
            {
                this.Value = (DForce)source;
                return true;
            }

            if (type == typeof(GH_MDForce))
            {
                this.Value = ((GH_MDForce)source).Value;
                return true;
            }

            return false;
        }
        public override bool CastTo<T>(ref T target)
        {
            if (typeof(T).IsAssignableFrom(typeof(MVector)))
            {
                object ptr = this.Value.Value;
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_MVector)))
            {
                object ptr = new GH_MVector(this.Value.Value);
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(Vector3d)))
            {
                object ptr = this.Value.Value.Cast();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Vector)))
            {
                object ptr = new GH_Vector(this.Value.Value.Cast());
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(double)))
            {
                object ptr = this.Value.Value.Length();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Number)))
            {
                object ptr = new GH_Number(this.Value.Value.Length());
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
        public void Draw(MVector vector, DisplayPipeline display, Color color, double scale = 1, double arrowSize = 0.15, int lineWidth = 1)
        {
            var force = vector.Cast();
            Vector3d scaledForce = scale * force;
            double l = force.Length;

            double h = scale * l * arrowSize; // arrow head size

            var startPointFrom = Value.StartPoint.Cast();
            var endPointFrom = Value.EndPoint.Cast();

            var edge = new Vector3d(endPointFrom - startPointFrom);

            var startPointTo = startPointFrom + scaledForce;
            var endPointTo = endPointFrom + scaledForce;

            var startLine = new Line(startPointFrom, startPointTo);
            var endLine = new Line(endPointFrom, endPointTo);
            var valueLine = new Line(startPointTo, endPointTo);

            if (l > 1e-3)
            {
                if (arrowSize != 0)
                {
                    display.DrawLine(valueLine, color, lineWidth);
                    display.DrawLineArrow(startLine, color, lineWidth, h);
                    display.DrawLineArrow(endLine, color, lineWidth, h);
                }
                else
                {
                    display.DrawLine(valueLine, color, lineWidth);
                    display.DrawLine(startLine, color, lineWidth);
                    display.DrawLine(endLine, color, lineWidth);
                }

                display.Draw2dText(String.Format("{0:E3} N", force.Length), color, 0.5 * (valueLine.From + valueLine.To) + 0.2 * scaledForce, true);
            }         
        }
        public void DrawEdgeComponent(MVector vector, DisplayPipeline display, Color color, double scale = 1, double arrowSize = 0.15, int lineWidth = 1)
        {
            var d3 = Value.LocalFrame.ZAxis;

            var proj = vector * d3;
            var projectedForce = (proj * d3).Cast();

            // direction of representation
            var u = (Value.LocalFrame.XAxis + Value.LocalFrame.YAxis).Cast();
            u.Unitize();

            double l = projectedForce.Length;
            Vector3d scaledForce = scale * l * u;
            double h = scale * l * arrowSize; // arrow head size

            var startPointFrom = Value.StartPoint.Cast();
            var endPointFrom = Value.EndPoint.Cast();

            var startPointTo = startPointFrom + scaledForce;
            var endPointTo = endPointFrom + scaledForce;

            Line startLine = new Line(startPointFrom, startPointTo);
            Line endLine = new Line(endPointFrom, endPointTo);

            Line valueLine;
            if (proj > 0)
                valueLine = new Line(startPointTo, endPointTo);
            else
                valueLine = new Line(endPointTo, startPointTo);

            if (l > 1e-3)
            {
                if (arrowSize != 0)
                {
                    display.DrawLineArrow(valueLine, color, lineWidth, h);
                    display.DrawLine(startLine, color, lineWidth);
                    display.DrawLine(endLine, color, lineWidth);
                }
                else
                {
                    display.DrawLine(valueLine, color, lineWidth);
                    display.DrawLine(startLine, color, lineWidth);
                    display.DrawLine(endLine, color, lineWidth);
                }

                display.Draw2dText(String.Format("{0:E3} N", projectedForce.Length), color, 0.5 * (valueLine.From + valueLine.To) + 0.2 * scaledForce, true);
            }
        }
    }
    #endregion
}
