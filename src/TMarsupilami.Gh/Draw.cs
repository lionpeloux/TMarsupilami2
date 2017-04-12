using Grasshopper;
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

namespace TMarsupilami.Gh
{
    public static class Draw
    {
        public static void DrawConcentratedForce(CForce force, DisplayPipeline display, Color color, double scale = 1.0, bool pointsToApplicationPoint = true)
        {
            DrawConcentratedForce(force.LocalFrame.Origin, force.Value, display, color, scale, pointsToApplicationPoint);
        }
        public static void DrawConcentratedForce(MPoint applicationPoint, MVector vector, DisplayPipeline display, Color color, double scale = 1.0, bool pointsToApplicationPoint = true)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.CVectorArrowHeadSize;
            int thickness = Settings.Default.CVectorLineThickness;

            MVector scaledVector = scale * vector;
            
            double value;
            MPoint fromPoint, toPoint;

            if (pointsToApplicationPoint)
            {
                value = - vector.Length();
                fromPoint = applicationPoint + scaledVector;
                toPoint = applicationPoint;
            }
            else
            {
                value = vector.Length();
                fromPoint = applicationPoint;
                toPoint = applicationPoint + scaledVector;
            }

            DrawForceArrow(fromPoint, toPoint, display, color, arrowSize, thickness);
        }
        private static void DrawForceArrow(MPoint fromPoint, MPoint toPoint, DisplayPipeline display, Color color, double arrowSize = 0.15, int thickness = 1)
        {
            var startPoint = fromPoint.Cast();
            var endPoint = toPoint.Cast();
            var line = new Line(startPoint, endPoint);

            display.DrawLine(line, color);
            
            if (arrowSize != 0)
            {
                display.DrawLineArrow(line, color, thickness, arrowSize);
               // display.DrawMarker(endPoint, line.Direction, color, thickness, arrowSize);
            }
        }

        public static void DrawConcentratedMoment(CMoment moment, DisplayPipeline display, Color color, double scale = 1.0, bool pointsToApplicationPoint = true)
        {
            DrawConcentratedMoment(moment.LocalFrame.Origin, moment.Value, display, color, scale, pointsToApplicationPoint);
        }
        public static void DrawConcentratedMoment(MPoint applicationPoint, MVector vector, DisplayPipeline display, Color color, double scale = 1.0, bool pointsToApplicationPoint = true)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.CVectorArrowHeadSize;
            int thickness = Settings.Default.CVectorLineThickness;

            MVector scaledVector = scale * vector;
            
            double value;
            MPoint fromPoint, toPoint;

            if (pointsToApplicationPoint)
            {
                value = -vector.Length();
                fromPoint = applicationPoint + scaledVector;
                toPoint = applicationPoint;
            }
            else
            {
                value = vector.Length();
                fromPoint = applicationPoint;
                toPoint = applicationPoint + scaledVector;
            }

            DrawMomentArrow(fromPoint, toPoint, display, color, arrowSize, thickness);
        }
        private static void DrawMomentArrow(MPoint fromPoint, MPoint toPoint, DisplayPipeline display, Color color, double arrowSize = 0.15, int thickness = 1)
        {
            var startPoint = fromPoint.Cast();
            var endPoint = toPoint.Cast();
            var line = new Line(startPoint, endPoint);

            display.DrawLine(line, color);

            if (arrowSize != 0)
            {
                display.DrawMarker(startPoint + 0.9 * line.Direction, line.Direction, color, thickness, arrowSize);
                display.DrawMarker(endPoint, line.Direction, color);
            }
        }


    }
}
