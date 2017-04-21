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
        public static void DrawAxialDistributedForce(MPoint edgeStartPoint, MPoint edgeEndPoint, MVector vector, MVector perpDirection, DisplayPipeline display, Color color, double scale, bool pointsToApplicationPoint, int refineCount)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.DVectorArrowHeadSize;
            int thickness = Settings.Default.DVectorLineThickness;

            double value;
            Line[] arrowLines;
            Line valueLine;
            Point3d labelPoint;
            perpDirection.Normalize();

            var edge = new MVector(edgeStartPoint, edgeEndPoint);
            double sign = Math.Sign(vector * edge);
            MVector signedVector = sign * vector.Length() * perpDirection;

            GetDistributedArrowProperties(edgeStartPoint, edgeEndPoint, signedVector, out arrowLines, out valueLine, out value, out labelPoint, arrowSize, scale, pointsToApplicationPoint, refineCount);

            // DRAW ARROWS
            for (int i = 0; i < arrowLines.Length; i++)
            {
                display.DrawLine(arrowLines[i], color, thickness);
            }

            // DRAW VALUE LINE
            if (sign < 0)
                valueLine.Flip();

            Point3d midPoint = 0.5 * (valueLine.From + valueLine.To);
            DrawConcentratedArrowWithSingleHead(new Line(valueLine.From, midPoint), display, color, arrowSize, thickness);
            display.DrawLine(new Line(midPoint, valueLine.To), color, thickness);

            // DRAW LABEL
            string labelTxt = string.Format("{0:E2} N", value);
            DrawLabel(labelTxt, labelPoint, display, color, Settings.Default.LabelFontHeightInPixel, Settings.Default.LabelFontName);
        }
        public static void DrawAxialDistributedMoment(MPoint edgeStartPoint, MPoint edgeEndPoint, MVector vector, MVector perpDirection, DisplayPipeline display, Color color, double scale, bool pointsToApplicationPoint, int refineCount)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.DVectorArrowHeadSize;
            double arrowSpacing = Settings.Default.DoubleArrowSpacingRatio * arrowSize;
            int thickness = Settings.Default.DVectorLineThickness;

            double value;
            Line[] arrowLines;
            Line valueLine;
            Point3d labelPoint;
            perpDirection.Normalize();

            var edge = new MVector(edgeStartPoint, edgeEndPoint);
            double sign = Math.Sign(vector * edge);
            MVector signedVector = sign * vector.Length() * perpDirection;

            GetDistributedArrowProperties(edgeStartPoint, edgeEndPoint, signedVector, out arrowLines, out valueLine, out value, out labelPoint, arrowSize, scale, pointsToApplicationPoint, refineCount);

            // DRAW ARROWS
            for (int i = 0; i < arrowLines.Length; i++)
            {
                display.DrawLine(arrowLines[i], color, thickness);
            }

            // DRAW VALUE LINE
            if (sign < 0)
                valueLine.Flip();

            Point3d midPoint = 0.5 * (valueLine.From + valueLine.To);
            DrawConcentratedArrowWithDoubleHead(new Line(valueLine.From, midPoint), display, color, arrowSpacing, arrowSize, thickness);
            display.DrawLine(new Line(midPoint, valueLine.To), color, thickness);

            // DRAW LABEL
            string labelTxt = string.Format("{0:E2} N", value);
            DrawLabel(labelTxt, labelPoint, display, color, Settings.Default.LabelFontHeightInPixel, Settings.Default.LabelFontName);
        }

        public static void DrawDistributedForce(MPoint edgeStartPoint, MPoint edgeEndPoint, MVector vector, DisplayPipeline display, Color color, double scale, bool pointsToApplicationPoint, int refineCount)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.DVectorArrowHeadSize;
            int thickness = Settings.Default.DVectorLineThickness;

            double value;
            Line[] arrowLines;
            Line valueLine;
            Point3d labelPoint;

            GetDistributedArrowProperties(edgeStartPoint, edgeEndPoint, vector, out arrowLines, out valueLine, out value, out labelPoint, arrowSize, scale, pointsToApplicationPoint, refineCount);

            // DRAW ARROWS
            for (int i = 0; i < arrowLines.Length; i++)
            {
                DrawConcentratedArrowWithSingleHead(arrowLines[i], display, color, arrowSize, thickness);
            }

            // DRAW VALUE LINE
            display.DrawLine(valueLine, color, thickness);

            // DRAW LABEL
            string labelTxt = string.Format("{0:E2} N", value);
            DrawLabel(labelTxt, labelPoint, display, color, Settings.Default.LabelFontHeightInPixel, Settings.Default.LabelFontName);
        }
        public static void DrawDistributedMoment(MPoint edgeStartPoint, MPoint edgeEndPoint, MVector vector, DisplayPipeline display, Color color, double scale, bool pointsToApplicationPoint, int refineCount)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.DVectorArrowHeadSize;
            double arrowSpacing = Settings.Default.DoubleArrowSpacingRatio * arrowSize;
            int thickness = Settings.Default.DVectorLineThickness;

            double value;
            Line[] arrowLines;
            Line valueLine;
            Point3d labelPoint;

            GetDistributedArrowProperties(edgeStartPoint, edgeEndPoint, vector, out arrowLines, out valueLine, out value, out labelPoint, arrowSize, scale, pointsToApplicationPoint, refineCount);

            // DRAW ARROWS
            for (int i = 0; i < arrowLines.Length; i++)
            {
                DrawConcentratedArrowWithDoubleHead(arrowLines[i], display, color, arrowSpacing, arrowSize, thickness);
            }

            // DRAW VALUE LINE
            display.DrawLine(valueLine, color, thickness);

            // DRAW LABEL
            string labelTxt = string.Format("{0:E2} Nm", value);
            DrawLabel(labelTxt, labelPoint, display, color, Settings.Default.LabelFontHeightInPixel, Settings.Default.LabelFontName);
        }

        public static void DrawConcentratedForce(MPoint applicationPoint, MVector vector, DisplayPipeline display, Color color, double scale, bool pointsToApplicationPoint)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.CVectorArrowHeadSize;
            int thickness = Settings.Default.CVectorLineThickness;

            Line arrowLine;
            Point3d labelPoint;
            GetConcentratedArrowProperties(applicationPoint, vector, out arrowLine, out labelPoint, arrowSize, scale, pointsToApplicationPoint);

            string labelTxt = string.Format("{0:E2} N", vector.Length());

            DrawConcentratedArrowWithSingleHead(arrowLine, display, color, arrowSize, thickness);
            DrawLabel(labelTxt, labelPoint, display, color, Settings.Default.LabelFontHeightInPixel, Settings.Default.LabelFontName);
        }
        public static void DrawConcentratedMoment(MPoint applicationPoint, MVector vector, DisplayPipeline display, Color color, double scale, bool pointsToApplicationPoint)
        {
            double arrowSize = CentralSettings.PreviewPlaneRadius * Settings.Default.CVectorArrowHeadSize;
            double arrowSpacing = Settings.Default.DoubleArrowSpacingRatio * arrowSize;
            int thickness = Settings.Default.CVectorLineThickness;

            Line arrowLine;
            Point3d labelPoint;
            GetConcentratedArrowProperties(applicationPoint, vector, out arrowLine, out labelPoint, arrowSize, scale, pointsToApplicationPoint);

            string labelTxt = string.Format("{0:E2} Nm", vector.Length());

            DrawConcentratedArrowWithDoubleHead(arrowLine, display, color, arrowSpacing, arrowSize, thickness);
            DrawLabel(labelTxt, labelPoint, display, color, Settings.Default.LabelFontHeightInPixel, Settings.Default.LabelFontName);
        }

        private static void DrawConcentratedArrowWithSingleHead(Line arrowLine, DisplayPipeline display, Color color, double arrowSize, int thickness = 1)
        {
            if (arrowSize != 0)
            {
                display.DrawLineArrow(arrowLine, color, thickness, arrowSize);
            }
            else
            {
                display.DrawLine(arrowLine, color);
            }
        }
        private static void DrawConcentratedArrowWithDoubleHead(Line arrowLine, DisplayPipeline display, Color color, double arrowSpacing, double arrowSize, int thickness = 1)
        {
            var enPoint2 = arrowLine.To - (arrowSpacing / arrowLine.Length) * arrowLine.Direction;
            var arrowLine2 = new Line(arrowLine.From, enPoint2);

            if (arrowSize != 0)
            {
                display.DrawLineArrow(arrowLine, color, thickness, arrowSize);
                display.DrawLineArrow(arrowLine2, color, thickness, arrowSize);
            }
            else
            {
                display.DrawLine(arrowLine, color);
            }
        }

        private static void GetConcentratedArrowProperties(MPoint applicationPoint, MVector vector, out Line arrowLine, out Point3d labelPoint, double labelAbsoluteSpacing, double scale = 1.0, bool pointsToApplicationPoint = true)
        {
            MVector scaledVector = scale * vector;
            double length = vector.Length();

            MPoint fromPoint, toPoint;

            if (pointsToApplicationPoint)
            {
                fromPoint = applicationPoint - scaledVector;
                toPoint = applicationPoint;
                labelPoint = (fromPoint - (labelAbsoluteSpacing / length) * vector).Cast();
            }
            else
            {
                fromPoint = applicationPoint;
                toPoint = applicationPoint + scaledVector;
                labelPoint = (toPoint + (labelAbsoluteSpacing / length) * vector).Cast();
            }

            arrowLine = new Line(fromPoint.Cast(), toPoint.Cast());
        }

        private static void GetDistributedArrowProperties(MPoint startApplicationPoint, MVector startVector, MPoint endApplicationPoint, MVector endVector,
                                                            out Line[] arrowLines, out Line valueLine, out double[] values, out Point3d[] labelPoints,
                                                            double labelAbsoluteSpacing, double scale = 1.0, bool pointsToApplicationPoint = true, int refineCount = 0)
        {
            Point3d fromPoint, toPoint, labelPoint;
            int n = 2 + refineCount;

            arrowLines = new Line[n];
            values = new double[n];
            labelPoints = new Point3d[n];

            // ADD VECTOR LINES
            Point3d ps = startApplicationPoint.Cast();
            Point3d pe = endApplicationPoint.Cast();
            Vector3d tp = new Vector3d(pe - ps);

            Vector3d vs = startVector.Cast();
            Vector3d ve = endVector.Cast();
            Vector3d tv = ve - vs;

            for (int i = 0; i < refineCount + 2; i++)
            {
                var param = (double)i / (refineCount + 1);
                var applicationPoint = ps + param * tp;
                var vector = vs + param * tv;
                var scaledVector = scale * vector;
                var value = vector.Length;

                if (pointsToApplicationPoint)
                {
                    fromPoint = applicationPoint - scaledVector;
                    toPoint = applicationPoint;
                    labelPoint = applicationPoint - (1 + labelAbsoluteSpacing / value) * vector;
                }
                else
                {
                    fromPoint = applicationPoint;
                    toPoint = applicationPoint + scaledVector;
                    labelPoint = applicationPoint + (1 + labelAbsoluteSpacing / value) * vector;
                }

                arrowLines[i] = new Line(fromPoint, toPoint);
                values[i] = value;
                labelPoints[i] = labelPoint;
            }

            // ADD VALUE LINE
            if (pointsToApplicationPoint)
            {
                valueLine = new Line(arrowLines[0].From, arrowLines[n - 1].From);
            }
            else
            {
                valueLine = new Line(arrowLines[0].To, arrowLines[n - 1].To);
            }
        }

        private static void GetDistributedArrowProperties(MPoint startApplicationPoint, MPoint endApplicationPoint, MVector vector,
                                                            out Line[] arrowLines, out Line valueLine, out double value, out Point3d midLabelPoint,
                                                            double labelAbsoluteSpacing, double scale = 1.0, bool pointsToApplicationPoint = true, int refineCount = 0)
        {
            Point3d fromPoint, toPoint;
            int n = 2 + refineCount;
            arrowLines = new Line[n];

            Vector3d scaledVector = (scale * vector).Cast();
            value = vector.Length();

            // ADD VECTOR LINES
            Point3d ps = startApplicationPoint.Cast();
            Point3d pe = endApplicationPoint.Cast();
            Vector3d e = new Vector3d(pe - ps);
            for (int i = 0; i < refineCount + 2; i++)
            {
                var param = (double)i / (refineCount + 1);
                var applicationPoint = ps + param * e;

                if (pointsToApplicationPoint)
                {
                    fromPoint = applicationPoint - scaledVector;
                    toPoint = applicationPoint;
                }
                else
                {
                    fromPoint = applicationPoint;
                    toPoint = applicationPoint + scaledVector;
                }
                arrowLines[i] = new Line(fromPoint, toPoint);
            }

            // ADD VALUE LINE
            if (pointsToApplicationPoint)
            {
                valueLine = new Line(arrowLines[0].From, arrowLines[n-1].From);
                midLabelPoint = ps + 0.5 * e - (1 + labelAbsoluteSpacing / scaledVector.Length) * scaledVector;
            }
            else
            {
                valueLine = new Line(arrowLines[0].To, arrowLines[n-1].To);
                midLabelPoint = ps + 0.5 * e + (1 + labelAbsoluteSpacing / scaledVector.Length) * scaledVector;
            }
        }

        private static void DrawLabel(string labelTxt, Point3d point, DisplayPipeline display, Color color, int fontHeightInPixel = 12, string fontFace = "Arial")
        {
            display.Draw2dText(labelTxt, color, point, true, fontHeightInPixel, fontFace);
        }

    }

}
