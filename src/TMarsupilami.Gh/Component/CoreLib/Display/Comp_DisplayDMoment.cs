using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using System.Drawing;
using GH_IO.Types;
using Grasshopper;
using TMarsupilami.Gh.Type;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayDMoment : GH_Component
    {

        private List<GH_MDMoment> ghMoments;
        private bool isProjected, isGlobal;
        private double scale;
        private bool isNull;

        public Comp_DisplayDMoment()
          : base("Moment Display - Distributed (m)", "m Disp",
              "Preview a distributed moment in the viewport.",
              "TMarsupilami", "Display")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.secondary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.DisplayDMoment;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{0FCF9A1E-2F38-4558-8554-8035AA8206DA}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MDMoment(), "Distributed Moment", "m", "Distributed moment to preview.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Project", "P", "Draw as a single vector (False) or draw each component in the appropriate coordinate system (True).", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Coordinate System", "CS", "The coordinate system to draw the moment components ; either Global (True) or Local (False).", GH_ParamAccess.item, true);
            pManager.AddNumberParameter("Scale", "S", "Scale factor.", GH_ParamAccess.item, 1);
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
            ghMoments = new List<GH_MDMoment>();
            isProjected = false;
            isGlobal = true;
            scale = 1;

            if (DA.GetDataList(0, ghMoments)){ isNull = false; }

            // OPTIONAL WITH DEFAULT
            DA.GetData(1, ref isProjected);
            DA.GetData(2, ref isGlobal);
            DA.GetData(3, ref scale);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (!isNull)
            {
                var minValue = 100;
                bool pointsToApplicationPoint = false;
                int refineCount = 4;

                Color color = Attributes.GetTopLevel.Selected ? args.WireColour_Selected : Settings.Default.DMomentColor;

                foreach (var ghMoment in ghMoments)
                {
                    var moment = ghMoment.Value;
                    var applicationPoint = moment.LocalFrame.Origin;

                    if (!isProjected)
                    {
                        var m = moment.Value;
                        if (m.Length() > minValue)
                            Draw.DrawDistributedMoment(moment.StartPoint, moment.EndPoint,m, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                    }
                    else
                    {
                        MVector m1, m2, m3;
                        moment.GetComponents(out m1, out m2, out m3, isGlobal);

                        if (m1.Length() > minValue)
                            Draw.DrawDistributedMoment(moment.StartPoint, moment.EndPoint, m1, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                        if (m2.Length() > minValue)
                            Draw.DrawDistributedMoment(moment.StartPoint, moment.EndPoint, m2, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                        if (m3.Length() > minValue)
                        {
                            MVector perpDir = moment.LocalFrame.XAxis + moment.LocalFrame.YAxis;
                            Draw.DrawAxialDistributedMoment(moment.StartPoint, moment.EndPoint, m3, perpDir, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                        }
                    }
                }
            }
        }
        public override bool IsPreviewCapable
        {
            get
            {
                return true;
            }
        }
    }
}
