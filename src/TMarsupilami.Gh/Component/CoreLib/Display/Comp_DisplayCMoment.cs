﻿using System;
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
using TMarsupilami.CoreLib3;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayCMoment : GH_Component
    {

        private List<GH_MCMoment> ghMoments;
        private bool isProjected, isGlobal;
        private double scale;
        private bool isNull;

        public Comp_DisplayCMoment()
          : base("Moment Display - Concentrated (M)", "M Disp",
              "Preview a concentrated moment in the viewport.",
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
                return Resources.DisplayCMoment;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{209EF586-ADF6-4DEC-BB2E-27A79E9944E7}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MCMoment(), "Concentrated Moment", "M", "Concentrated moment to preview.", GH_ParamAccess.list);
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
            ghMoments = new List<GH_MCMoment>();
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

                Color color = Attributes.GetTopLevel.Selected ? args.WireColour_Selected : Settings.Default.CMomentColor;

                foreach (var ghMoment in ghMoments)
                {
                    var moment = ghMoment.Value;
                    var applicationPoint = moment.LocalFrame.Origin;

                    if (!isProjected)
                    {
                        MVector M = moment.Value;
                        if (M.Length() > minValue)
                            Draw.DrawConcentratedMoment(applicationPoint, M, args.Display, color, scale, pointsToApplicationPoint);
                    }
                    else
                    {
                        MVector M1, M2, M3;
                        moment.GetComponents(out M1, out M2, out M3, isGlobal);
                        if (M1.Length() > minValue)
                            Draw.DrawConcentratedMoment(applicationPoint, M1, args.Display, color, scale, pointsToApplicationPoint);
                        if (M2.Length() > minValue)
                            Draw.DrawConcentratedMoment(applicationPoint, M2, args.Display, color, scale, pointsToApplicationPoint);
                        if (M3.Length() > minValue)
                            Draw.DrawConcentratedMoment(applicationPoint, M3, args.Display, color, scale, pointsToApplicationPoint);
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