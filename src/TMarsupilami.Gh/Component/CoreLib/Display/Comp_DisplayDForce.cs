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
    public class Comp_DisplayDForce : GH_Component
    {

        private List<GH_MDForce> ghForces;
        private bool isProjected, isGlobal;
        private double scale;
        private bool isNull;

        public Comp_DisplayDForce()
          : base("Force Display - Distributed (f)", "f Disp",
              "Preview a distributed force in the viewport.",
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
                return Resources.DisplayDForce;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{E3AEFE52-2F2B-47DF-ADB6-58F9DE87E8CE}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MDForce(), "Distributed Force", "f", "Distributed force to preview.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Project", "P", "Draw as a single vector (False) or draw each component in the appropriate coordinate system (True).", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Coordinate System", "CS", "The coordinate system to draw the force components ; either Global (True) or Local (False).", GH_ParamAccess.item, true);
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
            ghForces = new List<GH_MDForce>();
            isProjected = false;
            isGlobal = true;
            scale = 1;

            if (DA.GetDataList(0, ghForces)){ isNull = false; }

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

                    Color color = Attributes.GetTopLevel.Selected ? args.WireColour_Selected : Settings.Default.DForceColor;

                foreach (var ghForce in ghForces)
                {
                    var force = ghForce.Value;
                    var applicationPoint = force.LocalFrame.Origin;

                    if (!isProjected)
                    {
                        var f = force.Value;
                        if (f.Length() > minValue)
                            Draw.DrawDistributedForce(force.StartPoint, force.EndPoint,f, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                    }
                    else
                    {
                        MVector f1, f2, f3;
                        force.GetComponents(out f1, out f2, out f3, isGlobal);

                        if (f1.Length() > minValue)
                            Draw.DrawDistributedForce(force.StartPoint, force.EndPoint, f1, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                        if (f2.Length() > minValue)
                            Draw.DrawDistributedForce(force.StartPoint, force.EndPoint, f2, args.Display, color, scale, pointsToApplicationPoint, refineCount);
                        if (f3.Length() > minValue)
                        {
                            MVector perpDir = force.LocalFrame.XAxis + force.LocalFrame.YAxis;
                            Draw.DrawAxialDistributedForce(force.StartPoint, force.EndPoint, f3, perpDir, args.Display, color, scale, pointsToApplicationPoint, refineCount);
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
