using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using System.Drawing;
using GH_IO.Types;
using Grasshopper;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Type;
using TMarsupilami.CoreLib3;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayBeam_Energy : GH_Component
    {

        public Comp_DisplayBeam_Energy()
              : base("Elastic Energy (E)", "Energy",
                  "Extract Beam Internals Informations",
                  "TMarsupilami", "Display")
            {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.tertiary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.DisplayBeam_E;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{4B20D058-B9E2-49AF-AE84-C7FAEC52CA6A}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MBeam(), "Beam", "B", "The beam to preview.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scale", "S", "Scale", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("Configuration", "C", "Rest (0), Initial (1), Actual (2).", GH_ParamAccess.item, 2);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Axial Energy", "Ea", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Bending Energy", "Eb", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Twisting Energy", "Et", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (Ea)", "Da", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (Eb)", "Db", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (Et)", "Dt", "", GH_ParamAccess.list);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();
            var scale = 1.0;
            var configIndex = 2;

            if (!DA.GetData(0, ref ghBeam)) { return; }

            DA.GetData(1, ref scale);
            DA.GetData(2, ref configIndex);

            var beam = ghBeam.Value as Beam_4DOF_D;

            double[] Ea, Eb, Et;
            Configuration config;
            MPoint[] startPoints, endPoints;

            switch (configIndex)
            {
                case 0:
                    config = Configuration.Rest;
                    break;
                case 1:
                    config = Configuration.Initial;
                    break;
                default:
                    config = Configuration.Actual;
                    break;
            }

            beam.Get_Ea(out Ea);
            beam.Diagram_Energy(Ea, out startPoints, out endPoints, scale, config, Axis.d1);
            var Da = Diagram.GetOutlines(startPoints, endPoints);

            beam.Get_Eb(out Eb);
            beam.Diagram_Energy(Eb, out startPoints, out endPoints, scale, config, Axis.d1);
            var Db = Diagram.GetOutlines(startPoints, endPoints);

            beam.Get_Et(out Et);
            beam.Diagram_Energy(Et, out startPoints, out endPoints, scale, config, Axis.d1);
            var Dt = Diagram.GetOutlines(startPoints, endPoints);

            DA.SetDataList(0, Ea);
            DA.SetDataList(1, Eb);
            DA.SetDataList(2, Et);
            DA.SetDataList(3, Da);
            DA.SetDataList(4, Db);
            DA.SetDataList(5, Dt);

        }

    }
}
