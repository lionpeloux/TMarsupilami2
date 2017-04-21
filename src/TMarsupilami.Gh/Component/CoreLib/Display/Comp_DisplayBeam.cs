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
    public class Comp_DisplayBeam : GH_Component
    {

        public Comp_DisplayBeam()
          : base("Beam Internals", "BDis",
              "Extract Beam Internals Informations",
              "TMarsupilami", "Display")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.primary;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{80D808A8-9980-4392-AA89-FA8E052782B0}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MBeam(), "Beam", "B", "The beam to preview.", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The material frames of the beam.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Frames Mid", "Fmid", "The material frames of the beam.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Fext (G)", "Fext", "The applied force.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Mext (L)", "Mext", "The applied moment.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MDForce(), "fext (G)", "fext", "The applied force.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MDMoment(), "mext (L)", "mext", "The applied moment.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Fr (G)", "Fr", "The applied force reaction.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Mr (L)", "Mr", "The applied moment reaction.", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();

            if (!DA.GetData(0, ref ghBeam)){ return; }

            var beam = ghBeam.Value as Beam_4DOF_D;

            var Fext = new CForce[beam.Fext_g.Length];
            for (int i = 0; i < Fext.Length; i++)
            {
                var F = beam.Fext_g[i];
                Fext[i] = new CForce(F, beam.ActualConfiguration[2 * i]);
            }

            var Mext = new CMoment[beam.Mext_m.Length];
            for (int i = 0; i < Mext.Length; i++)
            {
                var M = beam.ToGlobalCoordinateSystem(beam.Mext_m[i], 2 * i);
                Mext[i] = new CMoment(M, beam.ActualConfiguration[2 * i]);
            }

            var fext = new DForce[beam.fext_g.Length];
            for (int i = 0; i < fext.Length; i++)
            {
                var f = beam.fext_g[i];
                fext[i] = new DForce(f, beam.ActualConfiguration[2 * i].Origin, beam.ActualConfiguration[2 * i + 2].Origin, beam.ActualConfiguration[2 * i]);
            }

            var mext = new DMoment[beam.mext_m.Length];
            for (int i = 0; i < mext.Length; i++)
            {
                var m = beam.ToGlobalCoordinateSystem(beam.mext_m[i], 2 * i);
                mext[i] = new DMoment(m, beam.ActualConfiguration[2 * i].Origin, beam.ActualConfiguration[2 * i + 2].Origin, beam.ActualConfiguration[2 * i]);
            }

            var Fr = new CForce[beam.Fr_g.Length];
            for (int i = 0; i < Fr.Length; i++)
            {
                var F = beam.Fr_g[i];
                Fr[i] = new CForce(F, beam.ActualConfiguration[2 * i]);
            }

            var Mr = new CMoment[beam.Mr_m.Length];
            for (int i = 0; i < Mr.Length; i++)
            {
                var M = beam.ToGlobalCoordinateSystem(beam.Mr_m[i], 2 * i);
                Mr[i] = new CMoment(M, beam.ActualConfiguration[2 * i]);
            }


            DA.SetDataList(0, beam.Get_MaterialFrames(Configuration.Actual));
            DA.SetDataList(1, beam.Get_MaterialFramesAtMid());
            DA.SetDataList(2, Fext);
            DA.SetDataList(3, Mext);
            DA.SetDataList(4, fext);
            DA.SetDataList(5, mext);
            DA.SetDataList(6, Fr);
            DA.SetDataList(7, Mr);

        }

    }
}
