using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class Comp_NormalizeVector : GH_Component
    {

        public Comp_NormalizeVector()
          : base("Normalize a Vector", "Norm",
              "Normalize a vector.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.quinary;
            }
        }
        //protected override System.Drawing.Bitmap Icon
        //{
        //    get
        //    {
        //        return Resources.ZRotatePlane_Diff;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{71B96FAC-C3CF-4C7C-86D6-A534045CB872}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector(s)", "v", "Vectors(s) to normalize.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector(s)", "v", "The normalized vector(s).", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var vector_list = new List<Vector3d>();

            if (!DA.GetDataList(0, vector_list)) { return; }

            var vectors = vector_list.Cast();

            var watch = Stopwatch.StartNew();
            for (int i = 0; i < vectors.Count; i++)
            {
                vectors[i] = MVector.Normalize(vectors[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + (watch.Elapsed.TotalMilliseconds*1000) + " us");

            DA.SetDataList(0, vectors.Cast());
        }
    }
}
