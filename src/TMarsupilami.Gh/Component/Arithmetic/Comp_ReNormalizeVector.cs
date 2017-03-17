using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ReNormalizeVector : GH_Component
    {

        public Comp_ReNormalizeVector()
          : base("Re-normalize a Vector", "ReNorm",
              "Fast normalization of a vector that is already almost normalized.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.senary;
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
            get { return new Guid("{4EC8C087-096B-4B9E-9CB6-5B9814DBF293}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector(s)", "v", "Vectors(s) to re-normalize.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector(s)", "v", "The re-normalized vector(s).", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var vector_list = new List<Vector3d>();

            if (!DA.GetDataList(0, vector_list)) { return; }

            var vectors = vector_list.Cast();

            var watch = Stopwatch.StartNew();
            for (int i = 0; i < vectors.Count; i++)
            {
                vectors[i] = MVector.ReNormalize(vectors[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time = " + watch.ElapsedMilliseconds + " ms");

            DA.SetDataList(0, vectors.Cast());
        }
    }
}
