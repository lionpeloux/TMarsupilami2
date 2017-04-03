using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    public class BeamLayout : IBeamLayout
    {
        // TOPOLOGY
        public int Nv { get; protected set; }
        public int Ne { get; protected set; }
        public bool IsClosed { get; protected set; }

        // APPLIED EXTERNAL FORCES AND MOMENTS
        public BeamLoadManager LoadManager { get; protected set; }

        // GEOMETRIC CONFIGURATION
        public MFrame[] ActualConfiguration { get; protected set; }
        public MFrame[] RestConfiguration { get; protected set; }

        // DYNAMIC CONFIGURATION
        public double[] LMx { get; protected set; }
        public double[] LMθ { get; protected set; }
        public MVector[] Vx { get; protected set; }
        public MVector[] Vθ { get; protected set; }
        public MVector[] Ax { get; protected set; }
        public MVector[] Aθ { get; protected set; }

        public bool IsEdgeIndexValid(int edgeIndex, bool throwError = false)
        {
            if (edgeIndex < 0 || edgeIndex >= Ne)
            {
                if (throwError)
                    throw new ArgumentException("The edge index must be set according to the structure of the BeamLayout");
                return false;
            }
            return true;
        }
        public bool IsVertexIndexValid(int vertexIndex, bool throwError = false)
        {
            if (vertexIndex < 0 || vertexIndex >= Nv)
            {
                if(throwError)
                    throw new ArgumentException("The vertex index must be set according to the structure of the BeamLayout");
                return false;
            }
            return true;
        }

        public MVector ConvertToMaterialCS(MVector globalVector, int index, bool isEdgeQuantity = false)
        {
            throw new NotImplementedException();
        }
        public MVector ConvertToGlobalCS(MVector materialVector, int index, bool isEdgeQuantity = false)
        {
            throw new NotImplementedException();
        }

        public BeamLayout(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> intialFrames, bool isClosed = false)
        {
            ActualConfiguration = intialFrames.ToArray();
            RestConfiguration = restFrames.ToArray();
            IsClosed = isClosed;
            Nv = ActualConfiguration.Length;
            Ne = (IsClosed) ? Nv : Nv - 1;

            LMx = new double[Nv];
            LMθ = new double[Nv];
            Vx = new MVector[Nv];
            Vθ = new MVector[Nv];
            Ax = new MVector[Nv];
            Aθ = new MVector[Nv];

            if (ActualConfiguration.Length != RestConfiguration.Length)
                throw new ArgumentException("restFrames and initialFrames must have the same number of frames.");
        }
    }
}
