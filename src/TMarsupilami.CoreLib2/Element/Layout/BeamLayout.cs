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
        public int Nv { get; protected set; }
        public int Ne { get; protected set; }
        public bool IsClosed { get; protected set; }
        public BeamLoadManager LoadManager { get; protected set; }

        public MFrame[] ActualConfiguration { get; protected set; }
        public MFrame[] RestConfiguration { get; protected set; }

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
    }
}
