using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{ 
    public enum VectorLoadType:int
    {
        Force,
        Moment,
    }

    public class BeamVectorLoad : Load
    {
        public VectorLoadType Type { get; protected set; }  // wether its a force or a moment vector load
        public bool IsDistributed { get; protected set; }   // wether the vector load is applied to vertices or edges
        public Beam Beam { get; protected set; }            // a load is applied to one beam element
        public MVector[] Value { get; protected set; }      // internal representation of the load

        protected BeamVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, Beam beam, int dimension, bool isStatic, bool isUniform, bool isGlobal) 
            : base(dimension, isStatic, isUniform, isGlobal)
        {
            Value = value;
            Type = type;
            IsDistributed = isDistributed;
            Beam = beam;
        }

        #region FACTORY
        // HELPERS
        private static BeamVectorLoad CreateBeamVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, Beam beam, int dimension, bool isUniform, bool isGlobal)
        {
            return new BeamStaticVectorLoad(value, type, isDistributed, beam, dimension, isUniform, isGlobal);
        }

        // SINGLE LOAD
        public static BeamVectorLoad Create_Fext(MVector Fext, int vertexIndex, Beam beam, bool isGlobal)
        {
            beam.IsVertexIndexValid(vertexIndex, true);

            var value = new MVector[beam.Nvh];
            value[vertexIndex] = Fext;
            return CreateBeamVectorLoad(value, VectorLoadType.Force, false, beam, 0, false, isGlobal);      
        }
        public static BeamVectorLoad Create_Fext(MVector Fext, Boundary boundary, Beam beam, bool isGlobal)
        {
            int vertexIndex = beam.BoundaryToVertexIndex(boundary);
            return Create_Fext(Fext, vertexIndex, beam, isGlobal);
        }

        public static BeamVectorLoad Create_Mext(MVector Mext, int vertexIndex, Beam beam, bool isGlobal)
        {
            beam.IsVertexIndexValid(vertexIndex, true);

            var value = new MVector[beam.Nvh];
            value[vertexIndex] = Mext;
            return CreateBeamVectorLoad(value, VectorLoadType.Moment, false, beam, 0, false, isGlobal);
        }
        public static BeamVectorLoad Create_Mext(MVector Mext, Boundary boundary, Beam beam, bool isGlobal)
        {
            int vertexIndex = beam.BoundaryToVertexIndex(boundary);
            return Create_Mext(Mext, vertexIndex, beam, isGlobal);
        }

        public static BeamVectorLoad Create_fext(MVector fext, int edgeIndex, Beam beam, bool isGlobal)
        {
            beam.IsEdgeIndexValid(edgeIndex, true);

            var value = new MVector[beam.Nvg];
            value[edgeIndex] = fext;
            return CreateBeamVectorLoad(value, VectorLoadType.Force, true, beam, 0, false, isGlobal);
        }
        public static BeamVectorLoad Create_fext(MVector fext, Boundary boundary, Beam beam, bool isGlobal)
        {
            int edgeIndex = beam.BoundaryToEdgeIndex(boundary);
            return Create_fext(fext, edgeIndex, beam, isGlobal);
        }

        public static BeamVectorLoad Create_mext(MVector mext, int edgeIndex, Beam beam, bool isGlobal)
        {
            beam.IsEdgeIndexValid(edgeIndex, true);

            var value = new MVector[beam.Nvg];
            value[edgeIndex] = mext;
            return CreateBeamVectorLoad(value, VectorLoadType.Moment, true, beam, 0, false, isGlobal);
        }
        public static BeamVectorLoad Create_mext(MVector mext, Boundary boundary, Beam beam, bool isGlobal)
        {
            int edgeIndex = beam.BoundaryToEdgeIndex(boundary);
            return Create_mext(mext, edgeIndex, beam, isGlobal);
        }

        #endregion
    }
    public class BeamStaticVectorLoad : BeamVectorLoad
    {
        internal BeamStaticVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, Beam beam, int dimension, bool isUniform, bool isGlobal)
            : base(value, type, isDistributed, beam, dimension, true, isUniform, isGlobal)
        {
        }
    }

}
