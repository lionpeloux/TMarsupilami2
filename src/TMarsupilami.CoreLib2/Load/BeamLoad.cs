using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
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
        public IBeamLayout Layout { get; protected set; }   // a load is applied to only one layout
        public MVector[] Value { get; protected set; }      // internal representation of the load

        protected BeamVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, IBeamLayout layout, int dimension, bool isStatic, bool isUniform, bool isGlobal) 
            : base(dimension, isStatic, isUniform, isGlobal)
        {
            Value = value;
            Type = type;
            IsDistributed = isDistributed;
            Layout = layout;
        }

        public MVector GetValueAt(int index, CoordinateSystem cs = CoordinateSystem.Global)
        {
            if (cs == CoordinateSystem.Global)
            {
                if (IsGlobal)
                {
                    return Value[index];
                }
                else
                {
                    return Layout.ConvertToGlobalCS(Value[index], index, IsDistributed);
                }
            }
            else
            {
                if (IsGlobal)
                {
                    return Layout.ConvertToMaterialCS(Value[index], index, IsDistributed);
                }
                else
                {
                    return Value[index];
                }
            }

        }

        #region FACTORY
        // HELPERS
        private static BeamVectorLoad CreateBeamVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, IBeamLayout layout, int dimension, bool isUniform, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            if (del == null)    // this is a static load
                return new BeamStaticVectorLoad(value, VectorLoadType.Force, isDistributed, layout, dimension, isUniform, isGlobal);
            else                // this is a dynamic load
                return new BeamDynamicVectorLoad(value, VectorLoadType.Force, isDistributed, layout, dimension, isUniform, isGlobal, del);
        }

        // SINGLE LOAD
        public static BeamVectorLoad Create_Fext(MVector Fext, int vertexIndex, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            layout.IsVertexIndexValid(vertexIndex, true);

            var value = new MVector[layout.Nv];
            value[vertexIndex] = Fext;
            return CreateBeamVectorLoad(value, VectorLoadType.Force, false, layout, 0, false, isGlobal, del);      
        }
        public static BeamVectorLoad Create_Mext(MVector Mext, int vertexIndex, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            layout.IsVertexIndexValid(vertexIndex, true);

            var value = new MVector[layout.Nv];
            value[vertexIndex] = Mext;
            return new BeamDynamicVectorLoad(value, VectorLoadType.Moment, false, layout, 0, false, isGlobal, del);
        }
        public static BeamVectorLoad Create_fext(MVector fext, int edgeIndex, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            layout.IsEdgeIndexValid(edgeIndex, true);

            var value = new MVector[layout.Ne];
            value[edgeIndex] = fext;
            return CreateBeamVectorLoad(value, VectorLoadType.Force, true, layout, 0, false, isGlobal, del);
        }
        public static BeamVectorLoad Create_mext(MVector mext, int edgeIndex, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            layout.IsEdgeIndexValid(edgeIndex, true);

            var value = new MVector[layout.Ne];
            value[edgeIndex] = mext;
            return new BeamDynamicVectorLoad(value, VectorLoadType.Moment, true, layout, 0, false, isGlobal, del);
        }

        // UNIFORM LOAD
        public static BeamVectorLoad Create_Fext(MVector Fext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Nv];
            MVector.Add(Fext, ref value);
            return CreateBeamVectorLoad(value, VectorLoadType.Force, false, layout, 0, true, isGlobal, del);
        }
        public static BeamVectorLoad Create_Mext(MVector Mext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Nv];
            MVector.Add(Mext, ref value);
            return new BeamDynamicVectorLoad(value, VectorLoadType.Moment, false, layout, 0, true, isGlobal, del);
        }
        public static BeamVectorLoad Create_fext(MVector fext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Ne];
            MVector.Add(fext, ref value);
            return CreateBeamVectorLoad(value, VectorLoadType.Force, true, layout, 0, true, isGlobal, del);
        }
        public static BeamVectorLoad Create_mext(MVector mext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Ne];
            MVector.Add(mext, ref value);
            return new BeamDynamicVectorLoad(value, VectorLoadType.Moment, true, layout, 0, true, isGlobal, del);
        }

        // VARIABLE LOAD
        public static BeamVectorLoad Create_Fext(MVector[] Fext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Nv];
            MVector.Add(Fext, ref value);
            return CreateBeamVectorLoad(value, VectorLoadType.Force, false, layout, 0, false, isGlobal, del);
        }
        public static BeamVectorLoad Create_Mext(MVector[] Mext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Nv];
            MVector.Add(Mext, ref value);
            return new BeamDynamicVectorLoad(value, VectorLoadType.Moment, false, layout, 0, false, isGlobal, del);
        }
        public static BeamVectorLoad Create_fext(MVector[] fext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Ne];
            MVector.Add(fext, ref value);
            return CreateBeamVectorLoad(value, VectorLoadType.Force, true, layout, 0, false, isGlobal, del);
        }
        public static BeamVectorLoad Create_mext(MVector[] mext, IBeamLayout layout, bool isGlobal, DynamicBeamVectorLoadUpdate del = null)
        {
            var value = new MVector[layout.Ne];
            MVector.Add(mext, ref value);
            return new BeamDynamicVectorLoad(value, VectorLoadType.Moment, true, layout, 0, false, isGlobal, del);
        }
        #endregion
    }
    public class BeamStaticVectorLoad : BeamVectorLoad
    {
        internal BeamStaticVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, IBeamLayout layout, int dimension, bool isUniform, bool isGlobal)
            : base(value, type, isDistributed, layout, dimension, true, isUniform, isGlobal)
        {
        }
    }
    public class BeamDynamicVectorLoad : BeamVectorLoad, IDynamicLoad
    {
        public DynamicBeamVectorLoadUpdate TimeUpdate { get; set; }
        internal BeamDynamicVectorLoad(MVector[] value, VectorLoadType type, bool isDistributed, IBeamLayout layout, int dimension, bool isUniform, bool isGlobal, DynamicBeamVectorLoadUpdate del)
            : base(value, type, isDistributed, layout, dimension, false, isUniform, isGlobal)
        {
            TimeUpdate += del;
        }
    }

}
