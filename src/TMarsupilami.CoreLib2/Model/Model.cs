using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    public sealed class Model
    {
        public List<ElementLayout> elements;
        public List<ConstraintLayout> constraints;

        public Model()
        {
            elements = new List<ElementLayout>();
            constraints = new List<ConstraintLayout>();
        }
 
        public int AddBeamLayout(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, IEnumerable<SectionProperty> sections, IEnumerable<MaterialProperty> materials, bool isClosed = false)
        {
            int id = elements.Count + 1;
            var layout = new BeamLayout(id, restFrames, actualFrames, sections, materials, isClosed);
            elements.Add(layout);
            return id;
        }
        public int AddBoundaryConstraint(int elementId, int elementVertexIndex, BoundaryConditionType type)
        {
            int id = constraints.Count + 1;
            var layout = new BoundaryConditionLayout(id, elementId, elementVertexIndex, type);
            constraints.Add(layout);
            return id;
        }
    }

    public enum ElementLayoutType
    {
        Spring      = 000,
        Link        = 010,

        Tie         = 100,
        Strut       = 110,
        Bar         = 120,
        Beam        = 130,

        Membrane    = 200,
        Plate       = 210,
        Shell       = 220,
    }
    public abstract class ElementLayout
    {
        public int Id { get; protected set; }
        public int Dimension { get; protected set; }
        public ElementLayoutType Type { get; protected set; }

        protected ElementLayout(ElementLayoutType layoutType, int id, int dimension)
        {
            Id = id;
            Dimension = dimension;
            Type = layoutType;
        }
    }
    public abstract class Element1DLayout : ElementLayout
    {
        // TOPOLOGY
        public int Nv { get; protected set; }
        public int Ne { get; protected set; }
        public bool IsClosed { get; protected set; }

        // MECHANICAL CAPABILITIES
        public bool IsCompressionCapable { get; protected set; }
        public bool IsTractionCapable { get; protected set; }
        public bool IsShearCapable { get; protected set; }
        public bool IsBendingCapable { get; protected set; }
        public bool IsTorsionCapable { get; protected set; }
  
        // SECTIONS
        public SectionProperty[] Sections { get; protected set; }
        public bool HasUniformSection { get { return Sections.Length == 1; }}

        // MATERIALS
        public MaterialProperty[] Materials { get; protected set; }
        public bool HasUniformMaterial { get { return Materials.Length == 1; }}

        // CTOR
        protected Element1DLayout(ElementLayoutType layoutType, int id, int nv, bool isClosed, IEnumerable<SectionProperty> sections, IEnumerable<MaterialProperty> materials)
            :base(layoutType, id, 1)
        {
            Type = layoutType;
            Nv = nv;
            Ne = (isClosed) ? Nv : Nv - 1;
            IsClosed = IsClosed;

            IsCompressionCapable = false;
            IsTractionCapable = false;
            IsShearCapable = false;
            IsBendingCapable = false;
            IsTorsionCapable = false;

            // SHALLOW COPY
            Sections = sections.ToArray();
            Materials = materials.ToArray();
        }

        // METHODS
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
                if (throwError)
                    throw new ArgumentException("The vertex index must be set according to the structure of the BeamLayout");
                return false;
            }
            return true;
        }
    }
    public sealed class BeamLayout : Element1DLayout, IBeamLayout
    {
        // GEOMETRIC CONFIGURATION
        public MFrame[] ActualConfiguration { get; private set; }
        public MFrame[] RestConfiguration { get; private set; }

        // ACTUAL LUMPED MASS
        public double[] LMx { get; private set; }
        public double[] LMθ { get; private set; }

        // ACTUAL VELOCITY
        public MVector[] Vx { get; private set; }
        public MVector[] Vθ { get; private set; }

        // ACTUAL ACCELERATION
        public MVector[] Ax { get; private set; }
        public MVector[] Aθ { get; private set; }

        public BeamLayout(int elementId, IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, IEnumerable<SectionProperty> sections, IEnumerable<MaterialProperty> materials, bool isClosed = false)
            :base(ElementLayoutType.Beam, elementId, restFrames.Count(), isClosed, sections, materials)
        {
            // MECHANICAL CAPABILITIES 
            IsCompressionCapable = true;
            IsTractionCapable = true;
            IsShearCapable = true;
            IsBendingCapable = true;
            IsTorsionCapable = true;

            // DEEP COPY of FRAMES
            ActualConfiguration = actualFrames.ToArray();
            RestConfiguration = restFrames.ToArray();

            LMx = new double[Nv];
            LMθ = new double[Nv];
            Vx = new MVector[Nv];
            Vθ = new MVector[Nv];
            Ax = new MVector[Nv];
            Aθ = new MVector[Nv];

            if (ActualConfiguration.Length != RestConfiguration.Length)
                throw new ArgumentException("restFrames and initialFrames must have the same number of frames.");
        }

        // OLD
        public BeamLoadManager LoadManager { get; private set; }
        new public int Id { get; set; }
        public MVector ConvertToMaterialCS(MVector globalVector, int index, bool isEdgeQuantity = false)
        {
            throw new NotImplementedException();
        }
        public MVector ConvertToGlobalCS(MVector materialVector, int index, bool isEdgeQuantity = false)
        {
            throw new NotImplementedException();
        }
    }

    public enum ConstraintLayoutType
    {
        BoundaryCondition = 000,
    }
    public abstract class ConstraintLayout
    {
        public int Id { get; protected set; }
        public int Dimension { get; protected set; }
        public ConstraintLayoutType Type { get; protected set; }

        protected ConstraintLayout(ConstraintLayoutType layoutType, int id, int dimension)
        {
            Id = id;
            Dimension = dimension;
            Type = layoutType;
        }
    }

    public enum Boundary
    {
        Start,
        End,
    }
    public enum BoundaryConditionType
    {
        Free,
        Pinned,     // fixed (x,y,z) & free (xx,yy,zz) + eventual end moments
        Clamped,    // fully fixed (x,y,z) & (xx,yy,zz)
    }
    public class BoundaryConditionLayout : ConstraintLayout
    {
        // TOPOLOGY
        public int ElementId { get; protected set; }
        public int ElementVertexIndex { get; protected set; }

        // RELEASE (true = release / false = fixed (infinit stiffness)
        public double Rx { get; protected set; }
        public double Rθ { get; protected set; }

        public CoordinateSystem CoordinateSystem { get; protected set; }
        public BoundaryConditionLayout(int id, int elementId, int elementVertexIndex, BoundaryConditionType type)
            :base(ConstraintLayoutType.BoundaryCondition, id, 0)
        {
            ElementId = elementId;
            ElementVertexIndex = elementVertexIndex;

            switch (type)
            {
                case BoundaryConditionType.Free:
                    Rx = 1;
                    Rθ = 1;
                    break;

                case BoundaryConditionType.Pinned:
                    Rx = 0;
                    Rθ = 1;
                    break;

                case BoundaryConditionType.Clamped:
                    Rx = 0;
                    Rθ = 0;
                    break;

                default:
                    break;
            }
        }
    }

}
