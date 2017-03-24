using System;
using TMarsupilami.CoreLib.Elements.Models;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Elements;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilami.CoreLib.Element
{

    public abstract class BeamModel : IModel1D
    {
        public string ModelRef { get; protected set; }
        public string Description { get; protected set; }
        public int SpatialDimension
        {
            get { return 1; }
        }

        public int NumberOfDof { get { return NumberOfTranslationalDof + NumberOfRotationalDof; } }
        public int NumberOfTranslationalDof { get { return 3; } }
        public int NumberOfRotationalDof { get; protected set; }

        public void Move(MVector[] dx)
        {
            // x = x + dx
            for (int i = 0; i < Nv; i++)
            {
                x[i] = MaterialFrame[i].Origin + dx[i];
            }
        }

        public void Move(double[] dθ)
        {
            throw new NotImplementedException();
        }

        public void UpdateBendingMoment()
        {
            throw new NotImplementedException();
        }

        public void UpdateTwistingMoment()
        {
            throw new NotImplementedException();
        }

        public void UpdateInternalNodalMoment()
        {
            throw new NotImplementedException();
        }

        public void UpdateResultantNodalMoment()
        {
            throw new NotImplementedException();
        }

        public void UpdateAxialForce()
        {
            throw new NotImplementedException();
        }

        public void UpdateShearForce()
        {
            throw new NotImplementedException();
        }

        public void UpdateInternalNodalForce()
        {
            throw new NotImplementedException();
        }

        public void UpdateResultantNodalForce()
        {
            throw new NotImplementedException();
        }

        public double GetAxialElasticEnergy()
        {
            throw new NotImplementedException();
        }

        public double GetBendingElasticEnergy()
        {
            throw new NotImplementedException();
        }

        public double GetTwistingElasticEnergy()
        {
            throw new NotImplementedException();
        }
    }
}
