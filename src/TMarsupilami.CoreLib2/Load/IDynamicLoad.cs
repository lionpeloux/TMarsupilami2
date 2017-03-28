
namespace TMarsupilami.CoreLib2
{

    public delegate void DynamicBeamVectorLoadUpdate(double t, BeamVectorLoad load);
    public interface IDynamicLoad
    {
        DynamicBeamVectorLoadUpdate TimeUpdate { get; }
    }
}
