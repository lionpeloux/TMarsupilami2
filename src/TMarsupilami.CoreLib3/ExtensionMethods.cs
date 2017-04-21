using System;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public static class ArrayExtension
    {
        /// <summary>
        /// Extracts axis from a set of frames.
        /// </summary>
        /// <param name="frames">The frames to extract the directors.</param>
        /// <param name="axis">The axis to extract : global (X, Y, Z) or local (d1, d2, t).</param>
        /// <returns>The extracted axis.</returns>
        public static MVector[] GetAxis(this MFrame[] frames, Axis axis)
        {
            var vectors = new MVector[frames.Length];

            switch (axis)
            {
                case Axis.X:
                    vectors.Populate(MVector.XAxis);
                    break;

                case Axis.Y:
                    vectors.Populate(MVector.YAxis);
                    break;

                case Axis.Z:
                    vectors.Populate(MVector.ZAxis);
                    break;

                case Axis.d1:
                    for (int i = 0; i < frames.Length; i++)
                        vectors[i] = frames[i].XAxis;
                    break;

                case Axis.d2:
                    for (int i = 0; i < frames.Length; i++)
                        vectors[i] = frames[i].YAxis;
                    break;

                case Axis.t:
                    for (int i = 0; i < frames.Length; i++)
                        vectors[i] = frames[i].ZAxis;
                    break;
            }
            return vectors;
        }
    }

}
