using System;

namespace TMarsupilami.MathLib
{
    public static class ArrayExtension
    {
        /// <summary>
        /// DeepCopy an array. A new array is created where each element is deep copied.
        /// </summary>
        /// <typeparam name="T">Array type. Must implement IDeepCopy<T>.</typeparam>
        /// <param name="value">Array to DeepCopy.</param>
        /// <returns>A new array with all elements deep copied.</returns>
        public static T[] DeepCopy<T>(this T[] value) where T : IDeepCopy<T>
        {
            T[] array = new T[value.Length];
            for (int i = 0; i < value.Length; i++)
            {
                array[i] = value[i].DeepCopy();
            }
            return array;
        }

        /// <summary>
        /// ShallowCopy. A new array is created where each element is shallow copied.
        /// </summary>
        /// <typeparam name="T">Array type.</typeparam>
        /// <param name="value">Array to ShallowCopy.</param>
        /// <returns>A new array with all element shallow copied.</returns>
        public static T[] ShallowCopy<T>(this T[] value)
        {
            T[] array = new T[value.Length];
            for (int i = 0; i < value.Length; i++)
            {
                array[i] = value[i];
            }
            return array;
        }
    }

}
