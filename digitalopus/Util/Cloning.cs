using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace digitalopus.util
{
    public class Cloning
    {
        /// <summary>
        /// Make a deep copy of serialized properties of an object.
        /// </summary>
        public static T DeepClone<T>(T obj)
        {
            using (var ms = new System.IO.MemoryStream())
            {
                var formatter = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();
                formatter.Serialize(ms, obj);
                ms.Position = 0;
                return (T)formatter.Deserialize(ms);
            }
        }
    }
}
