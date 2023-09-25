using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace digitalopus.util
{
    /// <summary>
    /// Adding comments to GameObjects in the Inspector.
    /// </summary>
    public class GComments : UnityEngine.MonoBehaviour
    {
        [Multiline(20)]
        public string text;
    }
}
