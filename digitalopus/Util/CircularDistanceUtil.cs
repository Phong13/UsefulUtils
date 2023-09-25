using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace digitalopus.util
{
    public static class CircularDistanceUtil
    {
        public static int GetNext(int idx, int numInRing, int incDir)
        {
            idx += incDir;
            if (idx >= numInRing) idx = 0;
            if (idx < 0) idx = numInRing - 1;
            return idx;
        }

        public static int GetPrevious(int idx, int numInRing, int incDir)
        {
            idx -= incDir;
            if (idx >= numInRing) idx = 0;
            if (idx < 0) idx = numInRing - 1;
            return idx;
        }

        public static int MinDistanceDirection(int a, int b, int numInRing)
        {
            Debug.Assert(a < numInRing && b < numInRing);
            Debug.Assert(a >= 0 && b >= 0);
            int d1 = b - a;
            int signD1 = (int)Mathf.Sign(d1);
            int d2 = -(signD1 * (numInRing - Mathf.Abs(d1)));
            if (Mathf.Abs(d1) <= Mathf.Abs(d2))
            {
                //Debug.Log("a:" + a + "  b:" + b + "  " + signD1 + "  d1:" +  d1 + "  " + d2 + "  " + signD1);
                return signD1;
            }
            else
            {
                //Debug.Log("a:" + a + "  b:" + b + "  " + (-signD1) + "    d1:" + d1 + "  " + d2 + "  " + signD1);
                return -signD1;
            }
        }

        public static void MinDistancePath(int a, int b, int numInRing, List<int> outPath, out int incrDir)
        {
            Debug.Assert(a < numInRing && b < numInRing);
            Debug.Assert(a >= 0 && b >= 0);
            int d1 = b - a;
            int signD1 = (int)Mathf.Sign(d1);
            int d2 = -(signD1 * (numInRing - Mathf.Abs(d1)));
            if (Mathf.Abs(d1) <= Mathf.Abs(d2))
            {
                incrDir = signD1;
            }
            else
            {
                incrDir = -signD1;
            }

            outPath.Clear();
            int idx = a;
            while (idx != b)
            {
                outPath.Add(idx);
                idx += incrDir;
                if (idx >= numInRing) idx = 0;
                if (idx < 0) idx = numInRing - 1;
            }

            outPath.Add(b);
            //Debug.Log("a:" + a + " b:" + b + "    path:" +  string.Join<int>(",", outPath));
        }

        public static int MinDistance(int a, int b, int numInRing)
        {
            Debug.LogError("Untested");
            Debug.Assert(a < numInRing && b < numInRing);
            Debug.Assert(a >= 0 && b >= 0);
            int mx = Mathf.Max(a, b);
            int mn = Mathf.Min(a, b);
            int d1 = mx - mn;
            int d0 = numInRing - d1;
            return Mathf.Min(d1, d0);
        }
    }
}
