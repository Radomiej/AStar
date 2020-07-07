using System;

namespace AStar.Core.Utils
{
    public class BinarySizeHelper
    {
        public static int ComputeSize(in int mapSize)
        {
            for (int i = 4; i < 16; i++)
            {
                int pow2Size = (int) Math.Pow(2, i);
                if (mapSize <= pow2Size)
                {
                    return pow2Size;
                }
            }

            throw new NotSupportedException("Map size is so large! " + mapSize);
        }
    }
}