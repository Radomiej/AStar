﻿namespace AStar.Core.Utils
{
    public class Grid
    {
        private readonly int _width;
        private readonly int _height;
        private readonly long _size;
        private readonly bool _inverse_index_dimension;

        public long Size => _size;
        public bool InverseIndexDimension => _inverse_index_dimension;
        public int Width => _width;
        public int Height => _height;

        public Grid(int width, int height)
        {
            _width = width;
            _height = height;
            _size = _width * _height;
            _inverse_index_dimension = _height > _width;
        }

        public long GetNodeIndex(Point position)
        {
            return GetNodeIndex(position.X, position.Y);
        }

        public long GetNodeIndex(int x, int y)
        {
            long baseIndex = _inverse_index_dimension ? y * _height : x * _width;
            baseIndex += _inverse_index_dimension ? x : y;
            return baseIndex;
        }

        public Point GetPositionForIndex(long index)
        {
            if (_inverse_index_dimension)
            {
                int x = (int) (index % _height);
                int y = (int) (index / _height);
                return new Point(x, y);
            }
            else
            {
                int x = (int) (index / _width);
                int y = (int) (index % _width);
                return new Point(x, y);
            }
        }

        public bool InBounds(Point pointToTranslate)
        {
            return pointToTranslate.X >= 0 && pointToTranslate.Y >= 0 && pointToTranslate.X < Width &&
                   pointToTranslate.Y < Height;
        }
    }
}