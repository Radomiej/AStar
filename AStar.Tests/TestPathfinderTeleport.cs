using NUnit.Framework;

namespace AStar.Tests
{
    public class TestPathfinderTeleport
    {
        [Test]
        public void TestPathfinderTeleportDefaultCase()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1}
            });
            pathFinder.AddTeleport(new Point(1, 1), new Point(6, 6));
            var result = pathFinder.FindPath(new Point(0, 0), new Point(7, 7));
            
            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result);
            Assert.AreEqual(4, result.Count);
        }
        
        [Test]
        public void TestPathfinderDynamicChangeCostOfMove()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1}
            });
            pathFinder.ChangeCostOfMove(1, 1, 0);
            pathFinder.ChangeCostOfMove(2, 2, 0);
            var result = pathFinder.FindPath(new Point(0, 0), new Point(3, 3));
            
            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result);
            Assert.AreEqual(5, result.Count);
        }
        
        [Test]
        public void TestPathfinderCannotMove1()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {0, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 0}
            });
            
            var result = pathFinder.FindPath(new Point(0, 0), new Point(3, 3));
            
            Assert.IsNull(result);
        }
        
        [Test]
        public void TestPathfinderCannotMove3()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 0}
            });
            
            var result = pathFinder.FindPath(new Point(0, 0), new Point(3, 3));
            
            Assert.IsNull(result);
        }
        
        [Test]
        public void TestPathfinderCannotMove2()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1}
            });
            pathFinder.ChangeCostOfMove(0, 0, 0);
            pathFinder.ChangeCostOfMove(3, 3, 0);
            var result = pathFinder.FindPath(new Point(0, 0), new Point(3, 3));
            
            Assert.IsNull(result);
        }
        
        [Test]
        public void TestPathfinderCannotMove4()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1},
                {1, 1, 1, 1}
            });
            pathFinder.ChangeCostOfMove(3, 3, 0);
            var result = pathFinder.FindPath(new Point(0, 0), new Point(3, 3));
            
            Assert.IsNull(result);
        }
        
        [Test]
        public void TestPathfinderCannotMove5()
        {
            PathFinder pathFinder = new PathFinder(new byte[,]
            {
                {1, 1, 0, 0},
                {1, 0, 0, 1},
                {0, 0, 1, 1},
                {0, 1, 1, 1}
            });
            var result = pathFinder.FindPath(new Point(0, 0), new Point(3, 3));
            
            Assert.IsNull(result);
        }
    }
}