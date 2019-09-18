package frc.robot;
import frc.robot.helpers.*;

public class BaseRobotProkecy {
    public static void testGeo() {
        /// Rotate point around point
        Tester.assertEquals(Geo.rotatePoint(new Point(-2,0), Math.toRadians(0), new Point(0,0)), 
                            new Point(-2,0), "rotatePointAroundPoint, Test #1");
        Tester.assertEquals(Geo.rotatePoint(new Point(-4,4), Math.toRadians(360), new Point(0,0)), 
                            new Point(-4,4), "rotatePointAroundPoint, Test #2");
        Tester.assertEquals(Geo.rotatePoint(new Point(-4,-4), Math.toRadians(180), new Point(0,0)), 
                            new Point(4,4), "rotatePointAroundPoint, Test #3");

        /// Flip x and y (Point)
        Tester.assertEquals(Geo.flipXandY(new Point(0,1)), new Point(1,0), "flipPoint, Test #1");
        Tester.assertNotEquals(Geo.flipXandY(new Point(500,200)), new Point(500,200), "flipPointX, Test #2");

        /// Flip x and y (Line)
        Tester.assertEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p1, new Point(1,0), "flipLineP1, Test #1");
        Tester.assertEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p2, new Point(2,1), "flipLineP2, Test #1");
        
        Tester.assertNotEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p1, new Point(0,1), "flipLineP1, Test #2");
        Tester.assertNotEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p2, new Point(1,2), "flipLineP2, Test #2");

        /// yOf 
        Tester.assertTrue(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-2,-2), new Point(-3,-4)), 0), 2), "yOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-1,1), new Point(-2,-4)), 0), 6), "yOf, Test #2");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-4,-1), new Point(1,4)), 0), 3), "yOf, Test #3");
        Tester.assertFalse(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-2,2), new Point(2,10)), 0), -3), "yOf, Test #4"); //x-intercept test

        /// xOf
        Tester.assertTrue(Geo.isPerciseEnough(Geo.xOf(new Line(new Point(-2,-2), new Point(-3,-4)), 0), -1), "xOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.xOf(new Line(new Point(-4,-1), new Point(1,4)), 0), -3), "xOf, Test #2");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.xOf(new Line(new Point(-2,2), new Point(2,10)), 0), -3), "xOf, Test #3");
        Tester.assertNotEquals(Geo.xOf(new Line(new Point(-2,-2), new Point(2,10)), 0), 6, "xOf, Test #3");

        /// thetaOf
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(1,0))), Math.toRadians(0), "thetaOf, Test #1");
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(1,1))), Math.toRadians(45), "thetaOf, Test #2");
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(1,-1))), Math.toRadians(-45), "thetaOf, Test #3");
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(0,-1))), Math.toRadians(-90), "thetaOf, Test #3");

        /// mOf
        Tester.assertTrue(Geo.isPerciseEnough(Geo.mOf(new Line(new Point(0,1), new Point(1,2))), 1), "mOf Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.mOf(new Line(new Point(0,1), new Point(10,6))), 0.5), "mOf Test #2");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.mOf(new Line(new Point(0,1), new Point(8,7))), 0.75), "mOf Test #3");

        /// Distance between two points 
        Tester.assertEquals(Geo.getDistance(new Point(0,0), new Point(0,1)), 1, "distanceOfTwoPoints, #1");
        Tester.assertNotEquals(Geo.getDistance(new Point(0,0), new Point(3,0)), 1, "distanceOfTwoPoints, #2");

        /// Distance between lineLike and point (Working)
        Tester.assertEquals(Geo.getDistance(new Line(new Point(2,0), new Point(2,2)), new Point(0,0)), 2, "distanceLinePoint");
        Tester.assertEquals(Geo.getDistance(new LineSegment(new Point(0,1), new Point(1,0)), new Point(1, -1)), 1, "distanceLineSegmentPoint");
        Tester.assertEquals(Geo.getDistance(new Ray(new Point(1,0), new Point(0,2)), new Point(1,-1)), 1, "distanceRayPoint");

        /// arePointsCollinear
        Tester.assertTrue(Geo.arePointsCollinear(new Point(1,0), new Point(1,2), new Point(1,1)), "arePointsCollinear, Test #1");
        Tester.assertTrue(Geo.arePointsCollinear(new Point(3,0), new Point(1,2), new Point(2,1)), "arePointsCollinear, Test #2");
        Tester.assertFalse(Geo.arePointsCollinear(new Point(1,0), new Point(1,2), new Point(0,1)), "arePointsCollinear, Test #3");
        Tester.assertFalse(Geo.arePointsCollinear(new Point(3,0), new Point(1,2), new Point(3,2)), "arePointsCollinear, Test #4");

        /// isPointInsideCircle (Confused)

        /// getMidpoint (Something fishy)
        Tester.assertEquals(Geo.getMidpoint(new Point(1,0), new Point(1,2)), new Point(1,1), "Midpoint, Test #1");
        Tester.assertEquals(Geo.getMidpoint(new Point(1,0), new Point(3,0)), new Point(2,0), "Midpoint, Test #2");
        Tester.assertEquals(Geo.getMidpoint(new Point(1,0), new Point(3,0)), new Point(3,0), "Midpoint, Test #3");

        /// getPerpendicular
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(1,0), new Point(1,2)), new Point(1,1)), 
                            new Line(new Point(-1,1), new Point(2,1)), "getPerpendicular, Test #1");
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(2,2), new Point(0,0)), new Point(1,1)), 
                            new Line(new Point(0,2), new Point(2,0)), "getPerpendicular, Test #2");
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(2,0), new Point(6,0)), new Point(3,0)), 
                            new Line(new Point(3,2), new Point(3,-1)), "getPerpendicular, Test #3");
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(2,0), new Point(6,4)), new Point(5,3)),
                            new Line(new Point(4,4), new Point(6,2)), "getPerpendicular, Test #4");
        /// areParallel

        /// getIntersection

        /// Scale point

        /// Add point

        /// Subtract point

        /// getMagnitude

        /// dot (Confused)

        /// Angle between points
    }
}