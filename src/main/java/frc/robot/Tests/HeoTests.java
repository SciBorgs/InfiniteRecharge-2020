package frc.robot.Tests;
import frc.robot.helpers.*;

public class HeoTests {
    public static void testGeo() {
        /// Rotate point around point
        Tester.assertEquals(Geo.rotatePoint(new Point(-2,0), Math.toRadians(0), new Point(0,0)), new Point(-2,0), "rotatePointAroundPoint, Test #1");
        Tester.assertEquals(Geo.rotatePoint(new Point(-4,4), Math.toRadians(360), new Point(0,0)), new Point(-4,4), "rotatePointAroundPoint, Test #2");
        Tester.assertEquals(Geo.rotatePoint(new Point(-4,-4), Math.toRadians(180), new Point(0,0)), new Point(4,4), "rotatePointAroundPoint, Test #3"); 

        /// Flip x and y (Point)
        Tester.assertEquals(Geo.flipXandY(new Point(0,1)), new Point(1,0), "flipPoint, Test #1");
        Tester.assertNotEquals(Geo.flipXandY(new Point(500,200)), new Point(500,200), "flipPointX, Test #2");
        
        /// Flip x and y (Line)
        Tester.assertEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p1, new Point(1,0), "flipLineP1, Test #1");
        Tester.assertEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p2, new Point(2,1), "flipLineP2, Test #1");
        Tester.assertNotEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p1, new Point(0,1), "flipLineP1, Test #2");
        Tester.assertNotEquals(Geo.flipXandY(new Line(new Point(0,1), new Point(1,2))).p2, new Point(1,2), "flipLineP2, Test #2");
        
        /// Bring in range (Not sure how it works)
        
        /// ThetaOf
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(1,0))), Math.toRadians(0), "thetaOf, Test #1");
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(1,1))), Math.toRadians(45), "thetaOf, Test #2");
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(1,-1))), Math.toRadians(-45), "thetaOf, Test #3");
        Tester.assertEquals(Geo.thetaOf(new Line(new Point(0,0), new Point(0,-1))), Math.toRadians(-90), "thetaOf, Test #3");
        
        /// mOf
        Tester.assertTrue(Geo.isPerciseEnough(Geo.mOf(new Line(new Point(0,1), new Point(1,2))), 1), "mOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.mOf(new Line(new Point(0,1), new Point(10,6))), 0.5), "mOf, Test #2");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.mOf(new Line(new Point(0,1), new Point(8,7))), 0.75), "mOf, Test #3");
        Tester.assertNotEquals(Geo.mOf(new Line(new Point(2,3), new Point(2,6))), 0, "mOf, Test #4");
        
        /// bOf
        Tester.assertTrue(Geo.isPerciseEnough(Geo.bOf(new Line(new Point(2,8), new Point(-4,-4))), 4), "bOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.bOf(new Line(new Point(-2,-8), new Point(2,4))), -2), "bOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.bOf(new Line(new Point(6,2), new Point(-2,-6))), -4), "bOf, Test #3");
        Tester.assertNotEquals(Geo.bOf(new Line(new Point(1,2), new Point(1,4))), 0, "bOf, Test #4");

        /// isVertical
        Tester.assertTrue(Geo.isVertical(new Line(new Point(2,3), new Point(2,4))), "isVertical, Test #1");
        Tester.assertTrue(Geo.isVertical(new Line(new Point(-1,4), new Point(-1,12))), "isVertical, Test #2");
        Tester.assertFalse(Geo.isVertical(new Line(new Point(2,3), new Point(10,3))), "isVertical, Test #3");
        Tester.assertFalse(Geo.isVertical(new Line(new Point(10,24), new Point(20,24))), "isVertical, Test #4");
        
        /// yOf 
        Tester.assertTrue(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-2,-2), new Point(-3,-4)), 0), 2), "yOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-1,1), new Point(-2,-4)), 0), 6), "yOf, Test #2");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-4,-1), new Point(1,4)), 0), 3), "yOf, Test #3");
        Tester.assertFalse(Geo.isPerciseEnough(Geo.yOf(new Line(new Point(-2,2), new Point(2,10)), 0), -3), "yOf, Test #4"); // See if code gets confused with y and x intercept.
        Tester.assertEquals(Geo.yOf(new Line(new Point(2,1), new Point(10, 1)), 0), 1, "yOf, Test #5"); // Horziontal

        /// xOf
        Tester.assertTrue(Geo.isPerciseEnough(Geo.xOf(new Line(new Point(-2,-2), new Point(-3,-4)), 0), -1), "xOf, Test #1");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.xOf(new Line(new Point(-4,-1), new Point(1,4)), 0), -3), "xOf, Test #2");
        Tester.assertTrue(Geo.isPerciseEnough(Geo.xOf(new Line(new Point(-2,2), new Point(2,10)), 0), -3), "xOf, Test #3");
        Tester.assertNotEquals(Geo.xOf(new Line(new Point(-2,-2), new Point(2,10)), 0), 6, "xOf, Test #4");
        Tester.assertEquals(Geo.xOf(new Line(new Point(-2,0), new Point(-2,1)), 0), -2, "xOf, Test #5"); // Vertical

        /// Point slope form
        Tester.assertEquals(Geo.pointSlopeForm(new Point(0,-4), 3), new Line(new Point(0,-4), new Point(1,-1)), "pointSlopeForm, Test #1");
        Tester.assertEquals(Geo.pointSlopeForm(new Point(1,4), 2), new Line(new Point(1,4), new Point(2,6)), "pointSlopeForm, Test #2");
        Tester.assertEquals(Geo.pointSlopeForm(new Point(2,2), 0), new Line(new Point(2,2), new Point(3,2)), "pointSlopeForm, Test #3");
        
        /// Slope intercept form
        Tester.assertEquals(Geo.slopeInterceptForm(2,3), new Line(new Point(0,3), new Point(1,5)), "slopeInterceptForm, Test #1");
        Tester.assertEquals(Geo.slopeInterceptForm(1,-3), new Line(new Point(0,-3), new Point(1,-2)), "slopeInterceptForm, Test #2");
        Tester.assertEquals(Geo.slopeInterceptForm(0,2), new Line(new Point(0,2), new Point(1,2)), "slopeInterceptForm, Test #3");
        
        /// Point Angle Form
        
        /// Distance between two points
        Tester.assertEquals(Geo.getDistance(new Point(0,0), new Point(0,1)), 1, "distanceOfTwoPoints, #1");
        Tester.assertNotEquals(Geo.getDistance(new Point(0,0), new Point(3,0)), 1, "distanceOfTwoPoints, #2");

        /// Distance between lineLike and point
        Tester.assertEquals(Geo.getDistance(new Line(new Point(2,0), new Point(2,2)), new Point(0,0)), 2, "distanceLinePoint");
        Tester.assertEquals(Geo.getDistance(new LineSegment(new Point(0,1), new Point(1,0)), new Point(1, -1)), 1, "distanceLineSegmentPoint");
        Tester.assertEquals(Geo.getDistance(new Ray(new Point(1,0), new Point(0,2)), new Point(1,-1)), 1, "distanceRayPoint");

        /// arePointsCollinear (One broken)
        Tester.assertTrue(Geo.arePointsCollinear(new Point(1,0), new Point(1,2), new Point(1,1)), "arePointsCollinear, Test #1");
        Tester.assertTrue(Geo.arePointsCollinear(new Point(3,0), new Point(1,2), new Point(2,1)), "arePointsCollinear, Test #2");
        Tester.assertFalse(Geo.arePointsCollinear(new Point(1,0), new Point(1,2), new Point(0,1)), "arePointsCollinear, Test #3"); // This one is for some reason true
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
        Tester.assertEquals(Geo.add(new Point(1,1), new Point(1,1)), new Point(2,2), "add, Test #1");
        Tester.assertEquals(Geo.add(new Point(3,1), new Point(1,2)), new Point(4,3), "add, Test #2");
        Tester.assertNotEquals(Geo.add(new Point(1,1), new Point(1,1)), new Point(0,0), "add, Test #3");

        /// Subtract point (Something is wrong)
        Tester.assertEquals(Geo.sub(new Point(1,1), new Point(1,1)), new Point(0,0), "sub, Test #1");
        Tester.assertEquals(Geo.sub(new Point(5,2), new Point(1,1)), new Point(7,3), "sub, Test #2");
        //Tester.assertNotEquals(Geo.sub(new Point(1,1), new Point(1,1)), new Point(2,2), "sub, Test #3");

        /// getMagnitude

        /// dot
        Tester.assertEquals(Geo.dot(new Point(1,1), new Point(2,3)), 5, "dot, Test #1");
        Tester.assertEquals(Geo.dot(new Point(-1,-1), new Point(4,5)), -9, "dot, Test #2");
        Tester.assertEquals(Geo.dot(new Point(2,5), new Point(1,3)), 17, "dot, Test #3");

        /// Angle between points
    }
}