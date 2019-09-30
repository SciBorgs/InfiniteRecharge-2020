package frc.robot.tests;

import frc.robot.helpers.*;
import frc.robot.shapes.*;

import java.util.Optional;

public class GeoTest {

    private static Line flipPoints(Line l){
        /*
        This shouldn't change the line. 
        f(l) should always be the same as f(flipPoints(l)) 
        [unless f is this or something that returns the points]
        */
        return new Line(l.p2, l.p1);
    }

    public static void testGeo() {
        /// Common point(s)
        Point o = new Point(0,0);
        
        /// Rotate point around point
        Tester.assertEquals(Geo.rotatePoint(new Point(-2, 0), Math.toRadians(0)),   new Point(-2,0), "rotatePointAroundPoint, Test #1");
        Tester.assertEquals(Geo.rotatePoint(new Point(-4, 4), Math.toRadians(360)), new Point(-4,4), "rotatePointAroundPoint, Test #2");
        Tester.assertEquals(Geo.rotatePoint(new Point(-4,-4), Math.toRadians(180)), new Point(4,4),  "rotatePointAroundPoint, Test #3"); 

        /// Flip x and y (Point)
        Tester.assertEquals(Geo.flipXandY(new Point(0,1)), new Point(1,0), "flipPoint, Test #1");
        Tester.assertNotEquals(Geo.flipXandY(new Point(500,200)), new Point(500,200), "flipPoint, Test #2");
        
        /// Flip x and y (Line)
        Line flipXYLine = new Line(new Point(0,1), new Point(1,2));
        Tester.assertEquals(Geo.flipXandY(flipXYLine), new Line(new Point(1,0), new Point(2,1)), "flipLine, Test #1");

        /// ThetaOf
        Line l = new Line(o, new Point(1, 0));
        Tester.assertEquals(Math.toDegrees(Geo.thetaOf(l)),          0, "thetaOf, Test #1");
        Tester.assertEquals(Geo.thetaOf(l), Geo.thetaOf(flipPoints(l)), "thetaOf, Test #2");
        
        Tester.assertEquals(Math.toDegrees(Geo.thetaOf(new Line(o, new Point(-1,1)))),  -45, "thetaOf, Test #3");
        Tester.assertEquals(Math.toDegrees(Geo.thetaOf(new Line(o, new Point(0,-2)))),  -90, "thetaOf, Test #4");
        
        /// mOf 
        Tester.assertImpresiceEquals(Geo.mOf(new Line(new Point(0,1), new Point(1,2))), 1, "mOf, Test #1");
        Tester.assertImpresiceEquals(Geo.mOf(new Line(new Point(0,1), new Point(10,6))), 0.5, "mOf, Test #2");
        
        Tester.assertEquals(Geo.mOf(new Line(new Point(2,3), new Point(2,6))), 1/0.0, "mOf, Test #4");
        
        /// bOf
        Tester.assertImpresiceEquals(Geo.bOf(new Line(new Point(2,8),   new Point(-4,-4))),  4,   "bOf, Test #1");
        Tester.assertImpresiceEquals(Geo.bOf(new Line(new Point(-2,-8), new Point(2,4))),   -2,   "bOf, Test #2");
        Tester.assertImpresiceEquals(Geo.bOf(new Line(new Point(6,2),   new Point(-2,-6))), -4,   "bOf, Test #3");
        
        Tester.assertEquals(Geo.bOf( new Line(new Point(1,2),   new Point(1,4))),  -1/0.0, "bOf, Test #4");

        /// isVertical
        Tester.assertTrue(Geo.isVertical(  new Line(new Point(2,3),   new Point(2,4))),   "isVertical, Test #1");
        Tester.assertTrue(Geo.isVertical(  new Line(new Point(-1,4),  new Point(-1,12))), "isVertical, Test #2");
        Tester.assertFalse(Geo.isVertical( new Line(new Point(2,3),   new Point(10,3))),  "isVertical, Test #3");
        Tester.assertFalse(Geo.isVertical( new Line(new Point(10,24), new Point(20,24))), "isVertical, Test #4");
        
        /// yOf 
        Tester.assertImpresiceEquals(Geo.yOf( new Line(new Point(-2,-2), new Point(-3,-4)), 0),  2,  "yOf, Test #1");
        Tester.assertImpresiceEquals(Geo.yOf( new Line(new Point(-1,1),  new Point(-2,-4)), 0),  6,  "yOf, Test #2");
        Tester.assertImpresiceEquals(Geo.yOf( new Line( new Point(-2,2), new Point(2,-6)),  0),  -2,  "yOf, Test #3");
        Tester.assertEquals(         Geo.yOf( new Line( new Point(2,1),  new Point(10, 1)), 0),  1,  "yOf, Test #4"); // Horziontal

        /// xOf
        Tester.assertImpresiceEquals(Geo.xOf(new Line(new Point(-2,-2), new Point(-3,-4)), 0), -1, "xOf, Test #1");
        Tester.assertImpresiceEquals(Geo.xOf(new Line(new Point(-4,-1), new Point(1,4)), 0),   -3, "xOf, Test #2");
        
        Tester.assertNotEquals(Geo.xOf(new Line(new Point(-2,-2), new Point(2,10)), 0),   6,  "xOf, Test #3");
        Tester.assertEquals(   Geo.xOf(new Line(new Point(-2,0),  new Point(-2,1)), 0),  -2,  "xOf, Test #4"); // Vertical

        /// Point slope form
        Tester.assertEquals(Geo.pointSlopeForm(new Point(0,-4), 3), new Line(new Point(0,-4), new Point(1,-1)),  "pointSlopeForm, Test #1");
        Tester.assertEquals(Geo.pointSlopeForm(new Point(1,4),  2), new Line(new Point(1,4),  new Point(2,6)),   "pointSlopeForm, Test #2");
        Tester.assertEquals(Geo.pointSlopeForm(new Point(2,2),  0), new Line(new Point(2,2),  new Point(3,2)),   "pointSlopeForm, Test #3"); // Horizontal
        
        /// Slope intercept form
        Tester.assertEquals(Geo.slopeInterceptForm(2,3),  new Line(new Point(0,3),  new Point(1,5)),  "slopeInterceptForm, Test #1");
        Tester.assertEquals(Geo.slopeInterceptForm(1,-3), new Line(new Point(0,-3), new Point(1,-2)), "slopeInterceptForm, Test #2");
        Tester.assertEquals(Geo.slopeInterceptForm(0,2),  new Line(new Point(0,2),  new Point(1,2)),  "slopeInterceptForm, Test #3"); // Horizontal
        
        /// Point Angle Form
        Tester.assertEquals(Geo.pointAngleForm(new Point(2,0), Math.toRadians(45)),  
                                               new Line(new Point(2,0),  new Point(3,1)),  "pointAngleForm, Test #1");
        Tester.assertEquals(Geo.pointAngleForm(new Point(2,0), Math.toRadians(135)), 
                                               new Line(new Point(2,0),  new Point(3,-1)), "pointAngleForm, Test #2");
        Tester.assertEquals(Geo.pointAngleForm(o, Math.toRadians(90)),  
                                               new Line(o, new Point(0,1)), "pointAngleForm, Test #3");
        
        /// Distance between two points 
        Tester.assertEquals(   Geo.getDistance(o, new Point(0,1)),  1, "distanceOfTwoPoints, #1");
        Tester.assertNotEquals(Geo.getDistance(o, new Point(3,0 )), 1, "distanceOfTwoPoints, #2");

        /// Distance between lineLike and point 
        Tester.assertEquals(Geo.getDistance(new Line(       new Point(2,0), new Point(2,2)), o),  2, "distanceLinePoint");
        Tester.assertEquals(Geo.getDistance(new LineSegment(new Point(0,1), new Point(1,0)), new Point(1,-1)), 1, "distanceLineSegmentPoint");
        Tester.assertEquals(Geo.getDistance(new Ray(        new Point(1,0), new Point(0,2)), new Point(1,-1)), 1, "distanceRayPoint");

        /// arePointsCollinear
        Tester.assertTrue( Geo.arePointsCollinear(new Point(1,0), new Point(1,2), new Point(1,1)), "arePointsCollinear, Test #1");
        Tester.assertTrue( Geo.arePointsCollinear(new Point(3,0), new Point(1,2), new Point(2,1)), "arePointsCollinear, Test #2");
        Tester.assertTrue( Geo.arePointsCollinear(new Point(0,1), new Point(2,1), new Point(1,1)), "arePointsCollinear, Test #3");
        Tester.assertTrue( Geo.arePointsCollinear(new Point(0,3), new Point(3,0), new Point(1,2)), "arePointsCollinear, Test #4");
        
        Tester.assertFalse(Geo.arePointsCollinear(new Point(1,0), new Point(1,2), new Point(0,1)), "arePointsCollinear, Test #5");
        Tester.assertFalse(Geo.arePointsCollinear(new Point(3,0), new Point(1,2), new Point(3,2)), "arePointsCollinear, Test #6");
        
        /// getMidpoint
        Tester.assertEquals(   Geo.getMidpoint(new Point(1,0), new Point(1,2)), new Point(1,1), "Midpoint, Test #1");
        Tester.assertEquals(   Geo.getMidpoint(new Point(1,0), new Point(3,0)), new Point(2,0), "Midpoint, Test #2");
        Tester.assertNotEquals(Geo.getMidpoint(new Point(1,0), new Point(3,0)), new Point(3,0), "Midpoint, Test #3");
        Tester.assertNotEquals(Geo.getMidpoint(o, new Point(8,0)), new Point(2,0), "Midpoint, Test #4");

        /// getPerpendicular
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(1,0),  new Point(1,2)), new Point(1,1)), 
                                                 new Line(new Point(-1,1), new Point(2,1)), "getPerpendicular, Test #1");
        
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(2,2),               o), new Point(1,1)), 
                                                 new Line(new Point(0,2),  new Point(2,0)), "getPerpendicular, Test #2");
        
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(2,0),  new Point(6,0)), new Point(3,0)), 
                                                 new Line(new Point(3,2),  new Point(3,-1)), "getPerpendicular, Test #3");
        
        Tester.assertEquals(Geo.getPerpendicular(new Line(new Point(2,0),  new Point(6,4)), new Point(5,3)),
                                                 new Line(new Point(4,4),  new Point(6,2)), "getPerpendicular, Test #4");

        /// areParallel
        Line parallelLine1 = new Line(new Point(0,2), new Point(-2,0));
        Line parallelLine2 = new Line(new Point(2,0), new Point(0,-2));

        Line notParallelLine1 = new Line(o, new Point(1,1));
        Line notParallelLine2 = new Line(new Point(1,0), new Point(0,1));

        Ray parallelRay1 = new Ray(new Point(1,0),  new Point(0,1));
        Ray parallelRay2 = new Ray(new Point(1,-1), o);

        LineSegment parallelSegment1 = new LineSegment(new Point(2,2), new Point(2,0));
        LineSegment parallelSegment2 = new LineSegment(new Point(1,0), new Point(1,2));
        
        Tester.assertTrue(Geo.areParellel(new Line(o, new Point(0,1)), 
                                          new Line(new Point(1,0), new Point(1,1))), "areParallel, Test #1"); // Vertical
        
        Tester.assertTrue(Geo.areParellel(parallelLine1, parallelLine2), "areParallel, Test #2");
        Tester.assertTrue(Geo.areParellel(flipPoints(parallelLine1), flipPoints(parallelLine2)), "areParallel, Test #3");

        Tester.assertTrue(Geo.areParellel(parallelRay1, parallelRay2), "areParallel, Test #4");

        Tester.assertTrue(Geo.areParellel(parallelSegment1, parallelSegment2), "areParallel, Test #5");

        Tester.assertFalse(Geo.areParellel(notParallelLine1, notParallelLine2), "areParallel, Test #6");

        /// getIntersection
        Line intersectionLine1 = new Line(new Point(-1,2), new Point(1,0));
        Line intersectionLine2 = new Line(new Point(-1,0), new Point(1,2));

        Ray noIntersectionRay1 = new Ray(new Point(3,0), new Point(4,1));
        Ray noIntersectionRay2 = new Ray(new Point(2,1), new Point(1,2));
        
        Ray yesIntersectionRay1 = new Ray(new Point(4,0), new Point(3,-1));
        Ray yesIntersectionRay2 = new Ray(o, new Point(1,-1));

        LineSegment intersectionSegment1 = new LineSegment(new Point(3,0), new Point(1,2));
        LineSegment intersectionSegment2 = new LineSegment(new Point(1,0), new Point(2,1));

        Tester.assertEquals(Geo.getIntersection(new Line(new Point(0,1), new Point(1,0)), 
                                                new Line(o, new Point(1,1))).get(), new Point(0.5,0.5), "getIntersection, Test #1");
        
        Tester.assertEquals(Geo.getIntersection(intersectionLine1, intersectionLine2).get(), new Point(0,1), "getIntersection, Test #2");
        Tester.assertEquals(Geo.getIntersection(flipPoints(intersectionLine1), 
                                                flipPoints(intersectionLine2)).get(), new Point(0,1), "getIntersection, Test #3");
        
        Tester.assertEquals(Geo.getIntersection(noIntersectionRay1, noIntersectionRay2), Optional.empty(), "getIntersection, Test #4");
        Tester.assertEquals(Geo.getIntersection(yesIntersectionRay1, yesIntersectionRay2).get(), new Point(2,-2), "getIntersection, Test #5");

        Tester.assertEquals(Geo.getIntersection(intersectionSegment1, intersectionSegment2).get(), new Point(2,1), "getIntersection, Test #6");

        /// Scale point
        Tester.assertEquals(Geo.scale(new Point(4,2), 3),   new Point(12,6),   "scale, Test #1");
        Tester.assertEquals(Geo.scale(new Point(-3,-5), 2), new Point(-6,-10), "scale, Test #1");

        /// Add points
        Tester.assertEquals(   Geo.add(new Point(1,1), new Point(1,1)), new Point(2,2), "add, Test #1");
        Tester.assertEquals(   Geo.add(new Point(3,1), new Point(1,2)), new Point(4,3), "add, Test #2");
        Tester.assertNotEquals(Geo.add(new Point(1,1), new Point(1,1)),              o, "add, Test #3");

        /// Subtract points
        Tester.assertEquals(Geo.sub(new Point(1,1), new Point(1,1)),              o, "sub, Test #1");
        Tester.assertEquals(Geo.sub(new Point(5,2), new Point(1,1)), new Point(4,1), "sub, Test #2");
        Tester.assertNotEquals(Geo.sub(new Point(1,1), new Point(1,1)), new Point(2,2), "sub, Test #3"); // Confused with add and sub

        /// getMagnitude
        Tester.assertEquals(Geo.getMagnitude(new Point(2,0)),  2, "getMagnitude, Test #1");
        Tester.assertEquals(Geo.getMagnitude(new Point(0,-7)), 7, "getMagnitude, Test #1");

        /// dot
        Tester.assertEquals(Geo.dot(new Point(1,1),   new Point(2,3)),  5, "dot, Test #1");
        Tester.assertEquals(Geo.dot(new Point(-1,-1), new Point(4,5)), -9, "dot, Test #2");
        Tester.assertEquals(Geo.dot(new Point(2,5),   new Point(1,3)), 17, "dot, Test #3");

        /// Angle between points
        Tester.assertEquals(Geo.angleBetween(o, new Point(1,0)),   Math.toRadians(0),    "angleBetween, Test #1");
        Tester.assertEquals(Geo.angleBetween(o, new Point(1,1)),   Math.toRadians(45),   "angleBetween, Test #2");
        Tester.assertEquals(Geo.angleBetween(o, new Point(-2,-2)), Math.toRadians(-135), "angleBetween, Test #3");
    }
}