package frc.robot.routing.navigationmesh;

import java.util.Objects;

import frc.robot.helpers.Point;

public class Edge {
    public Point origin, dest;
    public Edge prevOrigin; // Shares the origin & is clockwise to the edge.
    public Edge nextOrigin; // Shares the origin & is counterclockwise to the edge.
    public Edge symEdge;    // Same edge, but points in the opposite direction.
    
    //                         * <- dest / symEdge.origin
    //                         |
    //                         |
    //                         |
    //                         * <- origin / symEdge.dest
    //                       /   \
    //                      /     \
    //                     /       \
    // nextOrigin.dest -> *         * <- prevOrigin.dest

    public Edge(Point origin, Point dest) {
        this.origin = origin;
        this.dest   = dest;

        prevOrigin = null;
        nextOrigin = null;
        symEdge    = null;
    }

    @Override
    public boolean equals(Object o) {
        if (o == this)
            return true;
        if (!(o instanceof Edge)) {
            return false;
        }
        Edge edge = (Edge) o;
        return Objects.equals(origin, edge.origin) && Objects.equals(dest, edge.dest)
                && Objects.equals(prevOrigin, edge.prevOrigin) && Objects.equals(nextOrigin, edge.nextOrigin)
                && Objects.equals(symEdge, edge.symEdge);
    }

    @Override
    public int hashCode() {
        return Objects.hash(origin, dest, prevOrigin, nextOrigin, symEdge);
    }
}