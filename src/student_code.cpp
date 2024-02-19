#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (class member).
     *
     * @param points A vector of points in 2D
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
        // TODO (DONE) Part 1.
        if (points.size() <= 1) {
            return points;
        }
        vector<Vector2D> lerp;
        for (int i = 0; i < points.size() - 1; ++i) {
            lerp.push_back((1 - t) * points.at(i) + t * points.at(i + 1));
        }
        return lerp;
    }

    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (function parameter).
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const {
        // TODO (DONE) Part 2.
        if (points.size() <= 1) {
            return points;
        }
        vector<Vector3D> lerp;
        for (int i = 0; i < points.size() - 1; ++i) {
            lerp.push_back((1 - t) * points.at(i) + t * points.at(i + 1));
        }
        return lerp;
    }

    /**
     * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return Final interpolated vector
     */
    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const {
        // TODO (DONE) Part 2.
        if (points.size() == 1) {
            return points.at(0);
        }
        return evaluate1D(BezierPatch::evaluateStep(points, t), t);
    }

    /**
     * Evaluates the Bezier patch at parameter (u, v)
     *
     * @param u         Scalar interpolation parameter
     * @param v         Scalar interpolation parameter (along the other axis)
     * @return Final interpolated vector
     */
    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO (DONE) Part 2.
        // interpolate along u
        vector<Vector3D> interpolate_u;
        for (vector<Vector3D> pt: controlPoints) {
            interpolate_u.push_back(evaluate1D(pt, u));
        }
        // return interpolated along v
        return evaluate1D(interpolate_u, v);
    }

    Vector3D Vertex::normal(void) const {
        // TODO (DONE) Part 3.
        // Returns an approximate unit normal at this vertex, computed by
        // taking the area-weighted average of the normals of neighboring
        // triangles, then normalizing.

        // get current position information
        Vector3D n = Vector3D(0, 0, 0);
        HalfedgeCIter h = halfedge();
        HalfedgeCIter start = halfedge();
        Vector3D p = h->vertex()->position;
        do {
            // get edges of next triangle
            Vector3D p1 = h->twin()->next()->next()->vertex()->position;
            Vector3D p2 = h->twin()->vertex()->position;
            Vector3D v1 = p1 - p;
            Vector3D v2 = p2 - p;
            // normal by area is half the cross product
            Vector3D cross_prod = cross(v1, v2);
            n += cross_prod / 2;
            h = h->twin()->next();
        } while (h != start);
        return n.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // TODO Part 4.
        // This method should flip the given edge and return an iterator to the flipped edge.
        if (e0->isBoundary()) {
            return e0;
        }
        // List of all elements: half-edges, vertices, edges, and faces
        HalfedgeIter bc = e0->halfedge();
        HalfedgeIter ca = bc->next();
        HalfedgeIter ac = ca->twin();
        HalfedgeIter ab = ca->next();
        HalfedgeIter ba = ab->twin();
        HalfedgeIter cb = bc->twin();
        HalfedgeIter bd = cb->next();
        HalfedgeIter db = bd->twin();
        HalfedgeIter dc = bd->next();
        HalfedgeIter cd = dc->twin();
        VertexIter a = ab->vertex();
        VertexIter b = bc->vertex();
        VertexIter c = ca->vertex();
        VertexIter d = dc->vertex();
        EdgeIter e1 = ca->edge();
        EdgeIter e2 = ab->edge();
        EdgeIter e3 = bd->edge();
        EdgeIter e4 = dc->edge();
        FaceIter f0 = bc->face();
        FaceIter f1 = cb->face();
        // Set the pointers
        bc->setNeighbors(dc, cb, a, e0, f0);
        ca->setNeighbors(bc, ac, c, e1, f0);
        ac->setNeighbors(ac->next(), ca, a, e1, ac->face());
        ab->setNeighbors(bd, ba, a, e2, f1);
        ba->setNeighbors(ba->next(), ab, b, e2, ba->face());
        cb->setNeighbors(ab, bc, d, e0, f1);
        bd->setNeighbors(cb, db, b, e3, f1);
        db->setNeighbors(db->next(), bd, d, e3, db->face());
        dc->setNeighbors(ca, cb, d, e4, f0);
        cd->setNeighbors(cd->next(), dc, c, e4, cd->face());
        a->halfedge() = bc;
        b->halfedge() = bd;
        c->halfedge() = ca;
        d->halfedge() = dc;
        e0->halfedge() = bc;
        e1->halfedge() = ca;
        e2->halfedge() = ab;
        e3->halfedge() = bc;
        e4->halfedge() = dc;
        f0->halfedge() = cb;
        f1->halfedge() = bc;
        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 5.
        // This method should split the given edge and return an iterator to the newly inserted vertex.
        // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        return VertexIter();
    }


    void MeshResampler::upsample(HalfedgeMesh &mesh) {
        // TODO Part 6.
        // This routine should increase the number of triangles in the mesh using Loop subdivision.
        // One possible solution is to break up the method as listed below.

        // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // a vertex of the original mesh.

        // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

        // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
        // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
        // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
        // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

        // 4. Flip any new edge that connects an old and new vertex.

        // 5. Copy the new vertex positions into final Vertex::position.

    }
}
