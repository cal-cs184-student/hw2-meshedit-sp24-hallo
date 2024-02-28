#include "student_code.h"
#include "mutablePriorityQueue.h"
#include <iostream>
#include <iomanip>

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
        HalfedgeIter h0 = e0->halfedge(); //bc
        HalfedgeIter h1 = h0->next(); //ca
        HalfedgeIter h2 = h1->next(); //ab
        HalfedgeIter h3 = h0->twin(); //cb
        HalfedgeIter h4 = h3->next(); //bd
        HalfedgeIter h5 = h4->next(); //dc
        HalfedgeIter h6 = h1->twin(); //ac
        HalfedgeIter h7 = h2->twin(); //ba
        HalfedgeIter h8 = h4->twin(); //db
        HalfedgeIter h9 = h5->twin(); //cd
        VertexIter a = h2->vertex();
        VertexIter b = h0->vertex();
        VertexIter c = h1->vertex();
        VertexIter d = h5->vertex();
        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();
        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();

        // Set the pointers
        a->halfedge() = h0;
        b->halfedge() = h4;
        c->halfedge() = h1;
        d->halfedge() = h5;
        e0->halfedge() = h0;
        e1->halfedge() = h1;
        e2->halfedge() = h2;
        e3->halfedge() = h4;
        e4->halfedge() = h5;
        f0->halfedge() = h0;
        f1->halfedge() = h3;
        h0->setNeighbors(h5, h3, a, e0, f0);
        h1->setNeighbors(h0, h6, c, e1, f0);
        h2->setNeighbors(h4, h7, a, e2, f1);
        h3->setNeighbors(h2, h0, d, e0, f1);
        h4->setNeighbors(h3, h8, b, e3, f1);
        h5->setNeighbors(h1, h9, d, e4, f0);
        h6->setNeighbors(h6->next(), h1, a, e1, h6->face());
        h7->setNeighbors(h7->next(), h2, b, e2, h7->face());
        h8->setNeighbors(h8->next(), h4, d, e3, h8->face());
        h9->setNeighbors(h9->next(), h5, c, e4, h9->face());
        return e0;
    }




    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 5.
        // This method should split the given edge and return an iterator to the newly inserted vertex.
        // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        if (e0->isBoundary()) {

            HalfedgeIter h0 = e0->halfedge(); //ba
            if (h0->isBoundary()) {
                h0 = h0->twin();
            }
            HalfedgeIter h3 = h0->twin();
            HalfedgeIter h1 = h0->next(); //ac
            HalfedgeIter h2 = h1->next(); //cb

            VertexIter b = h0->vertex();
            VertexIter a = h1->vertex();
            VertexIter c = h2->vertex();

            EdgeIter e1 = h1->edge(); //ac
            EdgeIter e2 = h2->edge(); //cb

            FaceIter f0 = h2->face(); //abc

            // new midpoint
            VertexIter m = newVertex();
            m ->position = (a->position + b->position) * 0.5;

            // new edges
            EdgeIter e3 = newEdge(); //cm
            e3->isNew = true;
            EdgeIter e4 = newEdge(); //am

            // new halfedges
            HalfedgeIter hma = newHalfedge(); // ma
            HalfedgeIter ham = newHalfedge(); // am
            HalfedgeIter hmc = newHalfedge(); // mc
            HalfedgeIter hcm = newHalfedge(); // cm

            FaceIter f1 = newFace();

            //assign faces
            f1->halfedge() = hcm;
            f0->halfedge() = h2;

            //assign edges
            e3->halfedge() = hcm;
            e4->halfedge() = hma;
            e0->halfedge() = h0;

            // assign vertices
            m->halfedge() = hcm;


            // half edges declare or reassign
            hcm->setNeighbors(hma, hmc, c, e3, f1);
            hmc->setNeighbors(h2, hcm, m, e3, f0);
            hma->setNeighbors(h1, ham, m, e4, f1);
            ham->setNeighbors(h3, hma, a, e4, f1);

            h0->setNeighbors(hmc, h3, b, e0, f0);
            h3->setNeighbors(h3->next(), h0, m, e0, f0);

            h1->next() = hcm;
            h1->face() = f1;

            return m;
        }

        // List of all elements: half-edges, vertices, edges, and faces
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next(); //ca
        HalfedgeIter h2 = h1->next(); //ab
        HalfedgeIter h3 = h0->twin(); //cb
        HalfedgeIter h4 = h3->next(); //bd
        HalfedgeIter h5 = h4->next(); //dc

        VertexIter b = h0->vertex();
        VertexIter c = h1->vertex();

        VertexIter a = h2->vertex();
        VertexIter d = h5->vertex();

        EdgeIter e1 = h1->edge(); //ac
        EdgeIter e2 = h2->edge(); //ab
        EdgeIter e3 = h4->edge(); //bd
        EdgeIter e4 = h5->edge(); //dc

        FaceIter f0 = h2->face(); //abc
        FaceIter f1 = h3->face(); //bdc


        // new midpoint
        VertexIter m = newVertex();
        m ->position = (b->position + c->position) * 0.5;

        // new edges
        EdgeIter e5 = newEdge(); //am
        e5->isNew = true;
        EdgeIter e6 = newEdge(); //md
        e6->isNew = true;
        EdgeIter e7 = newEdge(); //mc
        e7->isNew = false;
        e0->isNew = false;

        // new halfedges
        HalfedgeIter ham = newHalfedge();
        HalfedgeIter hmc = newHalfedge();
        HalfedgeIter hma = newHalfedge();
        HalfedgeIter hmb = newHalfedge();
        HalfedgeIter hmd = newHalfedge();
        HalfedgeIter hdm = newHalfedge();

        FaceIter f2 = newFace(); //acm
        FaceIter f3 = newFace(); // cmd


        //assign faces
        f2->halfedge() = hmc;
        f3->halfedge() = h5;
        f0->halfedge() = h0;
        f1->halfedge() = h4;


        // assign edges
        e5->halfedge() = ham;
        e6->halfedge() = hmd;
        e7->halfedge() = hmc;
        e0->halfedge() = hmb; // mb


        // assign vertices
        m->halfedge() = hmd;

        // half edges declare or reassign
        ham->setNeighbors(hmc, hma, a, e5, f2);

        hma->setNeighbors(h2, ham, m, e5, f0);
        h1->setNeighbors(ham, h1->twin(), c, e1, f2);


        hmc->setNeighbors(h1, h3, m, e7, f2);
        h3->setNeighbors(hmd, hmc, c, e7, f3);
        h5->setNeighbors(h3, h5->twin(), d, e4, f3);


        hmd->setNeighbors(h5, hdm, m, e6, f3);
        hdm->setNeighbors(hmb, hmd, d, e6, f1);
        h4->setNeighbors(hdm, h4->twin(), b, e3, f1);


        hmb->setNeighbors(h4, h0, m, e0, f1);
        h0->setNeighbors(hma, hmb, b, e0, f0);
        h2->setNeighbors(h0, h2->twin(), a, e2, f0);
        return m;
    }


    void MeshResampler::upsample(HalfedgeMesh &mesh) {
        // TODO Part 6.
        // This routine should increase the number of triangles in the mesh using Loop subdivision.
        // One possible solution is to break up the method as listed below.

        // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // a vertex of the original mesh.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
            v->isNew = false;
            float u;
            float n = v->degree();
            if (n == 3) {
                u = 3.0 / 16;
            } else {
                u = 3.0 / (8.0 * n);
            }
            // calculate original_neighbor_position_sum
            HalfedgeIter h0 = v->halfedge();
            HalfedgeIter h = v->halfedge();
            Vector3D original_neighbor_position_sum(0, 0 ,0);
            do {
                original_neighbor_position_sum += h->twin()->vertex()->position;
                h = h->twin()->next();
            } while (h != h0);
            v->newPosition = (1.0 - n * u) * v->position + u * original_neighbor_position_sum;
        }


        // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            e->isNew = false;
            HalfedgeIter h1 = e->halfedge();
            VertexIter a = h1->vertex();
            VertexIter b = h1->twin()->vertex();
            VertexIter d = h1->next()->next()->vertex();
            VertexIter c = h1->twin()->next()->next()->vertex();
            e->newPosition = (3.0 /8.0) * (a->position + b->position) + (1.0/8.0) * (c->position + d->position);
        }

        // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
        // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
        // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
        // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            if (!e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew) {
                VertexIter m = mesh.splitEdge(e);
                m->newPosition = e->newPosition;
                m->isNew = true;

            }
        }

        // 4. Flip any new edge that connects an old and new vertex.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            if (e->isNew) {
                HalfedgeIter h1 = e->halfedge();
                if ((h1->vertex()->isNew && !h1->twin()->vertex()->isNew) ||
                    (!h1->vertex()->isNew && h1->twin()->vertex()->isNew)) {
                    mesh.flipEdge(e);
                    e->isNew = false;
                }
            }
        }
        // 5. Copy the new vertex positions into final Vertex::position.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
            v->position = v->newPosition;
            v->isNew = false;
        }

    }
}
