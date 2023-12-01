/*
** https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification/blob/master/README.md
**
Copyright © 2015-2019 Spacerat and contributors

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================


#include "MeshReducer.h"


#include <algorithm>
#include <chrono>

namespace
{
	using namespace common;
	static constexpr unsigned int PerPool_Unit = 10;
	static constexpr unsigned int PerPool_SIZE = 1 << PerPool_Unit;
    static const unsigned int TIMEOUT = 5;
    static const unsigned int MAX_ITERATION = 100;
    static const double QUADRIC_EPSILON = 1e-15;

	//PLEASE NOTE that this value would affect the final depth map correctness
	//change the value to 0.99*0.99 would greatly reduce the error, but increase the processing time
	//Possible todo: Make those Vertices touch the bounding box as Pivot Vertex
	//Pivot Vertex should not be changed during Mesh Simplification Process
	static const double VALID_THRESHOLD = 0.98f * 0.98f;// 0.9801;//  0.9604;   

    static const double AGGRESSIVE = 8.0;
    static const double AREA_TOLERANCE = 0.999;
    static const double NORMAL_TOLERANCE = 0.2;

    struct Edge;
    struct Face;
    struct FaceNode;

    struct Vertex
    {
        Vertex() : CurrentEdge(ToUpdateEdge) {}

        Vec3 Pos;
        uint16_t ID = 0;  //this ID is fixed since the creation of Vertex

                          // attribute
        bool Removed = false;

        // Need to update?
        Edge *ToUpdateEdge = nullptr;

        // Aliasing name
        Edge *&CurrentEdge;

        // use LOCAL_MARK to update
        unsigned int LOCAL_VERTEX_MARK = 0;

        // Quadric Matrix
        SymetricMatrix Q;

        // adjacent face list
        FaceNode *Neighbors = nullptr;


        void reset()
        {
            LOCAL_VERTEX_MARK = 0;
            //ID = 0;
            ToUpdateEdge = nullptr;
            Neighbors = nullptr;
            Removed = false;
            Q.reset();
        }

        static inline bool comparefunc(Vertex* const &lhs, Vertex* const &rhs)
        {
            return (lhs->Pos < rhs->Pos);
        }
    };

    struct Edge
    {
        Edge() : NextEdge(ToBeReplaced) {}

        // start & end vertex
        Vertex *Start = nullptr;
        Vertex *End = nullptr;

        // collapse property for edge-based simplification
        unsigned int LOCAL_EDGE_MARK = 0;


        double Priority = 0.0;
        Vec3 OptPos;

        // heap related
        int HeapIndex = -1;
        Edge *ToBeReplaced = nullptr;


        //for edge approach face quality calculation
        double SquareDistance = 0;

        // Aliasing name
        Edge *&NextEdge;

        void replaceVertex(Vertex *dst, Vertex *src)
        {
            if (Start == dst)
            {
                Start = src;
            }
            else if (End == dst)
            {
                End = src;
            }
        }


        void initEdge(Vertex *start, Vertex *end)
        {
            // update vertices
            Start = start;
            End = end;

            // adjacent faces
            //AdjFaces = 0;

            // attributes
            LOCAL_EDGE_MARK = 1; //LOCAL_MARK  is used to record edge degree at very beginning
                                 //Priority = 0.0;  //no need to reset this
                                 //HeapIndex = -1;  //no need to reset this
                                 //ToBeReplaced = nullptr;   //no need to reset this  auto set during initialization
        }
    };

    struct FaceNode
    {
        Face *f = nullptr; //f contain more info than instance, f is common refer to face, instance could be anything
        FaceNode *Next = nullptr;
    };

    struct Face
    {
        Face()
        {
            FaceNodes[0].f = this;
            FaceNodes[1].f = this;
            FaceNodes[2].f = this;
        }

        void resetFace()
        {
            Valid = true;
        }

        // adjacent vertices
        Vertex *Vertices[3] = { nullptr };

        // edges
        Edge *Edges[3] = { nullptr };


        // related face node
        FaceNode FaceNodes[3];
        // valid & dirty(need to update)
        bool Valid = true;


        // collapse property for fast simplification
        uint16_t OptEdge = 0;
        unsigned int LOCAL_FACE_MARK = 0; //to check whether face are updated for current iteration

                                          // Priority
                                          //double priority = 0;

        Vec3 InitUnitNormal;
        // get connected edges
        void getEdgesByVertex(Vertex *v0, Edge *&e1, Edge *&e2)
        {
            for (uint32_t i = 0; i < 3; ++i)
            {
                if (v0 == Vertices[i])
                {
                    e1 = Edges[(i + 2) % 3];
                    e2 = Edges[i];
                    break;
                }
            }
        }

        void getV1V2ByV0(Vertex *v0, Vertex *&v1, Vertex *&v2) const
        {
            for (uint32_t i = 0; i < 3; ++i)
            {
                if (v0 == Vertices[i])
                {
                    v1 = Vertices[(i + 1) % 3];
                    v2 = Vertices[(i + 2) % 3];
                    break;
                }
            }
        }

        void replace(Vertex *dst, Vertex *src)
        {
            for (uint32_t i = 0; i < 3; ++i)
            {
                if (dst == Vertices[i])
                {
                    Vertices[i] = src;
                    Edges[i]->replaceVertex(dst, src);
                    Edges[(i + 2) % 3]->replaceVertex(dst, src);
                    break;
                }
            }
        }

        bool containVertex(Vertex *v) const
        {
            return Vertices[0] == v || Vertices[1] == v || Vertices[2] == v;
        }

        Vec3 normal() const
        {
            const Vec3 &v0 = Vertices[0]->Pos;
            const Vec3 &v1 = Vertices[1]->Pos;
            const Vec3 &v2 = Vertices[2]->Pos;

            const Vec3 &e0 = v1 - v0;
            const Vec3 &e1 = v2 - v0;

            return e0.cross(e1);
        }

        double computeQuality(Vertex* start, const Vec3 &optPos)
        {
            Edge* e = this->getOppositeEdgeByV0_Safe(start);


            double len_e0 = e->SquareDistance;
            double len_e1 = Vec3::squaredLength(e->Start->Pos, optPos);
            double len_e2 = Vec3::squaredLength(e->End->Pos, optPos);


            double max_edge = std::max(len_e0, std::max(len_e1, len_e2));

            ////https://en.wikipedia.org/wiki/Heron%27s_formula
            //// A = \frac1{ 2 }\sqrt{ a ^ 2c ^ 2 - \left(\frac{ a ^ 2 + c ^ 2 - b ^ 2 }{2}\right) ^ 2 }
            double a2b2c2_2 = (len_e0 + len_e2 - len_e1)*0.5;
            double areaSquare = len_e0 * len_e2 - a2b2c2_2 * a2b2c2_2;

            ////    const Vec3 &e0 = v2->Pos - v1->Pos;
            ////    const Vec3 &e1 = v1->Pos - optPos;
            ////    const Vec3 &normal = e0.cross(e1);
            ////    double len_norm = normal.squaredLength();  //areaSquare == len_norm 

            return (areaSquare) / (max_edge * max_edge);
        }

        // get opposite edges
        void getEdgesByVertex(Vertex* v0, Vertex* v1, Edge*& v2v1, Edge*& v2v0, Vertex*& v2)
        {
            if (Vertices[0] == v0)
            {
                if (Vertices[1] == v1)  //v0 0 v1 1 v2 2
                {
                    v2v1 = this->Edges[1];
                    v2v0 = this->Edges[2];
                    v2 = Vertices[2];
                }
                else //v1 -> 2        //v0 0  v2 1 v1 2
                {
                    v2v0 = this->Edges[0];
                    v2v1 = this->Edges[1];
                    v2 = Vertices[1];
                }
            }
            else if (Vertices[1] == v0)
            {
                if (Vertices[2] == v1)  //v2 0  v0 1 v1 2
                {
                    v2v1 = this->Edges[2];
                    v2v0 = this->Edges[0];
                    v2 = Vertices[0];
                }
                else  //v1-> 0          //v1 0  v0 1 v2 2
                {
                    v2v1 = this->Edges[2];
                    v2v0 = this->Edges[1];
                    v2 = Vertices[2];
                }
            }
            else
            {// if (Vertices[2] == v0) {
                if (Vertices[0] == v1)  //v1 0  v2 1 v0 2
                {
                    v2v1 = this->Edges[0];
                    v2v0 = this->Edges[1];
                    v2 = Vertices[1];
                }
                else                    //v2 0  v1 1 v0 2
                {
                    v2v1 = this->Edges[0];
                    v2v0 = this->Edges[2];
                    v2 = Vertices[0];
                }
            }
        }

        void replaceDuplicatedEdge()
        {
            for (uint32_t i = 0; i < 3; ++i)
            {
                if (Edges[i]->ToBeReplaced)
                {
                    Edges[i] = Edges[i]->ToBeReplaced;
                }
            }
        }

        void  markV2V1ReplacedByV2V0(Vertex* v0, Vertex* v1, Vertex* &v2)
        {
            Edge* v2v1 = nullptr;
            Edge* v2v0 = nullptr;
            this->getEdgesByVertex(v0, v1, v2v1, v2v0, v2);
            v2v1->ToBeReplaced = v2v0;
            //return v2v1;
        }
        Edge*  getV2V1ReplacedByV2V0(Vertex* v0, Vertex* v1, Vertex* &v2)
        {
            Edge* v2v1 = nullptr;
            Edge* v2v0 = nullptr;
            this->getEdgesByVertex(v0, v1, v2v1, v2v0, v2);
            v2v1->ToBeReplaced = v2v0;
            return v2v1;
        }



    private:
        Edge* getOppositeEdgeByV0_Safe(Vertex* v0) const
        {
            if (v0 == Vertices[0])
            {
                return Edges[1];
            }
            else if (v0 == Vertices[1])
            {
                return Edges[2];
            }
            else //if (v0 == Vertices[2])
            {
                return Edges[0];
            }
        }
    };

    struct EdgeHeap
    {
        EdgeHeap(std::vector<Edge*> &Edges) : Container(Edges), Length(static_cast<int>(Edges.size()))
        {
            //how to get
            for ( int idx = 1; idx < Length; ++idx)
            {
                initElement(idx);
            }
            for ( int i = 0; i < Length; ++i)
            {
                Container[i]->HeapIndex = i;
            }
        }

        void initElement(int idx)
        {
            int parent = (idx - 1) >> 1;
            auto e = Container[idx];
            // heap up
            while (e->Priority < Container[parent]->Priority)
            {
                // swap and keep indexes updated
                Container[idx] = Container[parent];

                // top
                if (parent == 0)
                {
                    Container[0] = e;
                    return;
                }

                idx = parent;
                parent = (idx - 1) >> 1;
            }

            Container[idx] = e;
            return;
        }

        Edge *top()
        {
            Edge *item = Container[0];
            // mark as not inside heap anymore
            item->HeapIndex = -1;
            if (Length > 1)
            {
                // move the last one to top
                Container[0] = Container[Length - 1];
                Container[0]->HeapIndex = 0;
                // heap down
                heapifyDown(Container[0]);
            }

            // Remove the last element
            --Length;
            return item;
        }

        void update(Edge *e, double prev_priority)
        {
            if (e->Priority < prev_priority)
            {
                heapifyUp(e);
            }
            else if (e->Priority > prev_priority)
            {
                heapifyDown(e);
            }
        }

        void pop(Edge *edge)
        {
            // get current index
            int cur_idx = edge->HeapIndex;
            if (cur_idx < 0 || cur_idx >= Length || edge != Container[cur_idx])
            {
                return;
            }

            // mark as removed
            edge->HeapIndex = -2;
            if (cur_idx != (Length - 1))
            {
                auto last = Container[Length - 1];
                Container[cur_idx] = last;
                last->HeapIndex = cur_idx;
                if (last->Priority < edge->Priority)
                {
                    heapifyUp(last);
                }
                else if (last->Priority > edge->Priority)
                {
                    heapifyDown(last);
                }
            }

            // remove
            --Length;
        }

        void heapifyUp(Edge *e)
        {
            if (e->HeapIndex == 0)
            {
                return;
            }

            // get current index & parent
            int idx = e->HeapIndex;
            int parent = (idx - 1) >> 1;

            // heap up
            while (e->Priority < Container[parent]->Priority)
            {
                // swap
                std::swap(Container[idx], Container[parent]);
                Container[idx]->HeapIndex = idx;
                Container[parent]->HeapIndex = parent;

                // top
                if (parent == 0)
                {
                    return;
                }

                // update index
                idx = parent;
                parent = (idx - 1) >> 1;
            }
        }

        void heapifyDown(Edge *e)
        {
            int pos = e->HeapIndex;

            // get left & right children
            int left = (pos << 1) + 1;
            if (left >= Length)
            {
                return;
            }

            int right = left + 1;
            int smallest = pos;
            while (1)
            {
                if (Container[left]->Priority < e->Priority)
                {
                    if (right < Length && Container[right]->Priority < Container[left]->Priority)
                    {
                        smallest = right;
                    }
                    else
                    {
                        smallest = left;
                    }
                }
                else if (right < Length && Container[right]->Priority < e->Priority)
                {
                    smallest = right;
                }
                else
                {
                    Container[pos] = e;
                    e->HeapIndex = pos;
                    return;
                }

                // swap
                std::swap(Container[pos], Container[smallest]);
                Container[pos]->HeapIndex = pos;
                Container[smallest]->HeapIndex = smallest;

                left = (smallest << 1) + 1;
                if (left >= Length)
                {
                    return;
                }

                pos = smallest;
                right = left + 1;
            }
        }

        std::vector<Edge *> &Container;
        int Length = 0;
    };

    class CollapseHelper {
	private:

		// Face pool to reduce memory reallocation
		std::vector<Face *> FacePoolKeys;

		// Edge pool to reduce memory reallocation
		std::vector<Edge *> EdgePoolKeys;
		uint32_t EdgePoolIdx;

		// Edge pool to reduce memory reallocation
		std::vector<Vertex *> VertexPoolKeys;

    public:
        unsigned int GLOBAL_MARK;
        double ScaleFactor;


        void reset()
        {
            GLOBAL_MARK = 0;
            //ScaleFactor = 1.0;
            EdgePoolIdx = 0;
        }

        Edge *spawnEdgeFromPool(Vertex *v0, Vertex *v1)
        {
			if (EdgePoolIdx >= (EdgePoolKeys.size() << PerPool_Unit))
			{
				Edge* pool = new Edge[PerPool_SIZE];
				EdgePoolKeys.push_back(pool);
            }
			int row = EdgePoolIdx >> PerPool_Unit;
			int col = EdgePoolIdx & (PerPool_SIZE - 1);

            Edge* e = &EdgePoolKeys[row][col];
			EdgePoolIdx++;
            e->initEdge(v0, v1);
            return e;
        }

        void reservePool(unsigned int nVert, unsigned int nFace, std::vector<Vertex*>& Vertices, std::vector<Face*>& Faces)
        {
            // allocate memory for vertex
            while (nVert > (VertexPoolKeys.size() << PerPool_Unit) )
            {
                Vertex *v = new Vertex[PerPool_SIZE];
                uint16_t previousVertexPoolSize = static_cast<uint16_t>( VertexPoolKeys.size() << PerPool_Unit);
                for (unsigned int i_vert = 0; i_vert < PerPool_SIZE; ++i_vert)
                {
                    v[i_vert].ID = previousVertexPoolSize++;
				}
				VertexPoolKeys.push_back(v);
            }
			
            // allocate memory for face
            while (nFace > (FacePoolKeys.size()<< PerPool_Unit) )
            {
				Face *f = new Face[PerPool_SIZE];
				FacePoolKeys.push_back(f);                
            }

			copyVertexData(Vertices, nVert);
			copyFaceData(Faces, nFace);
        }

		void copyEdgeData(std::vector<Edge *>& Edges)
		{
			Edges.resize(EdgePoolIdx);


			int row = EdgePoolIdx >> PerPool_Unit;
			int col = EdgePoolIdx & (PerPool_SIZE - 1);
			for (int idx = 0; idx < row; idx++)
			{
				auto startE = &Edges[idx << PerPool_Unit];
				auto targetE = EdgePoolKeys[idx];
				for (int i = 0; i < PerPool_SIZE; i++) {
					startE[i] = &targetE[i];
				}
			}
			if (col > 0)
			{
				auto startE = &Edges[row << PerPool_Unit];
				auto targetE = EdgePoolKeys[row];
				for (int i = 0; i < col; i++) 
				{
					startE[i] = &targetE[i];
				}
			}


		}

        // cost = VT * Q * V
        double getQuadricCost(const Vec3 &v, const SymetricMatrix &Q)
        {
            double x = v.X;
            double y = v.Y;
            double z = v.Z;
            // Q[0] * x * x + 2 * Q[1] * x * y + 2 * Q[2] * x * z + 2 * Q[3] * x + Q[4] * y * y + 2 * Q[5] * y * z + 2 * Q[6] * y + Q[7] * z * z + 2 * Q[8] * z + Q[9];
            return (Q[0] * x + 2 * Q[1] * y + 2 * Q[2] * z + 2 * Q[3]) * x +
                (Q[4] * y + 2 * Q[5] * z + 2 * Q[6]) * y + (Q[7] * z + 2 * Q[8]) * z + Q[9];
        }

        // priority = cost / (normal * tri_quality)
        double computePriority(Edge& e, double QuadricCost)
        {
            const Vec3 &optPos = e.OptPos;
            auto start = e.Start;
            auto end = e.End;

            // for each face related to start vertex
            double minQual = 1;
            for (auto fn = start->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto face = fn->f;
                if (face->containVertex(end))
                {
                    continue;
                }

                double quality = face->computeQuality(start, optPos);
                if (quality == 0)
                {
                    return std::numeric_limits<double>::infinity();
                }
                minQual = std::min(minQual, quality);
            }

            // for each face related to end vertex
            for (auto fn = end->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto face = fn->f;
                if (face->containVertex(start))
                {
                    continue;
                }

                double quality = face->computeQuality(end, optPos);
                if (quality == 0)
                {
                    return std::numeric_limits<double>::infinity();
                }
                minQual = std::min(minQual, quality);
            }

            // cost
            double cost = ScaleFactor * QuadricCost;
            if (cost <= QUADRIC_EPSILON)
            {
                cost = -1 / sqrt((start->Pos - end->Pos).squaredLength() * minQual);
            }
            else
            {
                cost /= sqrt(minQual);
            }
            return cost;
        }

        void calcOptimalPosition(Edge &e, double &cost)
        {
            static const double COST_THRESHOLD = 200.0 * QUADRIC_EPSILON;
            auto start = e.Start;
            auto end = e.End;

            const SymetricMatrix &Q = start->Q + end->Q;
            Vec3 &optPos = e.OptPos;
            optPos = (start->Pos + end->Pos) / 2.0;
            // the geo meaning of getQuadricCost is the distance sum to plains
            // thus, small negative value should be the numeric error
            cost = getQuadricCost(optPos, start->Q) + getQuadricCost(optPos, end->Q);
            //cost = getQuadricCost(optPos, Q);// +getQuadricCost(optPos, end->Q);
            if (cost > COST_THRESHOLD)
            {
                if (!Q.solve(optPos))
                {
                    // calculate the cost
                    double cost0 = getQuadricCost(start->Pos, Q);
                    double cost1 = getQuadricCost(end->Pos, Q);

                    cost = std::min(cost0, std::min(cost1, cost));
                    if (cost == cost0)
                    {
                        optPos = start->Pos;
                    }
                    else if (cost == cost1)
                    {
                        optPos = end->Pos;
                    }
                    // else use mid point
                    return;
                }
                cost = getQuadricCost(optPos, Q);
            }
        }
        void calcOptimalPosition_ForFace(Edge& e, double &cost)
        {
            static const double COST_THRESHOLD = 200.0 * QUADRIC_EPSILON;
            auto start = e.Start;
            auto end = e.End;

            const SymetricMatrix &Q = start->Q + end->Q;
            Vec3 &optPos = e.OptPos;
            optPos = (start->Pos + end->Pos) / 2.0;
            // the geo meaning of getQuadricCost is the distance sum to plains
            // thus, small negative value should be the numeric error
            //cost = getQuadricCost(optPos, start->Q) + getQuadricCost(optPos, end->Q);
            cost = getQuadricCost(optPos, Q);//this equivalent to  getQuadricCost(optPos, start->Q) + getQuadricCost(optPos, end->Q);
            if (cost > COST_THRESHOLD)
            {
                if (!Q.solveForFace(optPos, cost))
                {
                    // calculate the cost
                    double cost0 = getQuadricCost(start->Pos, Q);
                    double cost1 = getQuadricCost(end->Pos, Q);

                    cost = std::min(cost0, std::min(cost1, cost));
                    if (cost == cost0)
                    {
                        optPos = start->Pos;
                    }
                    else if (cost == cost1)
                    {
                        optPos = end->Pos;
                    }
                    return;
                }
            }
            return;

        }

        void solve(Face &face)
        {
            for (unsigned int j = 0; j < 3; ++j)
            {
                Edge* e = face.Edges[j];

                //if edge mark is not larger than any of end mark, need to recalculate
                if (e->LOCAL_EDGE_MARK <= e->Start->LOCAL_VERTEX_MARK || e->LOCAL_EDGE_MARK <= e->End->LOCAL_VERTEX_MARK)
                {
                    e->LOCAL_EDGE_MARK = GLOBAL_MARK;
                    calcOptimalPosition_ForFace(*e, e->Priority);
                }
            }
            computeFaceOptEdge(face);
        }
        void computeFaceOptEdge(Face &face)
        {
            face.LOCAL_FACE_MARK = GLOBAL_MARK;

            double p = face.Edges[0]->Priority;
            face.OptEdge = 0;
            if (face.Edges[1]->Priority < face.Edges[0]->Priority)
            {
                face.OptEdge = 1;
                p = face.Edges[1]->Priority;
            }

            if (face.Edges[2]->Priority < p)
            {
                face.OptEdge = 2;
            }
        }


        void solve(Edge &edge)
        {
            if (edge.SquareDistance == 0)
            {
                edge.OptPos = edge.Start->Pos;
                edge.Priority = -std::numeric_limits<double>::infinity();
            }

            double cost = 0.0;
            calcOptimalPosition(edge, cost);
            edge.Priority = computePriority(edge, cost);
        }

        bool flipped(Vertex *start, Vertex *end, const Vec3 &optPos)
        {
            if (optPos == start->Pos)
            {
                return false;
            }
            for (auto fn = start->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto neighbor = fn->f;
                if (neighbor->containVertex(end))
                {
                    continue;
                }

                Vertex *v1 = nullptr;
                Vertex *v2 = nullptr;
                neighbor->getV1V2ByV0(start, v1, v2);
                const Vec3 &d1 = (v1->Pos - optPos).normalize();
                const Vec3 &d2 = (v2->Pos - optPos).normalize();

                if (std::fabs(d1.dot(d2)) > AREA_TOLERANCE)
                {
                    return true;
                }

                const Vec3 &unitNormal = (d1.cross(d2)).normalize();
                if (unitNormal.dot(neighbor->InitUnitNormal) < NORMAL_TOLERANCE)
                {
                    return true;
                }
            }
            return false;
        }

        unsigned int updateFace(Face &face)
        {
            // get vertices
            auto e = face.Edges[face.OptEdge];
            Vertex* v0 = e->Start;
            Vertex* v1 = e->End;
            const Vec3 &optPos = e->OptPos;

            if (flipped(v0, v1, optPos) || flipped(v1, v0, optPos))
            {
                return 0;
            }

            swapV0V1IfV0DegreeSmaller(v0, v1);

            // mark as Removed
            v1->Removed = true;

            // update v1 faces
            unsigned int nDeleted = 0;
            for (auto fn = v1->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto f = fn->f;

                // try to remove the face
                if (f->containVertex(v0))
                {
                    nDeleted++;
                    f->Valid = false;

                    Vertex* v2 = nullptr;
                    f->markV2V1ReplacedByV2V0(v0, v1, v2);
                    // v2->Neighbors->clear();
                    removeInvalidFace(v2);
                }
            }
            // v0->Neighbors = v0->Neighbors->import(v1);
            mergeV0V1ValidFacesIntoV0(v0, v1);
            if (v0->Neighbors == nullptr)
            {
                v0->Removed = true;
                return nDeleted;
            }

            GLOBAL_MARK++;
            // use v0 to store new vertex
            v0->Pos = optPos;

            // update v0
            v0->Q += v1->Q;
            v0->LOCAL_VERTEX_MARK = GLOBAL_MARK;
            for (auto fn = v1->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto f = fn->f;
                f->replace(v1, v0);
                f->replaceDuplicatedEdge();
            }

            // increase GLOBAL_MARK to update the faces
            GLOBAL_MARK++;
            for (auto fn = v0->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto f = fn->f;
                // update
                CollapseHelper::solve(*f);
            }
            return nDeleted;
        }

        unsigned int updateEdge(EdgeHeap &heap, Edge &edge)
        {
            // edge cache
            static std::vector<Edge *> EdgeCache;

            // update global mark
            GLOBAL_MARK++;

            // get vertex
            Vertex *v0 = edge.Start;
            Vertex *v1 = edge.End;

            swapV0V1IfV0DegreeSmaller(v0, v1);

            //prefer to use the one with larger capacity
            v1->Removed = true;

            // update v1 faces
            unsigned int nDeleted = 0;
            for (auto fn = v1->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto f = fn->f;
                // remove the face
                if (f->containVertex(v0))
                {
                    // set face as invalid
                    f->Valid = false;

                    // get another edge connected to v1, then remove it.
                    Vertex* v2 = nullptr;
                    auto v2v1 = f->getV2V1ReplacedByV2V0(v0, v1, v2);
                    if (v2v1->HeapIndex != -2)
                    {
                        heap.pop(v2v1);
                    }

                    // v2->Neighbors->clear();
                    removeInvalidFace(v2);

                    nDeleted++;
                    continue;
                }
            }

            // v0->Neighbors = v0->Neighbors->import(v1);
            mergeV0V1ValidFacesIntoV0(v0, v1);
            if (v0->Neighbors == nullptr) {
                v0->Removed = true;
                return nDeleted;
            }

            // update v0 neighbors
            for (auto fn = v1->Neighbors; fn != nullptr; fn = fn->Next)
            {
                auto f = fn->f;

                // replace
                f->replace(v1, v0);
                f->replaceDuplicatedEdge();
            }

            // use v0 to store new vertex
            v0->Pos = edge.OptPos;
            v0->LOCAL_VERTEX_MARK = GLOBAL_MARK;
            v0->Q += v1->Q;


            for (auto fn = v0->Neighbors; fn != nullptr; fn = fn->Next)
            {
                Edge *e1 = nullptr;
                Edge *e2 = nullptr;
                auto f = fn->f;
                f->getEdgesByVertex(v0, e1, e2);
                syncIntoEdgeCache(e1, v0, EdgeCache);
                syncIntoEdgeCache(e2, v0, EdgeCache);
                if (e1->ToBeReplaced != nullptr || e2->ToBeReplaced != nullptr)
                {
                    f->replaceDuplicatedEdge();
                }
            }
            for (auto e : EdgeCache)
            {
                if (e->ToBeReplaced == nullptr)
                {
                    e->SquareDistance = Vec3::squaredLength(e->Start->Pos, e->End->Pos);
                }
            }
            for (auto e : EdgeCache)
            {
                if (e->ToBeReplaced != nullptr)
                {
                    heap.pop(e);
                }
                else
                {
                    double old_priority = e->Priority;
                    solve(*e);
                    heap.update(e, old_priority);
                    e->Start->ToUpdateEdge = nullptr;
                }
            }
            EdgeCache.clear();
            return nDeleted;
        }

        void release()
        {
            // Pool is allocated as array, use delete [].
			for (auto key : FacePoolKeys)
			{
				delete[] key;
			}
			for (auto key : VertexPoolKeys)
			{
				delete[] key;
			}
			for (auto key : EdgePoolKeys)
			{
				delete[] key;
			}

            FacePoolKeys.clear();
            VertexPoolKeys.clear();
            EdgePoolKeys.clear();
        }

        void syncIntoEdgeCache(Edge *e1, Vertex *v0, std::vector<Edge *> &EdgeCache)
        {
            // force to be same topology
            if (e1->Start == v0)
            {
                e1->Start = e1->End;
                e1->End = v0;
            }

            if (e1->LOCAL_EDGE_MARK != GLOBAL_MARK)
            {
                if (e1->Start->ToUpdateEdge != nullptr && e1->Start->ToUpdateEdge != e1)
                {
                    e1->ToBeReplaced = e1->Start->ToUpdateEdge;
                }
                else
                {
                    e1->Start->ToUpdateEdge = e1;
                }
                e1->LOCAL_EDGE_MARK = GLOBAL_MARK;
                EdgeCache.push_back(e1);
            }
        }

        void removeInvalidFace(Vertex *v)
        {
            auto lastValid = v->Neighbors;
            for (auto fn = lastValid->Next; fn != nullptr; fn = fn->Next)
            {
                if (!fn->f->Valid)
                {
                    continue;
                }
                lastValid->Next = fn;
                lastValid = fn;
            }
            lastValid->Next = nullptr;

            if (v->Neighbors->f->Valid == false)
            {
                v->Neighbors = v->Neighbors->Next;
                if (v->Neighbors == nullptr)
                    v->Removed = true;
            }
        }


        void swapV0V1IfV0DegreeSmaller(Vertex*& v0, Vertex*& v1)
        {
            auto f0 = v0->Neighbors;
            auto f1 = v1->Neighbors;
            for (; f0 != nullptr && f1 != nullptr; f0 = f0->Next, f1 = f1->Next)
            {

            }
            //f0 has smaller degree
            if (f0 == nullptr && f1 != nullptr)
            {
                std::swap(v0, v1);
            }
        }

        void mergeV0V1ValidFacesIntoV0(Vertex *v0, Vertex *v1)
        {
            auto lastValid = v0->Neighbors;
            for (auto fn = lastValid->Next; fn != nullptr; fn = fn->Next)
            {
                if (!fn->f->Valid) {
                    continue;
                }
                lastValid->Next = fn;
                lastValid = fn;
            }

            while (v1->Neighbors != nullptr && v1->Neighbors->f->Valid == false)
            {
                v1->Neighbors = v1->Neighbors->Next;
            }
            lastValid->Next = v1->Neighbors;
            if (v1->Neighbors != nullptr)
            {
                for (auto fn = lastValid->Next; fn != nullptr; fn = fn->Next)
                {
                    if (!fn->f->Valid)
                    {
                        continue;
                    }
                    lastValid->Next = fn;
                    lastValid = fn;
                }
                lastValid->Next = nullptr;
            }

            if (v0->Neighbors->f->Valid == false)
            {
                v0->Neighbors = v0->Neighbors->Next;
            }
        }




		size_t getMemory()
		{
			size_t faceM = FacePoolKeys.size() * PerPool_SIZE * sizeof(Face);
			size_t edgeM = EdgePoolKeys.size() * PerPool_SIZE * sizeof(Edge);
			size_t vertexM = VertexPoolKeys.size() * PerPool_SIZE * sizeof(Vertex);
			return  faceM + edgeM + vertexM+
				FacePoolKeys.capacity() * sizeof(Face*) +
				EdgePoolKeys.capacity() * sizeof(Edge*) +
				VertexPoolKeys.capacity() * sizeof(Vertex*);
		}

private:

	void copyVertexData(std::vector<Vertex *>& Vertices, unsigned int nVert)
	{
		Vertices.resize(nVert);
		
		int row = nVert  >> PerPool_Unit;
		int col = nVert & (PerPool_SIZE - 1);
		for (int idx = 0; idx < row; idx++)
		{
			auto startV = &Vertices[idx << PerPool_Unit];
			auto targetV = VertexPoolKeys[idx];
			for (int i = 0; i < PerPool_SIZE; i++) {
				startV[i] = &targetV[i];
			}
		}
		if (col > 0)
		{
			auto startV = &Vertices[row << PerPool_Unit];
			auto targetV = VertexPoolKeys[row];
			for (int i = 0; i < col; i++) {
				startV[i] = &targetV[i];
			}
		}
	}

	void copyFaceData(std::vector<Face *>& Faces, unsigned int nFace)
	{
		Faces.resize(nFace);

		int row = nFace >> PerPool_Unit;
		int col = nFace & (PerPool_SIZE - 1);
		for (int idx = 0; idx < row; idx++)
		{
			auto startV = &Faces[idx << PerPool_Unit];
			auto targetV = CollapseHelper::FacePoolKeys[idx];
			for (int i = 0; i < PerPool_SIZE; i++) {
				startV[i] = &targetV[i];
			}
		}
		if (col > 0)
		{
			auto startV = &Faces[row << PerPool_Unit];
			auto targetV = CollapseHelper::FacePoolKeys[row];
			for (int i = 0; i < col; i++) {
				startV[i] = &targetV[i];
			}
		}
	}


	};

    class MeshReducerPrivate
    {
    public:
		CollapseHelper mHelper;
        MeshReducerPrivate();

        void reset();
        void reduce(unsigned int nTarget);

        void load(const float *vertices, const uint16_t *indices, unsigned int nVert, unsigned int nInd, bool forceStrict);
        void store(float *vertices, int&reducedVertNum, uint16_t* reducedIndices, int &reducedFaceNum);

        bool isNonClosedMesh() const;
        bool isValid() const;
		void reserveVerticesFaces(uint32_t MaxOccluderVerticeNum, uint32_t MaxFaceNum)
		{
			Vertices.reserve(MaxOccluderVerticeNum);
			Faces.reserve(MaxFaceNum);
		}

		size_t getMemory()
		{
			return mHelper.getMemory() + 
				Vertices.capacity() * sizeof(Vertex*) +
				Faces.capacity() * sizeof(Face*) +
				Edges.capacity() * sizeof(Edge*) +
				sizeof(FaceIterThresholds);
		}
    private:
        void buildQuadricMatrix();
        void initCollapses();
        void doFastLoop(unsigned int nTarget);
        void doStrictLoop(unsigned int nTarget);

        void cleanUp();
    private:

        double FaceIterThresholds[MAX_ITERATION];
        // non-manifold
        bool m_bStrictConstraint = false;

        // data section
        std::vector<Vertex *> Vertices;
        std::vector<Face *> Faces;
        std::vector<Edge *> Edges;

        // Bounding box Diagonal
        double OriginBBoxDiagonal = 0.0;
    };

    MeshReducerPrivate::MeshReducerPrivate() {
        for (unsigned int iter = 0; iter < MAX_ITERATION; ++iter)
        {
            FaceIterThresholds[iter] = 1e-9 * static_cast<double>(std::pow(iter + 3, AGGRESSIVE));
        }
    }


    void MeshReducerPrivate::reset()
    {
        m_bStrictConstraint = false;
        Vertices.clear();
        Faces.clear();
        Edges.clear();
		mHelper.reset();
		mHelper.release();
    }

    void MeshReducerPrivate::reduce(unsigned int nTarget) {

        // build Quadric matrix for each vertex
        buildQuadricMatrix();

        // compute the new position and quadric error
        initCollapses();

        // loop
        if (m_bStrictConstraint)
        {
            doStrictLoop(nTarget);
        }
        else
        {
            doFastLoop(nTarget);
        }

        // clean up
        cleanUp();
    }

    void MeshReducerPrivate::load(const float *vertices, const uint16_t *indices, unsigned int nVert, unsigned int nInd, bool forceStrict)
    {
        if (!vertices || !indices || !nVert || !nInd)
        {
            return;
        }

        // reserve first
        unsigned int nFace = nInd / 3;

		mHelper.reservePool(nVert, nFace, Vertices, Faces);

        if (forceStrict == true)//the first time already calculate max min
        {
            for (unsigned int i_vert = 0, i_vert_idx = 0; i_vert < nVert; ++i_vert, i_vert_idx += 3)
            {
                double x = vertices[i_vert_idx];
                double y = vertices[i_vert_idx + 1];
                double z = vertices[i_vert_idx + 2];

                auto vert = this->Vertices[i_vert];
                vert->reset();
                vert->Pos = Vec3(x, y, z);
            }
        }
        else
        {
			// build vertices
			Vec3 min(std::numeric_limits<double>::max());
			Vec3 max(std::numeric_limits<double>::min());
			for (unsigned int i_vert = 0, i_vert_idx = 0; i_vert < nVert; ++i_vert, i_vert_idx += 3)
			{

				double x = vertices[i_vert_idx];
				double y = vertices[i_vert_idx + 1];
				double z = vertices[i_vert_idx + 2];

				// get vertex from pool
				auto vert = this->Vertices[i_vert];
				vert->reset();
				vert->Pos = Vec3(x, y, z);
				//vert->ID = i_vert;

				// get scale
				min.X = std::min(min.X, x);
				max.X = std::max(max.X, x);

				min.Y = std::min(min.Y, y);
				max.Y = std::max(max.Y, y);

				min.Z = std::min(min.Z, z);
				max.Z = std::max(max.Z, z);
			}

            //CollapseHelper::ScaleFactor = double(1e8 * std::pow(1.0 / double((max - min).length()), 6));
            OriginBBoxDiagonal = (max - min).squaredLength();
			mHelper.ScaleFactor = double(1e8 * std::pow(1.0 / OriginBBoxDiagonal, 3));
        }



        uint32_t notDegreeOneEdgeCount = 0;
        for (unsigned int idx = 0, fCounter = 0; idx < nFace; ++idx)
        {
            auto face = this->Faces[idx];
            face->resetFace();
            // create vertex neighbor list
            for (unsigned int i_vert = 0; i_vert < 3; ++i_vert)
            {
                auto vert = Vertices[indices[fCounter++]];
                face->Vertices[i_vert] = vert;
                FaceNode* node = &face->FaceNodes[i_vert];
                //node->Instance = face; //the node is bound with face during initialization
                node->Next = vert->Neighbors;
                vert->Neighbors = node;
            }

            for (unsigned int i_vert = 0; i_vert < 3; ++i_vert)
            {
                Vertex *v0 = face->Vertices[i_vert];
                Vertex *v1 = face->Vertices[(i_vert + 1) % 3];

                if (v0->ID > v1->ID)
                {
                    std::swap(v0, v1);
                }

                Edge* e = nullptr;
                // ToUpdateEdge and ToBeReplaced are used as Current & Next Edge
                // as a list structure
                Edge * possibleE = v0->CurrentEdge;
                while (possibleE != nullptr)
                {
                    if (possibleE->End == v1)
                    {
                        e = possibleE;
                        break;
                    }
                    possibleE = possibleE->NextEdge;
                }

                if (e == nullptr)
                {
                    e = mHelper.spawnEdgeFromPool(v0, v1);


                    e->NextEdge = v0->CurrentEdge;
                    v0->CurrentEdge = e;
                }
                else {
                    if (e->LOCAL_EDGE_MARK == 1)
                    {
                        e->LOCAL_EDGE_MARK--;
                        notDegreeOneEdgeCount++;
                    }
                }
                face->Edges[i_vert] = e;
            }
        }
		mHelper.copyEdgeData(Edges);


        // manifold or non-manifold
        m_bStrictConstraint = forceStrict || (Edges.size() > notDegreeOneEdgeCount);

    }
    //it is confusing as Vertices is used. and vertices as result. rename vertices to outVerts
    void MeshReducerPrivate::store(float * reducedVertices, int &reducedVertNum, uint16_t* reducedIndices, int &reducedFaceNum )
    {
		reducedVertNum = (int)Vertices.size();
		int reducedIdx = 0;
        for (auto vert : Vertices)
		{
			reducedVertices[reducedIdx] = (float)vert->Pos.X;
			reducedVertices[reducedIdx+1] = (float)vert->Pos.Y;
			reducedVertices[reducedIdx + 2] = (float)vert->Pos.Z;
			reducedIdx += 3;
        }

        int validFace = 0;
        for (auto face : Faces)
        {
            if (!face->Valid)
            {
                continue;
            }
            Faces[validFace++] = face;
        }
		reducedFaceNum = validFace;

		for (int fIdx = 0, faceIndex = 0; fIdx < validFace; ++fIdx, faceIndex += 3)
        {
            auto f = Faces[fIdx];

			reducedIndices[faceIndex] = f->Vertices[0]->LOCAL_VERTEX_MARK;
			reducedIndices[faceIndex + 1] = f->Vertices[1]->LOCAL_VERTEX_MARK;
			reducedIndices[faceIndex + 2] = f->Vertices[2]->LOCAL_VERTEX_MARK;
        }

    }

    bool MeshReducerPrivate::isNonClosedMesh() const
    {
        return m_bStrictConstraint;
    }

    void MeshReducerPrivate::buildQuadricMatrix()
    {
        SymetricMatrix Q(true);
        for (auto &face : Faces)
        {
            // ax + by + cz + d = 0
            Vec3 normal = face->normal();
            if (!m_bStrictConstraint)
            {
                normal = normal.normalize();
                face->InitUnitNormal = normal;
            }
            else //strict mode
            {
                int mask = face->Edges[0]->LOCAL_EDGE_MARK | face->Edges[1]->LOCAL_EDGE_MARK | face->Edges[2]->LOCAL_EDGE_MARK;
                if (mask > 0)
                {
                    for (unsigned int j = 0; j < 3; ++j)
                    {
                        auto e = face->Edges[j];
                        if (e->LOCAL_EDGE_MARK == 1)
                        {
                            //stats show that direction does not matter
                            auto start = e->Start;
                            auto end = e->End;

                            const Vec3 &pStart = start->Pos;
                            const Vec3 &pEnd = end->Pos;

                            const Vec3 &edgePlane = normal.cross((pEnd - pStart).normalize());
                            double offset = -edgePlane.dot(pStart);


                            Q.syncByNormalD(edgePlane, offset);

                            // add to related vertices
                            start->Q += Q;
                            end->Q += Q;
                        }
                    }
                }
            }

            double d = -normal.dot(face->Vertices[0]->Pos);
            // assemble quadric matrix
            Q.syncByNormalD(normal, d);
            for (int i = 0; i < 3; ++i)
            {
                face->Vertices[i]->Q += Q;
            }
        }

    }

    void MeshReducerPrivate::initCollapses()
    {
		mHelper.GLOBAL_MARK = 2;//start from 2 as edge local mark might be 0, 1;
        if (m_bStrictConstraint)
        {
            for (auto &e : Edges)
            {
                e->ToBeReplaced = nullptr;
                e->Start->ToUpdateEdge = nullptr;
                e->End->ToUpdateEdge = nullptr;
                e->SquareDistance = Vec3::squaredLength(e->Start->Pos, e->End->Pos);
            }

            for (auto &edge : Edges)
            {
				mHelper.solve(*edge);
            }
        }
        else
        {
            for (auto &e : Edges)
            {
                e->ToBeReplaced = nullptr;
				mHelper.calcOptimalPosition_ForFace(*e, e->Priority);
            }

            for (auto &face : Faces)
            {
				//compute the initial optEdge here
				mHelper.computeFaceOptEdge(*face);
            }
        }
    }

    void MeshReducerPrivate::doFastLoop(unsigned int nTarget)
    {
        unsigned int faceCount = static_cast <unsigned int> (Faces.size());
        unsigned int nowCount = faceCount;


        for (unsigned int iter = 0; iter < MAX_ITERATION; ++iter)
        {
            double threshold = FaceIterThresholds[iter];


            int deletePerRound = 0;
            unsigned int currGlobalMark = mHelper.GLOBAL_MARK;
            for (auto &face : Faces)
            {
                // get the top heap element
                if (!face->Valid || face->LOCAL_FACE_MARK > currGlobalMark || face->Edges[face->OptEdge]->Priority > threshold)
                {
                    continue;
                }

                // update
                unsigned int nDeleted = mHelper.updateFace(*face);
                deletePerRound += nDeleted;

                nowCount -= nDeleted;
                if (nowCount <= nTarget)
                {
                    return;
                }
            }

			}
        }

    void MeshReducerPrivate::doStrictLoop(unsigned int nTarget)
    {
        unsigned int faceCount = static_cast<unsigned int>(Faces.size());
        unsigned int nowCount = faceCount;
        EdgeHeap heap(Edges);

        // clock
        auto start = std::chrono::steady_clock::now();
        auto end = start;

        // collapse in loop
        while ((nowCount > nTarget) && heap.Length != 0 &&
            std::chrono::duration_cast<std::chrono::seconds>(end - start).count() < TIMEOUT)
        {
            // get top element
            Edge &top = *heap.top();

            if (top.Start->Removed || top.End->Removed)
            {
                continue;
            }

            if (top.Priority == std::numeric_limits<double>::infinity()) {
                return;
            }

            // update time
            end = std::chrono::steady_clock::now();
            // update
            unsigned int nDeleted = mHelper.updateEdge(heap, top);
            nowCount -= nDeleted;
        }
    }

    void MeshReducerPrivate::cleanUp()
    {
        int nValidVert = 0;
        for (auto &v : Vertices)
        {
            if (v->Removed)
            {
                continue;
            }
            if (v->Neighbors == nullptr) {
                //count++;
                continue;
            }
            Vertices[nValidVert++] = v;
        }

        if (nValidVert == 0)
        {
            Vertices.clear();
            Faces.clear();
            return;
        }

        Vertices.resize(nValidVert);
        std::sort(Vertices.begin(), Vertices.end(), Vertex::comparefunc);

        // remove duplicated vertex

        //LOCAL_MARK is used to store index reference in the pVecResult
        auto refV = Vertices[0];
        refV->LOCAL_VERTEX_MARK = 0;
        //mResultVertices[0]->Removed = true;
        //unsigned int nDeleted = 0;
        int removeCount = 0;
        int lastStoreIdx = 0;
        for (int i = 1; i < nValidVert; ++i)
        {
            auto currentV = Vertices[i];
            if (currentV->Pos == refV->Pos)
            {
                currentV->LOCAL_VERTEX_MARK = lastStoreIdx;
                removeCount++;
            }
            else
            {
                Vertices[++lastStoreIdx] = currentV;
                currentV->LOCAL_VERTEX_MARK = lastStoreIdx;
                refV = currentV;
            }
        }
        //up to this point, all v in vertices is valid
        Vertices.resize(nValidVert - removeCount);
    }

    bool MeshReducerPrivate::isValid() const
    {
        if (Vertices.empty())
        {
            return false;
        }

        Vec3 min(std::numeric_limits<double>::max());
        Vec3 max(std::numeric_limits<double>::min());

        min.X = Vertices[0]->Pos.X;
        max.X = Vertices.back()->Pos.X;

        for (auto vert : Vertices)
        {
            min.Y = std::min(min.Y, vert->Pos.Y);
            max.Y = std::max(max.Y, vert->Pos.Y);

            min.Z = std::min(min.Z, vert->Pos.Z);
            max.Z = std::max(max.Z, vert->Pos.Z);
        }

        double len_diag = (max - min).squaredLength();
        double minDiag = std::min(len_diag, OriginBBoxDiagonal);
        double maxDiag = std::max(len_diag, OriginBBoxDiagonal);
        bool valid = minDiag > VALID_THRESHOLD * maxDiag;
		
		return valid;
    }
} // namespace

namespace common
{
    static MeshReducerPrivate * reducer = nullptr;
    void MeshReducer::release() 
	{
		if (reducer != nullptr) 
		{
			reducer->reset();
			delete reducer;
			reducer = nullptr;
		}
    }
    bool MeshReducer::reduce(const float *vertices, const uint16_t *indices, unsigned int nVert, unsigned int nIdx,
		float *reducedVertices, int &reducedVertNum, uint16_t* reducedIndices, int &reducedFaceNum, 
        unsigned int nTarget)
    {
        if (vertices == nullptr || indices == nullptr || nVert == 0 || nIdx == 0 || nTarget == 0)
        {
            return false;
        }
		if (reducer == nullptr) 
		{
			reducer = new MeshReducerPrivate();
		}
        reducer->reset();
        reducer->load(vertices, indices, nVert, nIdx, false);
        reducer->reduce(nTarget);
        bool bValid = reducer->isValid();
        if (!bValid && !reducer->isNonClosedMesh())
        {
            // do again if the result is invalid and the mesh is non-manifold
            // force to go through the strict route
            reducer->reset();
            reducer->load(vertices, indices, nVert, nIdx, true);
            reducer->reduce(nTarget);
            bValid = reducer->isValid();
        }

        if (bValid)
            reducer->store(reducedVertices, reducedVertNum, reducedIndices, reducedFaceNum);
        return bValid;
    }




} // namespace common
