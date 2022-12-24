/****************************************************************************
 * Mesh smoothing                                                            *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 ****************************************************************************/

/*! \file trimesh_smooth.cpp
\ingroup code_sample

\brief the minimal example of using the lib

This file contain a minimal example of the library

*/

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>

// input output
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/ply/plylib.cpp>

using namespace vcg;
using namespace std;

class MyFace;
class MyVertex;

struct MyUsedTypes : public UsedTypes<Use<MyVertex>::AsVertexType, Use<MyFace>::AsFaceType>
{
};
class MyVertex : public Vertex<MyUsedTypes, vertex::VFAdj, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags>
{
};
class MyFace : public Face<MyUsedTypes, face::VFAdj, face::Normal3f, face::VertexRef, face::BitFlags>
{
};
class MyMesh : public vcg::tri::TriMesh<vector<MyVertex>, vector<MyFace>>
{
};

int main(int argc, char **argv)
{
    while (1)
    {
        usleep(1000);
    }
    if (argc < 6)
    {
        printf("Usage: trimesh_smooth <input_file> <output_file> <steps> <normal_steps> <sigma> <fitstep>\n");
        return 0;
    }

    int Step = atoi(argv[3]);

    MyMesh m;

    // open a mesh
    int err = tri::io::Importer<MyMesh>::Open(m, argv[1]);
    if (err)
    { // all the importers return 0 in case of success
        printf("Error in reading %s: '%s'\n", argv[1], tri::io::Importer<MyMesh>::ErrorMsg(err));
        exit(-1);
    }

    // some cleaning to get rid of bad file formats like stl that duplicate vertexes..
    // int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(m);
    // int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(m);
    // printf("Removed %i duplicate and %i unreferenced vertices from mesh %s\n", dup, unref, argv[1]);

    tri::UpdateTopology<MyMesh>::VertexFace(m);

    for (int i = 0; i < Step; ++i)
    {
        tri::UpdateNormal<MyMesh>::PerFaceNormalized(m);
        tri::Smooth<MyMesh>::VertexCoordPasoDoble(m, atoi(argv[4]), atof(argv[5]), atoi(argv[6]));
    }

    // LaplacianSmooth(m,atoi(argv[2]));
    tri::io::ExporterPLY<MyMesh>::Save(m, argv[2]);

    return 0;
}
