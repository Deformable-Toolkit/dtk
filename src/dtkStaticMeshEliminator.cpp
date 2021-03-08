#include "dtkStaticMeshEliminator.h"
#include "dtkCollisionDetectPrimitive.h"

namespace dtk
{
    dtkStaticMeshEliminator::dtkStaticMeshEliminator()
    {
    }

    dtkStaticMeshEliminator::~dtkStaticMeshEliminator()
    {
    }

    void dtkStaticMeshEliminator::EliminateTriangles( dtkID id, size_t originalFaceNum, std::vector< dtkID3 >& collisionFaces, bool singleMethod )
    {
        dtkStaticTriangleMesh::Ptr trimesh = mTriangleMeshes[id];
        dtkStaticTetraMesh::Ptr tetramesh = mTetraMeshes[id];
        dtkCollisionDetectHierarchy::Ptr hierarchy = mCollisionDetectHierarchies[id];
		dtkPhysTetraMassSpring::Ptr tetraMassSpring = mTetraMassSprings[id];

		//��¼ԭ���ж����棬�����ж���Щ���Ǳ����ϵġ�
		size_t faceNum = trimesh->GetECTable().size();

		if( singleMethod )
		{
			for ( dtkID i = 0; i < collisionFaces.size(); i++ )
			{
				for ( dtkID j = 0; j < tetramesh->GetECTable().size(); j++ )
				{
					if ( Contains(tetramesh->GetECTable()[j], collisionFaces[i]) == true )
					{
						dtkID4 deleteTetra = tetramesh->GetECTable()[j];
						std::vector<dtkID3> surfaceDeleteFace, surfaceAddFace;
						bool findResult[4] = { false, false, false, false };
						for ( dtkID k = 0; k < trimesh->GetECTable().size(); k++ )
						{
							if ( IsIdentical( dtkID3( deleteTetra.a, deleteTetra.c, deleteTetra.b ), trimesh->GetECTable()[k] ) == true )
							{
								findResult[0] = true;
                                surfaceDeleteFace.push_back(trimesh->GetECTable()[k]);
								trimesh->RemoveTriangle(trimesh->GetECTable()[k]);
								if (k < faceNum) faceNum--;
								if (k < originalFaceNum) originalFaceNum--;
								k--;
							}
							else if ( IsIdentical( dtkID3( deleteTetra.a, deleteTetra.b, deleteTetra.d ), trimesh->GetECTable()[k] ) == true )
							{
								findResult[1] = true;
                                surfaceDeleteFace.push_back(trimesh->GetECTable()[k]);
								trimesh->RemoveTriangle(trimesh->GetECTable()[k]);
								if (k < faceNum) faceNum--;
								if (k < originalFaceNum) originalFaceNum--;
								k--;
							}
							else if ( IsIdentical( dtkID3( deleteTetra.a, deleteTetra.d, deleteTetra.c ), trimesh->GetECTable()[k] ) == true )
							{
								findResult[2] = true;
                                surfaceDeleteFace.push_back(trimesh->GetECTable()[k]);
								trimesh->RemoveTriangle(trimesh->GetECTable()[k]);
								if (k < faceNum) faceNum--;
								if (k < originalFaceNum) originalFaceNum--;
								k--;
							}
							else if ( IsIdentical( dtkID3( deleteTetra.b, deleteTetra.c, deleteTetra.d ), trimesh->GetECTable()[k] ) == true )
							{
								findResult[3] = true;
                                surfaceDeleteFace.push_back(trimesh->GetECTable()[k]);
								trimesh->RemoveTriangle(trimesh->GetECTable()[k]);
								if (k < faceNum) faceNum--;
								if (k < originalFaceNum) originalFaceNum--;
								k--;
							}
						}

						if ( findResult[0] == false )
						{
							trimesh->InsertTriangle( deleteTetra.a, deleteTetra.b, deleteTetra.c );
							surfaceAddFace.push_back( dtkID3( deleteTetra.a, deleteTetra.b, deleteTetra.c ) );
						}
						if ( findResult[1] == false )
						{
							trimesh->InsertTriangle( deleteTetra.a, deleteTetra.d, deleteTetra.b );
							surfaceAddFace.push_back( dtkID3( deleteTetra.a, deleteTetra.d, deleteTetra.b ) );
						}
						if ( findResult[2] == false )
						{
							trimesh->InsertTriangle( deleteTetra.a, deleteTetra.c, deleteTetra.d );
							surfaceAddFace.push_back( dtkID3( deleteTetra.a, deleteTetra.c, deleteTetra.d ) );
						}
						if ( findResult[3] == false )
						{
							trimesh->InsertTriangle( deleteTetra.b, deleteTetra.d, deleteTetra.c );
							surfaceAddFace.push_back( dtkID3( deleteTetra.b, deleteTetra.d, deleteTetra.c ) );
						}
						tetraMassSpring->DeleteTetra( j );
						tetramesh->RemoveTetra( tetramesh->GetECTable()[j] );
						//if ( mMeshEliminatorDeleteTetraCallback.find(id) != mMeshEliminatorDeleteTetraCallback.end() )
						//{
						//    ( mMeshEliminatorDeleteTetraCallback[id] )( id, j );
						//}
						dtkPoints::Ptr pts = trimesh->GetPoints();
						ChangeHierarchy( hierarchy, pts, collisionFaces[i], surfaceAddFace, surfaceDeleteFace );

						break;
					}
				}
			}
		}
		else
		{
			//��collisionFaces����Ԥ����ʹ������֮�䲻������ͬ�ĵ�
			for ( dtkID i = 0; i < collisionFaces.size() - 1; i++ )
			{
				for ( dtkID j = i + 1; j < collisionFaces.size(); j++ )
				{
					if ( Intersected( collisionFaces[i], collisionFaces[j] ) == true )
					{
						collisionFaces.erase( collisionFaces.begin() + j );
						j--;
					}
				}
			}

			//����ÿһ��collisionFace
			for ( dtkID i = 0; i < collisionFaces.size(); i++ )
			{
				//��¼ɾ������Щ�����ϵ���
				std::vector<dtkID3> surfaceDeleteFaceI, surfaceAddFaceI;
				//����������ϵ�������
				for ( dtkID j = 0; j < 3; j++ )
				{
					//����ÿһ��������
					for ( dtkID k = 0; k < tetramesh->GetECTable().size(); k++ )
					{
						//�����������庬��collisionFaces[i]�ϵĵ㣬���������Ӧ�ñ�ɾ����
						//����������Ӧ���ĸ��棬����ڱ����ϣ���ɾ����������ڱ����ϣ���ӵ������ϡ�
						//��ײ�������ɾ������ɾ�����棬����collisionFaces[i]������µ��档
						if( tetramesh->GetECTable()[k].a == collisionFaces[i][j] || 
							tetramesh->GetECTable()[k].b == collisionFaces[i][j] || 
							tetramesh->GetECTable()[k].c == collisionFaces[i][j] || 
							tetramesh->GetECTable()[k].d == collisionFaces[i][j] )
						{
							bool findTriangle[4] = { false, false, false, false };//��¼����������Щ�汻�ҵ��ˣ���Щû�б��ҵ���
							dtkID4 deleteTetra = tetramesh->GetECTable()[k];
							for ( dtkID m = 0; m < trimesh->GetECTable().size(); m++ )
							{
								//����ҵ�����ɾ����
								if ( IsIdentical( dtkID3( deleteTetra.a, deleteTetra.c, deleteTetra.b ), trimesh->GetECTable()[m] ) == true )
								{
									findTriangle[0] = true;
									if ( m < originalFaceNum ) originalFaceNum--;
									surfaceDeleteFaceI.push_back( dtkID3( deleteTetra.a, deleteTetra.c, deleteTetra.b ) );
									if ( m < faceNum ) faceNum--;
									trimesh->RemoveTriangle( trimesh->GetECTable()[m] );
									m--;
								}
								else if ( IsIdentical( dtkID3( deleteTetra.a, deleteTetra.b, deleteTetra.d ), trimesh->GetECTable()[m] ) == true )
								{
									findTriangle[1] = true;
									if ( m < originalFaceNum ) originalFaceNum--;
									surfaceDeleteFaceI.push_back( dtkID3( deleteTetra.a, deleteTetra.b, deleteTetra.d ) );
									if ( m < faceNum ) faceNum--;
									trimesh->RemoveTriangle( trimesh->GetECTable()[m] );
									m--;
								}
								else if ( IsIdentical( dtkID3( deleteTetra.a, deleteTetra.d, deleteTetra.c ), trimesh->GetECTable()[m] ) == true )
								{
									findTriangle[2] = true;
									if ( m < originalFaceNum ) originalFaceNum--;
									surfaceDeleteFaceI.push_back( dtkID3( deleteTetra.a, deleteTetra.d, deleteTetra.c ) );
									if ( m < faceNum ) faceNum--;
									trimesh->RemoveTriangle( trimesh->GetECTable()[m] );
									m--;
								}
								else if ( IsIdentical( dtkID3( deleteTetra.b, deleteTetra.c, deleteTetra.d ), trimesh->GetECTable()[m] ) == true )
								{
									findTriangle[3] = true;
									if ( m < originalFaceNum ) originalFaceNum--;
									surfaceDeleteFaceI.push_back( dtkID3( deleteTetra.b, deleteTetra.c, deleteTetra.d ) );
									if ( m < faceNum ) faceNum--;
									trimesh->RemoveTriangle( trimesh->GetECTable()[m] );
									m--;
								}
							}

							//���û�ҵ�������ӵ����档
							if ( findTriangle[0] == false )
							{
								trimesh->InsertTriangle( dtkID3( deleteTetra.a, deleteTetra.b, deleteTetra.c ) );
								surfaceAddFaceI.push_back( dtkID3( deleteTetra.a, deleteTetra.b, deleteTetra.c ) );
							}
							if ( findTriangle[1] == false )
							{
								trimesh->InsertTriangle( dtkID3( deleteTetra.a, deleteTetra.d, deleteTetra.b ) );
								surfaceAddFaceI.push_back( dtkID3( deleteTetra.a, deleteTetra.d, deleteTetra.b ) );
							}
							if ( findTriangle[2] == false )
							{
								trimesh->InsertTriangle( dtkID3( deleteTetra.a, deleteTetra.c, deleteTetra.d ) );
								surfaceAddFaceI.push_back( dtkID3( deleteTetra.a, deleteTetra.c, deleteTetra.d ) );
							}
							if ( findTriangle[3] == false )
							{
								trimesh->InsertTriangle( dtkID3( deleteTetra.b, deleteTetra.d, deleteTetra.c ) );
								surfaceAddFaceI.push_back( dtkID3( deleteTetra.b, deleteTetra.d, deleteTetra.c ) );
							}
							tetraMassSpring->DeleteTetra( k );
							tetramesh->RemoveTetra( tetramesh->GetECTable()[k] );
							k--;
						}
					}
				}

				for( dtkID j = i + 1; j < collisionFaces.size(); j++ )
				{
					for( dtkID k = 0; k < surfaceDeleteFaceI.size(); k++ )
					{
						if( IsIdentical( surfaceDeleteFaceI[k], collisionFaces[j] ) )
						{
							collisionFaces.erase( collisionFaces.begin() + j );
							j--;
							break;
						}
					}
				}
				//�ı���ײ�����
				ChangeHierarchy( hierarchy, trimesh->GetPoints(), collisionFaces[i], surfaceAddFaceI, surfaceDeleteFaceI );
			}
		}

        //surfaceAddFace������ӵı���
        std::vector< dtkID3 > surfaceAddFace;
        for ( size_t i = faceNum; i < trimesh->GetECTable().size(); i++ )
        {
            surfaceAddFace.push_back( trimesh->GetECTable()[i] );
        }

        GroupModel(hierarchy);

        //�ص�����
        if ( mMeshEliminatorResultsCallback.find(id) != mMeshEliminatorResultsCallback.end() && mMeshEliminatorResultsCallback[id] != 0 )
        {
            ( mMeshEliminatorResultsCallback[id] )( id, originalFaceNum, surfaceAddFace, mMeshEliminatorResultsCallbackContext[id] );
        }
    }

    void dtkStaticMeshEliminator::ChangeHierarchy( dtkCollisionDetectHierarchy::Ptr hierarchy, dtkPoints::Ptr pts, dtkID3 collisionFace, const std::vector<dtkID3>& addFace, const std::vector<dtkID3>& deleteFace )
    {
        //�ҵ�collisionFace����addFace��ӽ�ȥ��
        for ( dtkID i = 0; i < hierarchy->GetNumberOfNodes(); i++ )
        {
            dtkCollisionDetectNode* nodeTemp = hierarchy->GetNode( i );
            for ( dtkID j = 0; j < nodeTemp->GetNumOfPrimitives(); j++ )
            {
                dtkCollisionDetectPrimitive *priDelete = nodeTemp->GetPrimitive( j );
                if ( nodeTemp->IsLeaf() && IsIdentical( collisionFace, dtkID3(priDelete->mIDs[0], priDelete->mIDs[1], priDelete->mIDs[2]) ) == true )
                {
                    for (dtkID k = 0; k < addFace.size(); k++)
                    {
                        dtkCollisionDetectPrimitive *priAdd;
                        priAdd = hierarchy->InsertTriangle( pts, addFace[k] );
                        priAdd->mMajorID = priDelete->mMajorID;
						priAdd->mDetailIDs[0] = addFace[k][0];
						priAdd->mDetailIDs[1] = addFace[k][1];
						priAdd->mDetailIDs[2] = addFace[k][2];
                        nodeTemp->AddPrimitive(priAdd);
                    }
                }
            }
        }

        //ɾ��deleteFace�е��档
        for ( dtkID i = 0; i < hierarchy->GetNumberOfNodes(); i++ )
        {
            dtkCollisionDetectNode* nodeTemp = hierarchy->GetNode( i );
            for ( dtkID j = 0; j < nodeTemp->GetNumOfPrimitives(); j++ )
            {
                dtkCollisionDetectPrimitive *priDelete = nodeTemp->GetPrimitive( j );
                for (dtkID k = 0; k < deleteFace.size(); k++)
                {
                    if ( nodeTemp->IsLeaf() && IsIdentical( deleteFace[k], dtkID3(priDelete->mIDs[0], priDelete->mIDs[1], priDelete->mIDs[2]) ) == true )
                    {
                        nodeTemp->DeletePrimitive( priDelete );
                        j--;
                    }
                }
            }
        }
    }

    void dtkStaticMeshEliminator::GroupModel(dtkCollisionDetectHierarchy::Ptr hierarchy)
    {
        
    }

    bool dtkStaticMeshEliminator::IsIdentical( dtkID3 x, dtkID3 y )
    {
        if ( ( x.a == y.a && x.b == y.b && x.c == y.c ) ||
             ( x.a == y.c && x.b == y.a && x.c == y.b ) ||
             ( x.a == y.b && x.b == y.c && x.c == y.a ) )
        {
            return true;
        }
        return false;
    }

    bool dtkStaticMeshEliminator::Contains( dtkID4 x, dtkID3 y )
    {
        if ( ( x.a == y.a && x.c == y.b && x.b == y.c ) ||
             ( x.a == y.c && x.c == y.a && x.b == y.b ) ||
             ( x.a == y.b && x.c == y.c && x.b == y.a ) ||

             ( x.a == y.a && x.b == y.b && x.d == y.c ) ||
             ( x.a == y.c && x.b == y.a && x.d == y.b ) ||
             ( x.a == y.b && x.b == y.c && x.d == y.a ) ||

             ( x.a == y.a && x.d == y.b && x.c == y.c ) ||
             ( x.a == y.c && x.d == y.a && x.c == y.b ) ||
             ( x.a == y.b && x.d == y.c && x.c == y.a ) ||

             ( x.b == y.a && x.c == y.b && x.d == y.c ) ||
             ( x.b == y.c && x.c == y.a && x.d == y.b ) ||
             ( x.b == y.b && x.c == y.c && x.d == y.a ) )
        {
            return true;
        }
        return false;
    }

    bool dtkStaticMeshEliminator::Intersected( dtkID3 x, dtkID3 y )
    {
        for (int i = 0; i < 3; i++ )
        {
            for (int j = 0; j < 3; j++ )
            {
                if (x[i] == y[j])
                {
                    return true;
                }
            }
        }
        return false;
    }
}