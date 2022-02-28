#include "dtkPhysTetraMassSpring.h" 
#ifdef DTK_DEBUG
    #define DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
#endif //DTK_DEBUG

#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
    #include <iostream>
    using namespace std;
#endif

using namespace std;

namespace dtk
{
	
    const dtkID dtkPhysTetraMassSpring::tetra_order[7][4]= {
        { 3, 2, 1, 0 }, // point to face
	    { 2, 3, 0, 1 },
		{ 1, 0, 3, 2 },
		{ 0, 1, 2, 3 },
		{ 0, 1, 3, 2 }, // edge to edge
		{ 0, 2, 1, 3 },
		{ 0, 3, 2, 1 }
    };
                
	
	dtkPhysTetraMassSpring::dtkPhysTetraMassSpring(bool fullUseAltSpring, double defaultMass, double defaultStiff, double defaultDamp, double defaultPointDamp, double defaultPointResistence, dtkDouble3 defaultGravityAccel )
        :dtkPhysMassSpring( defaultMass, defaultStiff, defaultDamp, defaultPointDamp, defaultPointResistence, defaultGravityAccel )
	{
        mAltitudeStiff = defaultStiff;
        mAltitudeDamp = defaultDamp;
        /*
        mAltitudePoint1 = dtkT3<T>(0,0,0);
        mAltitudePoint2 = dtkT3<T>(0,0,0);
        mForceA = dtkT3<T>(0,0,0);
        mForceB = dtkT3<T>(0,0,0);
        mForceC = dtkT3<T>(0,0,0);
        mForceD = dtkT3<T>(0,0,0);
        */
        mFullUseAltSpring = fullUseAltSpring; 
	}

	
	dtkPhysTetraMassSpring::~dtkPhysTetraMassSpring()
	{
#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
        cout << "[dtkPhysTetraMassSpring::~dtkPhysTetraMassSpring]" << endl;
        cout << "[/dtkPhysTetraMassSpring::~dtkPhysTetraMassSpring]" << endl;
        cout << endl;
#endif
	}

	
    bool dtkPhysTetraMassSpring::PreUpdate(double timeslice, ItrMethod method, dtkID iteration)
    {
        for(int i = 0;i < mTetraMesh->GetTetraNum();i++)
        {
            dtkID4 tempTetra = mTetraMesh->GetTetraById(i);
            if(tempTetra[0] != dtkErrorID) 
            {
                //有效id, 有效四面体

                dtkPhysMassPoint* massPoints[4];
                dtkT3<double> positions[4];
                dtkT3<double> vels[4];
                for( dtkID j = 0; j < 4; j++ )
                {
                    massPoints[j] = this->GetMassPoint( tempTetra[j] );
                    positions[j] = massPoints[j]->GetPosition( method, iteration );
                    vels[j] = massPoints[j]->GetVel( method, iteration );
                }

                /*
                if( GK::IsDegenerate( GK::Tetrahedron3( 
                                GK::Point3( positions[0][0], positions[0][1], positions[0][2] ),
                                GK::Point3( positions[1][0], positions[1][1], positions[1][2] ),
                                GK::Point3( positions[2][0], positions[2][1], positions[2][2] ),
                                GK::Point3( positions[3][0], positions[3][1], positions[3][2] )
                                ) ) )
                {
                    assert(false);
                    continue;
                }
                */

                dtkT3<double> n_area[7];
                bool in_use[7];

                // tracker to the max buttom-area
                double max_area = -1;
                dtkID max_order = 0;
                for( dtkID j = 0; j < 7; j++ )
                {
                    if( j < 4 )
                        n_area[j] = cross( 
                                positions[ tetra_order[j][1] ] - positions[ tetra_order[j][0] ],
                                positions[ tetra_order[j][2] ] - positions[ tetra_order[j][1] ] );
                    else
                        n_area[j] = cross( 
                                positions[ tetra_order[j][1] ] - positions[ tetra_order[j][0] ],
                                positions[ tetra_order[j][3] ] - positions[ tetra_order[j][2] ] );

                    if(mFullUseAltSpring)
                    {
                        in_use[j] = true;
                    }
					// add spring into the max cross within 7.
                    else
                    {
                        double area = length( n_area[j] );
                        if( area > max_area )
                        {
                            max_area = area;
                            in_use[ max_order ] = false;
                            max_order = j;
                            in_use[ max_order ] = true;
                        }
                        else
                        {
                            in_use[j] = false;
                        }
                    }
                }

				// one tetra has four kinds of one vertex to one triangle.
				// when in_use[i] is true, it present ith kind will be added spring.
                for( dtkID j = 0; j < 4; j++ )
                {
                    if( !in_use[j] )
                        continue;

#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
                    //cout << "Altitude Spring " << j << " in use." << endl;
#endif
					// compute the current distance of the point to the face in tetra.
                    dtkT3<double> v_side_edge = positions[ tetra_order[j][3] ] - positions[ tetra_order[j][1] ];
                    dtkT3<double> n_area_unit = normalize( n_area[j] );

                    double oriLength = mOriLengths[ i ][ j ]; //原本长度 
                    double curLength = dot( n_area_unit, v_side_edge); // 更新长度 

                    dtkT3<double> uvw = barycentricWeight( 
                            positions[ tetra_order[j][3] ], 
                            positions[ tetra_order[j][0] ], 
                            positions[ tetra_order[j][1] ], 
                            positions[ tetra_order[j][2] ] );
                    if(uvw[0] > 0 && uvw[0] < 1 && uvw[1] > 0 && uvw[1] < 1 && uvw[2] > 0 && uvw[2] < 1)
                    {
                        dtkT3<double> stiffForce = n_area_unit * ((oriLength - curLength) * mAltitudeStiff / oriLength ); //弹力 
						// compute the velocity of barycentric of tetra. 
                        dtkT3<double> vP = vels[ tetra_order[j][0] ] * uvw[0] 
                            + vels[ tetra_order[j][1] ] * uvw[1] 
                            + vels[ tetra_order[j][2] ] * uvw[2];
                        dtkT3<double> dampForce = - n_area_unit * ( 
                                dot( (vels[ tetra_order[j][3] ] - vP), n_area_unit ) 
                                * (timeslice * mAltitudeStiff / oriLength + mAltitudeDamp) );
                        dtkT3<double> force = stiffForce + dampForce;
                        massPoints[ tetra_order[j][3] ]->AddForce(force);
                        massPoints[ tetra_order[j][0] ]->AddForce(force * ( (-1.0) * uvw[0]) );
                        massPoints[ tetra_order[j][1] ]->AddForce(force * ( (-1.0) * uvw[1]) );
                        massPoints[ tetra_order[j][2] ]->AddForce(force * ( (-1.0) * uvw[2]) ); //力 
#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
                        //cout << "get altitude spring." << endl;
						// compute the two mass position of spring  
                        mAltitudeSpringPositions[i].first = positions[ tetra_order[j][3] ] - n_area_unit * curLength;
                        mAltitudeSpringPositions[i].second = positions[ tetra_order[j][3] ];
#endif
                    }
                }

				// one tetra has three kinds of one edge to one edge.
				// when in_use[i] is true, it present ith kind will be added spring.
                for( dtkID j = 4; j < 7; j++ )
                {
                    if( !in_use[j] )
                        continue;

#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
                    //cout << "Altitude Spring " << j << " in use." << endl;
#endif
                    dtkT3<double> v_side_edge = positions[ tetra_order[j][2] ] - positions[ tetra_order[j][0] ];
                    dtkT3<double> n_area_unit = normalize( n_area[j] );
                    
                    double oriLength = mOriLengths[ i ][ j ];
                    double curLength = dot( n_area_unit, v_side_edge);

                    dtkT3<double> v_edge_0 = positions[ tetra_order[j][1] ] - positions[ tetra_order[j][0] ];
                    dtkT3<double> v_edge_1 = positions[ tetra_order[j][3] ] - positions[ tetra_order[j][2] ];
                    dtkT3<double> n_edge_face_0 = cross( n_area_unit, v_edge_0 );
                    dtkT3<double> n_edge_face_1 = cross( v_edge_1, n_area_unit );
                        
                    double fraction_0 =  dot( n_edge_face_1, v_side_edge ) / dot( n_edge_face_1, v_edge_0 ); 
                    dtkT3<double> P0 = positions[ tetra_order[j][0] ] + v_edge_0 * fraction_0;
                    dtkT2<double> uv_0( 1.0 - fraction_0, fraction_0 );

                    double fraction_1 =  - dot( n_edge_face_0, v_side_edge ) / dot( n_edge_face_0, v_edge_1 ); 
                    dtkT3<double> P1 = positions[ tetra_order[j][2] ] + v_edge_1 * fraction_1;
                    dtkT2<double> uv_1( 1.0 - fraction_1, fraction_1 );

#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
                    //cout << uv_0 << " " << uv_1 << endl;
#endif
                    if( uv_0[0] >= 0 && uv_0[0] <= 1 && uv_0[1] >= 0 && uv_0[1] <= 1 
                            && uv_1[0] >= 0 && uv_1[0] <= 1 && uv_1[1] >= 0 && uv_1[1] <= 1)    
                    {
                        dtkT3<double> stiffForce = n_area_unit * ( (oriLength - curLength) * mAltitudeStiff / oriLength);
                        dtkT3<double> vP0 = vels[ tetra_order[j][0] ] * uv_0[0] + vels[ tetra_order[j][1] ] * uv_0[1];
                        dtkT3<double> vP1 = vels[ tetra_order[j][2] ] * uv_1[0] + vels[ tetra_order[j][3] ] * uv_1[1];

                        dtkT3<double> dampForce = - n_area_unit * ( 
                                dot( (vP1 - vP0), n_area_unit ) 
                                * (timeslice * mAltitudeStiff / oriLength + mAltitudeDamp) );
                        dtkT3<double> force = stiffForce + dampForce;
                        massPoints[ tetra_order[j][2] ]->AddForce(force * uv_1[0]);
                        massPoints[ tetra_order[j][3] ]->AddForce(force * uv_1[1]);
                        massPoints[ tetra_order[j][0] ]->AddForce(force * ( (-1.0) * uv_0[0]) );
                        massPoints[ tetra_order[j][1] ]->AddForce(force * ( (-1.0) * uv_0[1]) );
#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
                        //cout << "get altitude spring." << endl;
                        mAltitudeSpringPositions[i].first = P0;
                        mAltitudeSpringPositions[i].second = P1;
#endif
                    }
                }
            }
        }

        return true;
    }

	
	void dtkPhysTetraMassSpring::SetTetraMesh(dtkStaticTetraMesh::Ptr newTetraMesh)
	{
		mTetraMesh = newTetraMesh;

        dtkPoints::Ptr pts = mTetraMesh->GetPoints();

        this->SetPoints( pts );

        mOriPointsPtr = dtkPointsVector::New();
        for( dtkID i = 0; i < pts->GetNumberOfPoints(); i++ )
        {
            mOriPointsPtr->SetPoint(i,newTetraMesh->GetPoint(i));
            this->AddMassPoint(i, this->mDefaultMass, dtkDouble3(0,0,0), mDefaultPointDamp, mDefaultPointResistence, mDefaultGravityAccel );
        }
        const vector<dtkID4>& tempECTable = mTetraMesh->GetECTable();

#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
        cout<<"ectable size: "<<tempECTable.size()<<endl;
#endif

        for(dtkID i = 0;i < tempECTable.size();i++)
        {
            dtkID4 tempTetra = tempECTable[i];
                
            dtkT3<double> ori_positions[4];
            for( int x = 0; x < 4; x++ )
            {
                for( int y = x + 1; y < 4; y++ )
                {
					this->AddSpring( tempTetra[x], tempTetra[y], this->mDefaultStiff, this->mDefaultDamp );
					dtkID2 edge(tempTetra[x], tempTetra[y]);
					if( edge[0] > edge[2] )
						swap( edge[0], edge[2] );
					if( mSpringCount.find( edge ) != mSpringCount.end() )
						mSpringCount[edge]++;
					else
						mSpringCount[edge] = 1;
                }
                const GK::Point3& gk_oripoint = mOriPointsPtr->GetPoint( tempTetra[x] );
                ori_positions[x] = dtkT3<double>( gk_oripoint.x(), gk_oripoint.y(), gk_oripoint.z() );
            }

			mOriLengths.push_back( vector<double>() );
            // Back up original data for altitude spring
            for( dtkID j = 0; j < 7; j++ )
            {
                if( j < 4 )
                {//相对底面的高
                    dtkT3<double> ori_n_area = cross( 
                            ori_positions[ tetra_order[j][1] ] - ori_positions[ tetra_order[j][0] ],
                            ori_positions[ tetra_order[j][2] ] - ori_positions[ tetra_order[j][1] ] );
                    dtkT3<double> ori_v_side_edge = ori_positions[ tetra_order[j][3] ] - ori_positions[ tetra_order[j][1] ];
                    dtkT3<double> ori_n_area_unit = normalize( ori_n_area );

                    double oriLength = dot( ori_n_area_unit, ori_v_side_edge);
                    mOriLengths[i].push_back( oriLength );

                    
                }
                else
                {//相对边的距离
                    dtkT3<double> ori_n_area = cross( 
                                ori_positions[ tetra_order[j][1] ] - ori_positions[ tetra_order[j][0] ],
                                ori_positions[ tetra_order[j][3] ] - ori_positions[ tetra_order[j][2] ] );
                    dtkT3<double> ori_v_side_edge = ori_positions[ tetra_order[j][2] ] - ori_positions[ tetra_order[j][0] ];
                    dtkT3<double> ori_n_area_unit = normalize( ori_n_area );

                    double oriLength = dot( ori_n_area_unit, ori_v_side_edge);
                    mOriLengths[i].push_back( oriLength );
                    
                }
            }
            
#ifdef DTK_PHYSTETRAMASSSPRINGIMPL_DEBUG
            mAltitudeSpringPositions.push_back( pair< dtkT3<double>, dtkT3<double> >( dtkT3<double>(0,0,0), dtkT3<double>(0,0,0) ) );
#endif
        }
    }


	void dtkPhysTetraMassSpring::DeleteTetra( dtkID i )
	{
		dtkID4 tetra = mTetraMesh->GetECTable()[i];
		mOriLengths.erase( mOriLengths.begin() + i );
		for( int x = 0; x < 4; x++ )
		{
			for( int y = x + 1; y < 4; y++ )
			{
				dtkID2 edge(tetra[x], tetra[y]);
				if( edge[0] > edge[2] )
					swap( edge[0], edge[2] );
				mSpringCount[edge]--;
				if( mSpringCount[edge] == 0 )
				{
					DeleteSpring(tetra[x], tetra[y]);
				}
			}
		}
	}

	void dtkPhysTetraMassSpring::AddTetra( dtkID i )
	{
		assert( i == mTetraMesh->GetECTable().size() - 1 );
		dtkID4 tetra = mTetraMesh->GetECTable()[i];
		dtkT3<double> ori_positions[4];
		//dtkPoints::Ptr meshPts = mTetraMesh->GetPoints();
		
		// add springs into the tetra.
		for( int x = 0; x < 4; x++ )
		{
			for( int y = x + 1; y < 4; y++ )
			{
				this->AddSpring( tetra[x], tetra[y], this->mDefaultStiff, this->mDefaultDamp );
				dtkID2 edge(tetra[x], tetra[y]);
				if( edge[0] > edge[1] )
					swap( edge[0], edge[1] );
				if( mSpringCount.find( edge ) != mSpringCount.end() )
					mSpringCount[edge]++;
				else
					mSpringCount[edge] = 1;
			}
			const GK::Point3& gk_oripoint = mOriPointsPtr->GetPoint( tetra[x] );
			ori_positions[x] = dtkT3<double>( gk_oripoint.x(), gk_oripoint.y(), gk_oripoint.z() );
		}

		mOriLengths.push_back( vector<double>() );
		// Back up original data for altitude spring
		
		// when j<4 : compute the distance of point to face in tetra.
		// when j>=4: compute the distance of two edges in tetra.
		for( dtkID j = 0; j < 7; j++ )
		{
			if( j < 4 )
			{
				dtkT3<double> ori_n_area = cross( 
					ori_positions[ tetra_order[j][1] ] - ori_positions[ tetra_order[j][0] ],
					ori_positions[ tetra_order[j][2] ] - ori_positions[ tetra_order[j][1] ] );
				dtkT3<double> ori_v_side_edge = ori_positions[ tetra_order[j][3] ] - ori_positions[ tetra_order[j][1] ];
				dtkT3<double> ori_n_area_unit = normalize( ori_n_area );

				double oriLength = dot( ori_n_area_unit, ori_v_side_edge);
				mOriLengths[i].push_back( oriLength );
			}
			else
			{
				dtkT3<double> ori_n_area = cross( 
					ori_positions[ tetra_order[j][1] ] - ori_positions[ tetra_order[j][0] ],
					ori_positions[ tetra_order[j][3] ] - ori_positions[ tetra_order[j][2] ] );
				dtkT3<double> ori_v_side_edge = ori_positions[ tetra_order[j][2] ] - ori_positions[ tetra_order[j][0] ];
				dtkT3<double> ori_n_area_unit = normalize( ori_n_area );

				double oriLength = dot( ori_n_area_unit, ori_v_side_edge);
				mOriLengths[i].push_back( oriLength );
			}
		}
	}

	void dtkPhysTetraMassSpring::ReshapeTetra( dtkID i )
	{
		dtkID4 tetra = mTetraMesh->GetECTable()[i];
		dtkT3<double> ori_positions[4];
		//dtkPoints::Ptr meshPts = mTetraMesh->GetPoints();
		for( int x = 0; x < 4; x++ )
		{
			const GK::Point3& gk_oripoint = mOriPointsPtr->GetPoint( tetra[x] );
			ori_positions[x] = dtkT3<double>( gk_oripoint.x(), gk_oripoint.y(), gk_oripoint.z() );
		}

		for( dtkID j = 0; j < 7; j++ )
		{
			if( j < 4 )
			{
				dtkT3<double> ori_n_area = cross( 
					ori_positions[ tetra_order[j][1] ] - ori_positions[ tetra_order[j][0] ],
					ori_positions[ tetra_order[j][2] ] - ori_positions[ tetra_order[j][1] ] );
				dtkT3<double> ori_v_side_edge = ori_positions[ tetra_order[j][3] ] - ori_positions[ tetra_order[j][1] ];
				dtkT3<double> ori_n_area_unit = normalize( ori_n_area );

				double oriLength = dot( ori_n_area_unit, ori_v_side_edge);
				mOriLengths[i][j] = oriLength;
			}
			else
			{
				dtkT3<double> ori_n_area = cross( 
					ori_positions[ tetra_order[j][1] ] - ori_positions[ tetra_order[j][0] ],
					ori_positions[ tetra_order[j][3] ] - ori_positions[ tetra_order[j][2] ] );
				dtkT3<double> ori_v_side_edge = ori_positions[ tetra_order[j][2] ] - ori_positions[ tetra_order[j][0] ];
				dtkT3<double> ori_n_area_unit = normalize( ori_n_area );

				double oriLength = dot( ori_n_area_unit, ori_v_side_edge);
				mOriLengths[i][j] = oriLength;
			}
		}
	}
	//template<typename T>
	//dtkT3<T> dtkPhysTetraMassSpring<T>::GetBarycentricWeight(dtkT3<T> point1, dtkT3<T> point2, dtkT3<T> point3, dtkT3<T> point4)
 //   {
 //   // point4 is the projection node
 //       dtkT3<T> uvw;
 //       T detA = point1[0] * point2[1] * (point3[2] - point1[2]) + 
 //                   point3[0] * point1[1] * (point2[2] - point1[2]) - 
 //                   point1[0] * point3[1] * (point2[2] - point1[2]) - 
 //                   point2[0] * point1[1] * (point3[2] - point1[2]);
 //       if(detA == 0)
 //       {
 //           T D1 = (point2[2] - point1[2]) * (point3[0] - point1[0]) - (point3[2] - point1[2]) * (point2[0] - point1[0]);
 //           if(D1 != 0)
 //           {
 //               uvw[1] = ((point4[2] - point1[2]) * (point3[0] - point1[0]) - (point3[2] - point1[2]) * (point4[0] - point1[0])) / D1; 
 //               uvw[2] = ((point2[2] - point1[2]) * (point4[0] - point1[0]) - (point4[2] - point1[2]) * (point2[0] - point1[0])) / D1; 
 //               uvw[0] = 1 - uvw[1] - uvw[2];
 //           }
 //           else
 //           {
 //               T D2 = ((point2[1] - point1[1]) * (point3[0] - point1[0]) - (point3[1] - point1[1]) * (point2[0] - point1[0]));
 //               uvw[1] = ((point4[0] - point1[0]) * (point3[1] - point1[1]) - (point3[0] - point1[0]) * (point4[1] - point1[1])) / D2;
 //               uvw[2] = ((point2[0] - point1[0]) * (point4[1] - point1[1]) - (point4[0] - point1[0]) * (point2[1] - point1[1])) / D2;
 //               uvw[0] = 1 - uvw[1] - uvw[2];
 //           }
 //       }
 //       else
 //       {
 //           uvw[0] = (point4[0] * point2[1] * (point3[2] - point1[2]) + 
 //                   point2[0] * point3[1] * (point4[2] -point1[2]) + 
 //                   point3[0] * point4[1] * (point2[2] - point1[2]) -
 //                   point3[0] * point2[1] * (point4[2] - point1[2]) - 
 //                   point4[0] * point3[1] * (point2[2] - point1[2]) - 
 //                   point2[0] * point4[1] * (point3[2] - point1[2])) / detA;
 //           uvw[1] = (point1[0] * point4[1] * (point3[2] - point1[2]) + 
 //                   point3[0] * point1[1] * (point4[2] - point1[2]) -
 //                   point1[0] * point3[1] * (point4[2] - point1[2]) -
 //                   point4[0] * point1[1] * (point3[2] - point1[2])) / detA;
 //           uvw[2] = (point1[0] * point2[1] * (point4[2] - point1[2]) + 
 //                   point4[0] * point1[1] * (point2[2] - point1[2]) - 
 //                   point1[0] * point4[1] * (point2[2] - point1[2]) - 
 //                   point2[0] * point1[1] * (point4[2] - point1[2])) / detA;
 //       }

 //       return uvw;
 //   }


}

