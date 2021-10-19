#include "vascularOpenGLRender.h"


//#pragma comment(lib, "glew32.lib")

namespace dtk
{
	void vascularOpenGLRender::SetTriangleMesh(dtkStaticTriangleMesh::Ptr ptr)
	{
		mTriangleMeshPtr = ptr;
	}
	void vascularOpenGLRender::Draw()
	{
		if ( ! mTriangleMeshPtr.get() )
			return ;
		Update();
		const std::vector<dtkID3>& triFacets = mTriangleMeshPtr->GetECTable(); 
		
		// 把导丝的材质属性修改成血管的材质属性
		GLfloat mVascular_mat_ambient[] = {1.0, 0.0, 0.0, 1.0};
		GLfloat mVascular_mat_diffuse[] = {1.0, 0.0, 0.0, 1.0};
		GLfloat mVascular_mat_specular[] = {1.0, 1.0, 1.0, 1.0};
		GLfloat mVascular_mat_shininess = 1;

		glMaterialfv(GL_FRONT, GL_AMBIENT, mVascular_mat_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mVascular_mat_diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, mVascular_mat_specular);
		glMaterialf(GL_FRONT, GL_SHININESS, mVascular_mat_shininess);

		glEnable(GL_LIGHT0);
		glDisable(GL_LIGHT1);
		
		glVertexPointer( 3, GL_DOUBLE, 0, &( mPoints[0].x ) );
		glNormalPointer( GL_DOUBLE, 0, &( mNormals[0].x ) );
		glDrawElements(GL_TRIANGLES, triFacets.size() * 3, GL_UNSIGNED_INT, &(triFacets[0].a));
	}

	void vascularOpenGLRender::Update()
	{
		if( mTriangleMeshPtr.get() )
		{
			const std::vector<dtkID3>& triFacets = mTriangleMeshPtr->GetECTable(); 
			mPoints.clear();
			mNormals.clear();
			mNormals.resize( mTriangleMeshPtr->GetPoints()->GetNumberOfPoints() );
			for( dtkID i = 0; i < mTriangleMeshPtr->GetPoints()->GetNumberOfPoints(); i++ )
			{
				GK::Point3 point = mTriangleMeshPtr->GetPoint( i );
				mPoints.push_back( dtkT3<double>( point.x(), point.y(), point.z() ) );
			}
			for( dtkID i = 0; i < triFacets.size(); i++ )
			{
				dtkT3<double> p1 = mPoints[triFacets[i][0]];
				dtkT3<double> p2 = mPoints[triFacets[i][1]];
				dtkT3<double> p3 = mPoints[triFacets[i][2]];

				dtkT3<double> normal = cross( p1 - p2, p3 - p2 );
				mNormals[triFacets[i][0]] = mNormals[triFacets[i][0]] + normal;
				mNormals[triFacets[i][1]] = mNormals[triFacets[i][1]] + normal;
				mNormals[triFacets[i][2]] = mNormals[triFacets[i][2]] + normal;
			}
			for( dtkID i = 0; i < mNormals.size(); i++ )
			{
				if (mNormals[i].x == 0 && mNormals[i].y == 0 && mNormals[i].z == 0)
				{
					double nValue = std::sqrt(1/3.0);
					mNormals[i] = dtkT3<double>(nValue, nValue, nValue); 
				}
				else
 				mNormals[i] = normalize( mNormals[i] );
			}
		}
	}

	vascularOpenGLRender::~vascularOpenGLRender()
	{
		// nothing;
	}
	vascularOpenGLRender::vascularOpenGLRender()
	{
	}
}