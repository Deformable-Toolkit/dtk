#ifndef CUTHEAD_H
#define  CUTHEAD_H

#include <vector>
#include "dtkTx.h"
#include "dtkStaticTriangleMesh.h"

namespace dtk
{
	class cutHead
	{
	public:
		cutHead(dtkStaticTriangleMesh::Ptr triangleMesh, const std::vector<dtkID> & avoidPoint);
		const std::vector<dtkID3> & AvoidTriangle();

	private:
		dtkStaticTriangleMesh::Ptr mTriangleMesh;
		std::vector<dtkID> mAvoidPoint; 
		std::vector<dtkID3> mAvoidTriangle; 
		
	};
}

#endif