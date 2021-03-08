#ifndef DTK_STATICTRIANGLEMESHWRITER_H
#define DTK_STATICTRIANGLEMESHWRITER_H

#include "dtkStaticTriangleMesh.h"

#include <cstring>

#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
	class dtkStaticTriangleMeshWriter: boost::noncopyable
	{
	public:
		
		typedef std::shared_ptr<dtkStaticTriangleMeshWriter> Ptr;

		static dtkStaticTriangleMeshWriter::Ptr New()
		{
			return dtkStaticTriangleMeshWriter::Ptr(new dtkStaticTriangleMeshWriter());
		}

	public:
		virtual ~dtkStaticTriangleMeshWriter() {}

		void SetInput(dtkStaticTriangleMesh::Ptr mesh);
		void SetFileName(const char *filePath);

		bool Write();

	private:
		dtkStaticTriangleMeshWriter();

		dtkStaticTriangleMesh::Ptr	mMesh;
		std::string	    			mFilePath;
	};
}

#endif //DTK_STATICTRIANGLEMESHWRITER_H

