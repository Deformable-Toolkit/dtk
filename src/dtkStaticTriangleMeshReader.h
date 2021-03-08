#ifndef DTK_STATICTRIANGLEMESHREADER_H
#define DTK_STATICTRIANGLEMESHREADER_H

#include "dtkStaticTriangleMesh.h"

#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
	class dtkStaticTriangleMeshReader: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkStaticTriangleMeshReader> Ptr;

		static dtkStaticTriangleMeshReader::Ptr New()
		{
			return dtkStaticTriangleMeshReader::Ptr(new dtkStaticTriangleMeshReader());
		}

	public:
		virtual ~dtkStaticTriangleMeshReader() {}

		void SetOutput(dtkStaticTriangleMesh::Ptr mesh);
		void SetFileName(const char* filePath);

		bool Read();

	private:
		dtkStaticTriangleMeshReader();

		dtkStaticTriangleMesh::Ptr	mMesh;
		std::string 				mFilePath;
	};
}

#endif //DTK_STATICTRIANGLEMESHREADER_H

