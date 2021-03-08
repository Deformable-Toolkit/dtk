#ifndef DTK_STATICTETRAMESHWRITER_H
#define DTK_STATICTETRAMESHWRITER_H

#include "dtkStaticTetraMesh.h"

#include <cstring>

#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
	class dtkStaticTetraMeshWriter: public boost::noncopyable
	{
	public:

		typedef std::shared_ptr<dtkStaticTetraMeshWriter> Ptr;

		static dtkStaticTetraMeshWriter::Ptr New()
		{
			return dtkStaticTetraMeshWriter::Ptr(new dtkStaticTetraMeshWriter());
		}

	public:
		virtual ~dtkStaticTetraMeshWriter() {}

		void SetInput(dtkStaticTetraMesh::Ptr mesh);
		void SetFileName(const char* filePath);

		bool Write();

	private:
		dtkStaticTetraMeshWriter();

		dtkStaticTetraMesh::Ptr	mMesh;
		std::string			mFilePath;
	};
}

#endif //DTK_STATICTETRAMESHWRITER_H

